/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Middleware.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/sensor/RealisticFovSensor.h"
#include "artery/envmod/sensor/SensorDetection.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObstacle.h"
#include <boost/geometry/geometries/register/linestring.hpp>
#include <omnetpp/ccomponent.h>
#include <unordered_set>

using namespace omnetpp;

using LineOfSight = std::array<artery::Position, 2>;
BOOST_GEOMETRY_REGISTER_LINESTRING(LineOfSight)

namespace artery
{

RealisticFovSensor::RealisticFovSensor() :
    mGroupFigure(nullptr), mSensorConeFigure(nullptr), mLinesOfSightFigure(nullptr),
    mObjectsFigure(nullptr), mObstaclesFigure(nullptr)
{
}

void RealisticFovSensor::finish()
{
    if (mGroupFigure) {
        delete mGroupFigure->removeFromParent();
        mGroupFigure = nullptr;
    }
    BaseSensor::finish();
}

void RealisticFovSensor::initialize()
{
    BaseSensor::initialize();

    std::string groupName = getEgoId();
    if (groupName.empty()) {
        const cModule* host = getMiddleware().getIdentity().host;
        assert(host);
        groupName += host->getName();
    }
    groupName += "-" + getSensorName();
    mGroupFigure = new cGroupFigure(groupName.c_str());
    mGlobalEnvironmentModel->getCanvas()->addFigure(mGroupFigure);
    mColor = cFigure::GOOD_DARK_COLORS[getId() % cFigure::NUM_GOOD_DARK_COLORS];

    mFovConfig.egoID = getEgoId();
    mFovConfig.sensorID = getId();
    mFovConfig.sensorPosition = determineSensorPosition(par("attachmentPoint"));

    mFovConfig.fieldOfView.range = par("fovRange").doubleValue() * boost::units::si::meters;
    mFovConfig.fieldOfView.angle = par("fovAngle").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.angleError = par("noiseVarAngle").doubleValue();
    mFovConfig.fieldOfView.rangeError = par("noiseVarRange").doubleValue();
    mFovConfig.numSegments = par("numSegments");
    mFovConfig.doLineOfSightCheck = par("doLineOfSightCheck");

    initializeVisualization();
}

void RealisticFovSensor::measurement()
{
    Enter_Method("measurement");
    auto detection = detectObjects();
    mLocalEnvironmentModel->complementObjects(detection, *this);
    mLastDetection = std::move(detection);
}

SensorDetection RealisticFovSensor::detectObjects() const
{
    namespace bg = boost::geometry;
    if (mFovConfig.fieldOfView.range <= 0.0 * boost::units::si::meter) {
        throw std::runtime_error("sensor range is 0 meter or less");
    } else if (mFovConfig.fieldOfView.angle > 360.0 * boost::units::degree::degrees) {
        throw std::runtime_error("sensor opening angle exceeds 360 degree");
    }

    SensorDetection detection = createSensorCone();
    auto preselObjectsInSensorRange = mGlobalEnvironmentModel->preselectObjects(mFovConfig.egoID, detection.sensorCone);
    for (const auto& object : preselObjectsInSensorRange)
    {
        std::vector<Position> newNoisyOutlineVector;
        for (const auto& objectPoint : object->getOutline())
        {        
            double angleError = normal(0, mFovConfig.fieldOfView.angleError, 2);//get random noise for sensor
            double rangeError = normal(0, mFovConfig.fieldOfView.rangeError, 2);
            double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
            double xError = abs(rangeError) * cos(angleErrorRad);//convert polar coordinates
            double yError = abs(rangeError) * sin(angleErrorRad);
            Position newNoisyPosition(objectPoint.x.value() + xError, objectPoint.y.value() + yError);//create new objectPoints with noise added
            //std::cout << "x err: " << xError << "; y err: " << yError << "\n";
            //std::cout << "x: " << objectPoint.x.value() << "; y: " << objectPoint.y.value() << "\n";
            newNoisyOutlineVector.push_back(newNoisyPosition);//add new, noisy objectPoint to NoisyOutlineVector                
        }
        object->setNoisyOutline(newNoisyOutlineVector);//add NoisyOutline to the object, overwriting old Noisyoutline
    }
    // get obstacles intersecting with sensor cone
    auto obstacleIntersections = mGlobalEnvironmentModel->preselectObstacles(detection.sensorCone);

    if (mFovConfig.doLineOfSightCheck)
    {
        std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;

        // check if objects in sensor cone are hidden by another object or an obstacle
        for (const auto& object : preselObjectsInSensorRange)
        {
            bool isInLoS = false;
            for (const auto& objectPoint : object->getNoisyOutline())//TODO: here
            {
                // skip objects points outside of sensor cone
                if (!bg::covered_by(objectPoint, detection.sensorCone)) {
                    continue;
                }

                LineOfSight lineOfSight;
                lineOfSight[0] = detection.sensorOrigin;
                lineOfSight[1] = objectPoint;

                bool noVehicleOccultation = std::none_of(preselObjectsInSensorRange.begin(), preselObjectsInSensorRange.end(),
                        [&](const std::shared_ptr<EnvironmentModelObject>& object) {
                            return bg::crosses(lineOfSight, object->getNoisyOutline()); //TODO: here
                        });

                bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
                        [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
                            ASSERT(obstacle);
                            if (bg::intersects(lineOfSight, obstacle->getOutline())) {
                                blockingObstacles.insert(obstacle);
                                return true;
                            } else {
                                return false;
                            }
                        });

                if (noVehicleOccultation && noObstacleOccultation) {
                    if (detection.objects.empty() || detection.objects.back() != object) {
                        detection.objects.push_back(object);
                        isInLoS = true;
                    }

                    if (mDrawLinesOfSight) {
                        detection.visiblePoints.push_back(objectPoint);//TODO: 
                    } else {
                        // no need to check other object points in detail except for visualization
                        break;
                    }
                }
            } // for each (corner) point of object polygon
            if (!isInLoS) {
                object->removeNoisyOutline();//TODO: this instead of sensor function hasNoise()?
            }
        } // for each object

        detection.obstacles.assign(blockingObstacles.begin(), blockingObstacles.end());
    } else {
        for (const auto& object : preselObjectsInSensorRange) {
            // preselection: object's bounding box and sensor cone's bounding box intersect
            // now: check if their actual geometries intersect somewhere
            if (bg::intersects(object->getNoisyOutline(), detection.sensorCone)) {//TODO: here
                detection.objects.push_back(object);
            }
        }
    }

    return detection;
}

SensorDetection RealisticFovSensor::createSensorCone() const
{
    SensorDetection detection;
    const auto& egoObj = mGlobalEnvironmentModel->getObject(mFovConfig.egoID);
    if (egoObj) {
        detection.sensorOrigin = egoObj->getAttachmentPoint(mFovConfig.sensorPosition);
        detection.sensorCone = createSensorArc(mFovConfig, *egoObj);
    } else {
        throw std::runtime_error("no object found for ID " + mFovConfig.egoID);
    }
    return detection;
}

void RealisticFovSensor::initializeVisualization()
{
    assert(mGroupFigure);
    mDrawLinesOfSight = par("drawLinesOfSight");
    bool drawSensorCone = par("drawSensorCone");
    bool drawObjects = par("drawDetectedObjects");
    bool drawObstacles = par("drawBlockingObstacles");

    if (drawSensorCone && !mSensorConeFigure) {
        mSensorConeFigure = new cPolygonFigure("sensor cone");
        mSensorConeFigure->setLineColor(mColor);
        mGroupFigure->addFigure(mSensorConeFigure);
    } else if (!drawSensorCone && mSensorConeFigure) {
        delete mSensorConeFigure->removeFromParent();
        mSensorConeFigure = nullptr;
    }

    if(mDrawLinesOfSight && !mLinesOfSightFigure) {
        mLinesOfSightFigure = new cGroupFigure("lines of sight");
        mGroupFigure->addFigure(mLinesOfSightFigure);
    } else if (!mDrawLinesOfSight && mLinesOfSightFigure) {
        delete mLinesOfSightFigure->removeFromParent();
        mLinesOfSightFigure = nullptr;
    }

    if (drawObstacles && !mObstaclesFigure) {
        mObstaclesFigure = new cGroupFigure("obstacles");
        mGroupFigure->addFigure(mObstaclesFigure);
    } else if (!drawObstacles && mObstaclesFigure) {
        delete mObstaclesFigure->removeFromParent();
        mObstaclesFigure = nullptr;
    }

    if (drawObjects && !mObjectsFigure) {
        mObjectsFigure = new cGroupFigure("objects");
        mGroupFigure->addFigure(mObjectsFigure);
    } else if (!drawObjects && mObjectsFigure) {
        delete mObjectsFigure->removeFromParent();
        mObjectsFigure = nullptr;
    }
}

const FieldOfView& RealisticFovSensor::getFieldOfView() const
{
    return mFovConfig.fieldOfView;
}

omnetpp::SimTime RealisticFovSensor::getValidityPeriod() const
{
    using namespace omnetpp;
    return SimTime { 200, SIMTIME_MS };
}

SensorPosition RealisticFovSensor::position() const
{
    return mFovConfig.sensorPosition;
}

const std::string& RealisticFovSensor::getSensorCategory() const
{
    static const std::string category = "RealisticFoV";
    return category;
}

const std::string RealisticFovSensor::getSensorName() const
{
    return mFovConfig.sensorName;
}

void RealisticFovSensor::setSensorName(const std::string& name)
{
    mFovConfig.sensorName = name;
}

void RealisticFovSensor::refreshDisplay() const
{
    if (!mLastDetection) {
        return;
    }

    if (mSensorConeFigure) {
        std::vector<cFigure::Point> points;
        for (const auto& position : mLastDetection->sensorCone) {
            points.push_back(cFigure::Point { position.x.value(), position.y.value() });
        }
        mSensorConeFigure->setPoints(points);
    }

    if (mLinesOfSightFigure) {
        // remove previous lines
        while (mLinesOfSightFigure->getNumFigures() > 0) {
            delete mLinesOfSightFigure->removeFigure(0);
        }

        const Position& startPoint = mLastDetection->sensorCone.front();

        for (const Position& endPoint : mLastDetection->visiblePoints) {
            auto line = new cLineFigure();
            line->setLineColor(mColor);
            line->setLineStyle(cFigure::LINE_DASHED);
            line->setStart(cFigure::Point { startPoint.x.value(), startPoint.y.value() });
            line->setEnd(cFigure::Point { endPoint.x.value(), endPoint.y.value() });
            mLinesOfSightFigure->addFigure(line);
        }
    }

    if (mObstaclesFigure) {
        // remove previous obstacles
        while (mObstaclesFigure->getNumFigures() > 0) {
            delete mObstaclesFigure->removeFigure(0);
        }

        for (const auto& obstacle : mLastDetection->obstacles) {
            auto polygon = new cPolygonFigure(obstacle->getObstacleId().c_str());
            polygon->setFilled(true);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::BLUE);
            for (const auto& position : obstacle->getOutline()) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObstaclesFigure->addFigure(polygon);
        }
    }

    if (mObjectsFigure) {
        // remove previous objects
        while (mObjectsFigure->getNumFigures() > 0) {
            delete mObjectsFigure->removeFigure(0);
        }

        for (const auto& object : mLastDetection->objects) {
            auto polygon = new cPolygonFigure(object->getExternalId().c_str());
            polygon->setFilled(true);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::RED);
            for (const auto& position : object->getNoisyOutline()) {//TODO: here
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObjectsFigure->addFigure(polygon);
        }
    }
}

} // namespace artery
