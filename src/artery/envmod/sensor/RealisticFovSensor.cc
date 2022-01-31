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
#include <boost/geometry/geometries/point_xy.hpp>
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
    mFovConfig.fieldOfView.angleAccuracy = par("angleAccuracy").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.rangeAccuracy = par("rangeAccuracy").doubleValue() * boost::units::si::meters;
    mFovConfig.fieldOfView.angleResolution = par("angleResolution").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.rangeResolution = par("rangeResolution").doubleValue() * boost::units::si::meters;
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
    /*for (const auto& object : preselObjectsInSensorRange)
    {
        std::vector<Position> newNoisyOutlineVector;
        for (const auto& objectPoint : object->getOutline())
        {        
            double angleError = normal(0, mFovConfig.fieldOfView.angleAccuracy.value(), 2);//get random noise for sensor
            double rangeError = normal(0, mFovConfig.fieldOfView.rangeAccuracy.value(), 2);
            double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
            double xError = abs(rangeError) * cos(angleErrorRad);//convert polar coordinates
            double yError = abs(rangeError) * sin(angleErrorRad);
            Position newNoisyPosition(objectPoint.x.value() + xError, objectPoint.y.value() + yError);//create new objectPoints with noise added
            //std::cout << "x err: " << xError << "; y err: " << yError << "\n";
            //std::cout << "x: " << objectPoint.x.value() << "; y: " << objectPoint.y.value() << "\n";
            newNoisyOutlineVector.push_back(newNoisyPosition);//add new, noisy objectPoint to NoisyOutlineVector                
        }
        object->setNoisyOutline(newNoisyOutlineVector);//add NoisyOutline to the object, overwriting old Noisyoutline
    }*/
    // get obstacles intersecting with sensor cone
    auto obstacleIntersections = mGlobalEnvironmentModel->preselectObstacles(detection.sensorCone);

    if (mFovConfig.doLineOfSightCheck)
    {
        std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;

        // check if objects in sensor cone are hidden by another object or an obstacle
        for (const auto& object : preselObjectsInSensorRange)
        {
            std::vector<Position> visibleObjectPoints;
            for (auto objectPoint : object->getOutline())
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
                            return bg::crosses(lineOfSight, object->getOutline()); 
                        });

                bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
                        [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
                            ASSERT(obstacle);
                            return bg::intersects(lineOfSight, obstacle->getOutline());
                        });

                if (noVehicleOccultation && noObstacleOccultation) {
                    Position sensorOri = detection.sensorOrigin;
                    double xDistance = objectPoint.x.value() - sensorOri.x.value();
                    double yDistance = objectPoint.y.value() - sensorOri.y.value();
                    double angleError = normal(0, mFovConfig.fieldOfView.angleAccuracy.value(), 2);//get random noise for sensor
                    double rangeError = normal(0, mFovConfig.fieldOfView.rangeAccuracy.value(), 2);

                    double relativeAngle = atan2(yDistance,xDistance);
                    double xNewRange = rangeError * cos(relativeAngle);//transform matrix for range
                    double yNewRange = rangeError * sin(relativeAngle);
                
                    double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
                    double xNewAngle = xDistance * cos(angleErrorRad) - yDistance * sin(angleErrorRad);//rotation matrix for angle
                    double yNewAngle = xDistance * sin(angleErrorRad) + yDistance * cos(angleErrorRad);
                    Position newNoisyPosition(sensorOri.x.value() + xNewAngle + xNewRange, sensorOri.y.value() + yNewAngle + yNewRange);
                    visibleObjectPoints.push_back(newNoisyPosition);
                }
            } // for each (corner) point of object polygon
            object->setNoisyOutline(visibleObjectPoints);//add NoisyOutline to the object, overwriting old Noisyoutline
        } // for each object

        std::vector<std::pair<std::shared_ptr<EnvironmentModelObject>,std::shared_ptr<EnvironmentModelObject>>>  pairList;
        std::map<std::shared_ptr<EnvironmentModelObject>, std::vector<Position>> visibleObjectMap;
        //check if the generated noisy positions are in LOS of the sensor
        for (const auto& object : preselObjectsInSensorRange)
        {
            if (object->getNoisyOutline().empty()) {
                continue;
            }
            std::vector<Position> visibleObjectPointsResolution;
            std::vector<std::shared_ptr<EnvironmentModelObject>> combinedObjects;
            combinedObjects.emplace_back(std::shared_ptr<EnvironmentModelObject>(object));
            for (auto noisyObjectPoint : object->getNoisyOutline())
            {
                LineOfSight lineOfNoisySight;
                lineOfNoisySight[0] = detection.sensorOrigin;
                lineOfNoisySight[1] = noisyObjectPoint;

                bool noNoisyVehicleOccultation = std::none_of(preselObjectsInSensorRange.begin(), preselObjectsInSensorRange.end(),
                    [&](const std::shared_ptr<EnvironmentModelObject>& object) {
                        return bg::crosses(lineOfNoisySight, object->getNoisyOutline()); 
                    });

                bool noNewObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
                    [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
                        ASSERT(obstacle);
                        if (bg::intersects(lineOfNoisySight, obstacle->getOutline())) {
                            blockingObstacles.insert(obstacle);
                            return true;
                        } else {
                            return false;
                        }
                    });
                
                if (noNoisyVehicleOccultation && noNewObstacleOccultation) {//if noisy Position is in LOS, check for resolution //TODO sensor cone
                    if (detection.objects.empty() || detection.objects.back() != object) {
                        detection.objects.push_back(object);
                    }
                    if (mDrawLinesOfSight) {
                        detection.visiblePoints.push_back(noisyObjectPoint);
                    } /*else {
                        // no need to check other object points in detail except for visualization
                        break;
                    }*/
                    Position sensorOri = detection.sensorOrigin;
                    double xDistance = noisyObjectPoint.x.value() - sensorOri.x.value();
                    double yDistance = noisyObjectPoint.y.value() - sensorOri.y.value();

                    double relativeAngle = atan2(yDistance,xDistance);
                    double xNewRange = mFovConfig.fieldOfView.rangeResolution.value() * cos(relativeAngle);//transform matrix for range
                    double yNewRange = mFovConfig.fieldOfView.rangeResolution.value() * sin(relativeAngle);

                    double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;//convert degree to rad for sin/cos
                    double xNewAngle = xDistance * cos(angleResolutionRad) - yDistance * sin(angleResolutionRad);//rotation matrix for angle
                    double yNewAngle = xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);
                    
                    Position firstPos(sensorOri.x.value() + xNewRange + xNewAngle, sensorOri.y.value() + yNewRange + yNewAngle);
                    Position secondPos(sensorOri.x.value() + xNewRange - xNewAngle, sensorOri.y.value() + yNewRange - yNewAngle);
                    Position thirdPos(sensorOri.x.value() - xNewRange + xNewAngle, sensorOri.y.value() - yNewRange + yNewAngle);
                    Position fourthPos(sensorOri.x.value() - xNewRange - xNewAngle, sensorOri.y.value() - yNewRange - yNewAngle);
                    std::vector<Position> resolution = {firstPos, secondPos, thirdPos, fourthPos};//create box for resolution

                    for (const auto& it : preselObjectsInSensorRange) {
                        if (bg::intersects(resolution,  it->getNoisyOutline())) {
                            if (it->getExternalId() != object->getExternalId()) {
                                combinedObjects.emplace_back(std::shared_ptr<EnvironmentModelObject>(it));
                                pairList.emplace_back(std::shared_ptr<EnvironmentModelObject>(object),std::shared_ptr<EnvironmentModelObject>(it));
                            }
                            for (auto point : it->getNoisyOutline()) {
                                if (!(bg::within(point, resolution))) { //&& noisyObjectPoint != point) {
                                    visibleObjectPointsResolution.push_back(point);
                                    //std::cout << "size: " << object->getNoisyOutline().size() <<" ";
                                    //object->removeNoisyOutlinePoint(noisyObjectPoint);
                                    //std::cout << "remaining: " << object->getNoisyOutline().size() <<"\n";
                                } 
                            }
                        }
                    }
                    std::cout <<"\n";

                } else { //if noisy Position is not in LOS, remove it from the NoisyOutline vector
                    object->removeNoisyOutlinePoint(noisyObjectPoint);
                }
            }
            auto visibleObjectPoints = object->getNoisyOutline();
            
            double centreX = 0, centreY = 0, newPar1 = 0, newPar2 = 0;
            measureDimensions(&visibleObjectPoints, &newPar1, &newPar2, &centreX, &centreY);
            Position newCentre(centreX,centreY);
            boost::units::quantity<boost::units::si::length> meterPar1 = newPar1 * boost::units::si::meters;
            boost::units::quantity<boost::units::si::length> meterPar2 = newPar2 * boost::units::si::meters;
            detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(combinedObjects, visibleObjectPoints, meterPar1, meterPar2, newCentre));
        
            
        }
        detection.obstacles.assign(blockingObstacles.begin(), blockingObstacles.end());
    } else {
        for (const auto& object : preselObjectsInSensorRange) {
            // preselection: object's bounding box and sensor cone's bounding box intersect
            // now: check if their actual geometries intersect somewhere
            if (bg::intersects(object->getNoisyOutline(), detection.sensorCone)) {//TODO: ask here
                detection.objects.push_back(object);
            }
        }
    }

    return detection;
}
//par1 was width, par2 was length
void RealisticFovSensor::measureDimensions(std::vector<Position> *visibleObjectPoints, double *par1, double *par2, double *centreX, double *centreY) const
{
    switch (visibleObjectPoints->size()) {
        case 1://no information about vehicle with just one objectPoint
        {
            *par1 = 0;
            *par2 = 0;
            *centreX = 0;
            *centreY = 0;
            break;
        }
        case 2: //determine whether two objectpoints are width or length of vehicle
        {
            double xdist = visibleObjectPoints->at(0).x.value() - visibleObjectPoints->at(1).x.value();
            double ydist = visibleObjectPoints->at(0).y.value() - visibleObjectPoints->at(1).y.value();
            double distance = sqrt(pow(xdist, 2)+pow(ydist, 2));
            if (distance < 2.2) {//real length of all simulated cars is 2.5, width is 1.8
                *par1 = distance;
                *par2 = 0;
            } else {
                *par2 = distance;
                *par1 = 0;
            }
            /*boost::geometry::model::d2::point_xy<double> lineCentre;; //the centre is not between the 2 corners
            boost::geometry::centroid(*visibleObjectPoints, lineCentre);
            *centreX = centroid[0]; // visibleObjectPoints->at(1).x.value() + (0.5*xdist);
            *centreY = centroid[1];*/ //visibleObjectPoints->at(1).y.value() + (0.5*ydist);
            *centreX = 0;
            *centreY = 0;
            break;
        }
        case 3://with 3 objectpoints, one distance is the diagonal line of the car. This line is longer than the actual length or width of the car
        {
            double maxDistance = 0.0, xdiagonal, ydiagonal;
            int diagonalStartIndex;
            for (int i = 0; i < visibleObjectPoints->size(); i++) {
                int j = i + 1;
                if (j == 3) {
                    j = 0;
                }
                double xdist = visibleObjectPoints->at(i).x.value() - visibleObjectPoints->at(j).x.value();
                double ydist = visibleObjectPoints->at(i).y.value() - visibleObjectPoints->at(j).y.value();
                double distance = sqrt(pow(xdist, 2)+pow(ydist, 2));
                if (distance > maxDistance) {
                    *par2 = maxDistance;
                    maxDistance = distance;
                    xdiagonal = xdist;
                    ydiagonal = ydist;
                    diagonalStartIndex = j;
                }
                if (distance < *par1 || *par1 == 0) {
                    *par1 = distance;
                }
                if (distance != maxDistance && distance > *par2) {
                    *par2 = distance;
                }
            }
            *centreX = visibleObjectPoints->at(diagonalStartIndex).x.value()+0.5*xdiagonal;
            *centreY = visibleObjectPoints->at(diagonalStartIndex).y.value()+0.5*ydiagonal;
            break;
        }
        /*case 4://exact 4 points, centre is near the sum of all coordinates divided by 4 for x and y, assume bigger length/width as real value
        {
            double smallPar1 = 0, bigPar1 = 0;
            for (int i = 0; i < visibleObjectPoints->size(); i++) {
                int j = i + 1;
                if (j == 4) {
                    j = 0;
                }
                *centreX += visibleObjectPoints->at(i).x.value();
                *centreY += visibleObjectPoints->at(i).y.value();
                double xdist = visibleObjectPoints->at(i).x.value() - visibleObjectPoints->at(j).x.value();
                double ydist = visibleObjectPoints->at(i).y.value() - visibleObjectPoints->at(j).y.value();
                double distance = sqrt(pow(xdist, 2)+pow(ydist, 2));
                if (distance < smallPar1 || smallPar1 == 0) {
                    bigPar1 = smallPar1;
                    smallPar1 = distance;
                } 
                if (distance > smallPar1 && (bigPar1 == 0 || distance < bigPar1)){
                    bigPar1 = distance;
                }
                if (distance > *par2 || *par2 == 0) {
                    *par2 = distance;
                }
            }
            *par1 = bigPar1;
            *centreX = *centreX/4;
            *centreY = *centreY/4;
            break;
        }*/
        default://with more than 4 objecpoints two cars could be mixed together so no clear rectangle can be drawn -> estimation
        {
            std::vector<Position> hull;
            boost::geometry::model::d2::point_xy<double> hullCentre;
            boost::geometry::convex_hull(*visibleObjectPoints, hull);//alternatively use envelope function for a "bounding box"
            boost::geometry::centroid(hull, hullCentre);//TODO: centroid exception
            *centreX = hullCentre.x();
            *centreY = hullCentre.y();
            *par1 = 0;
            *par2 = 0;
            break;
            /*double xmax = 0, xmin = 0, ymax = 0, ymin = 0;//TODO: this or add width/length of the seperate cars together
            for (const auto &visPos: *visibleObjectPoints) {
                if (xmax == 0 || visPos.x.value() > xmax) {
                    xmax = visPos.x.value();
                }
                if (xmin == 0 || visPos.x.value() < xmin) {
                    xmin = visPos.x.value();
                }
                if (ymax == 0 || visPos.y.value() > ymax) {
                    ymax = visPos.y.value();
                }
                if (ymin == 0 || visPos.y.value() < ymin) {
                    ymin = visPos.y.value();
                }

            }
            double xdiff = xmax - xmin;
            double ydiff = ymax - ymin;
            if (abs(xdiff) >= abs(ydiff)) {
                *length = xdiff;
                *width = ydiff;
            } else {
                *length = ydiff;
                *width = xdiff;  
            }
            *centreX = xmin+0.5*xdiff;
            *centreY = ymin+0.5*ydiff;
            break;*/
        }
    }
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
