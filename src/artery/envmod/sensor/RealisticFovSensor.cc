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
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graph_utility.hpp>
#include <omnetpp/ccomponent.h>
#include <unordered_set>

using namespace omnetpp;

using LineOfSight = std::array<artery::Position, 2>;
struct Node {std::shared_ptr<artery::EnvironmentModelObject> objectPtr;};
using AdjList = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node>;
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
    auto obstacleIntersections = mGlobalEnvironmentModel->preselectObstacles(detection.sensorCone);

    if (mFovConfig.doLineOfSightCheck)
    {
        std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;
        auto graph = AdjList();
        std::map<std::string, boost::graph_traits<AdjList>::vertex_descriptor> graphPropertyMap;
        std::map<std::shared_ptr<EnvironmentModelObject>, std::vector<Position>> noisyMap;
        // check if objects in sensor cone are hidden by another object or an obstacle
        for (const auto& object : preselObjectsInSensorRange)
        {
            std::vector<Position> visibleNoisyObjectPoints;
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
                            if (bg::intersects(lineOfSight, obstacle->getOutline())) {
                                blockingObstacles.insert(obstacle);
                                return true;
                            } else {
                                return false;
                            }
                        });
                //if objectPoint is in LOS, generate noisyObjectPoint and after all objectPoints store it into noisyMap
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
                    visibleNoisyObjectPoints.push_back(newNoisyPosition);

                    if (detection.objects.empty() || detection.objects.back() != object) {
                        detection.objects.push_back(object);
                    }
                    if (mDrawLinesOfSight) {
                        detection.visiblePoints.push_back(newNoisyPosition);
                    }
                }
            } // for each (corner) point of object polygon
            if (!visibleNoisyObjectPoints.empty()) {
                noisyMap.emplace(std::shared_ptr<EnvironmentModelObject>(object), visibleNoisyObjectPoints);//store NoisyOutline
                Node vertexNode;
                vertexNode.objectPtr = std::shared_ptr<EnvironmentModelObject>(object);
                auto vertex_descript = boost::add_vertex(vertexNode,graph);//create vertex for every detected object
                graphPropertyMap.emplace(object->getExternalId(), vertex_descript);//store vertex_descriptor and object name
            }
        } // for each object real LOS

        //check if distance between each point is bigger than resolution, connect objects in graph if distance is too small
        //if other points are inside the resolution area, the point is removed
        //graph: https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp
        typedef boost::graph_traits<AdjList>::vertex_descriptor Vertex;
        typedef boost::graph_traits<AdjList>::vertices_size_type VertexIndex; 
        std::vector<VertexIndex> rank(num_vertices(graph));
        std::vector<Vertex> parent(num_vertices(graph));
        typedef VertexIndex* Rank;
        typedef Vertex* Parent;
        boost::disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);
        initialize_incremental_components(graph, ds);
        incremental_components(graph, ds);
        
        for(auto& objectPair : noisyMap)
        {
            auto& object = objectPair.first;
            auto& noisyObjectPoints = objectPair.second;
            if (noisyObjectPoints.empty()) {
                continue;
            }
            
            for (auto loopIterator = noisyObjectPoints.begin(); loopIterator != noisyObjectPoints.end(); ++loopIterator) 
            {
                auto& noisyObjectPoint = *loopIterator;
                bool removePoint = false;
                Position sensorOri = detection.sensorOrigin;
                double xDistance = noisyObjectPoint.x.value() - sensorOri.x.value();
                double yDistance = noisyObjectPoint.y.value() - sensorOri.y.value();

                double relativeAngle = atan2(yDistance,xDistance);
                double xNewRangePos = xDistance + mFovConfig.fieldOfView.rangeResolution.value() * cos(relativeAngle);//transform matrix for range                double yNewRangePos = yDistance + mFovConfig.fieldOfView.rangeResolution.value() * sin(relativeAngle);
                double xNewRangeNeg = xDistance - mFovConfig.fieldOfView.rangeResolution.value() * cos(relativeAngle);
                double yNewRangeNeg = yDistance - mFovConfig.fieldOfView.rangeResolution.value() * sin(relativeAngle);
                double yNewRangePos = yDistance + mFovConfig.fieldOfView.rangeResolution.value() * sin(relativeAngle);

                double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;//convert degree to rad for sin/cos
                double xNewAngleCounterClock = xDistance * cos(angleResolutionRad) - yDistance * sin(angleResolutionRad);//rotation matrix for angle
                double yNewAngleCounterClock = xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);

                double xNewAngleClock = xDistance * cos(angleResolutionRad) + yDistance * sin(angleResolutionRad);//rotation matrix for angle for other direction
                double yNewAngleClock = -xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);
                        
                Position firstPos(sensorOri.x.value() + xNewRangePos , sensorOri.y.value() + yNewRangePos);
                Position secondPos(sensorOri.x.value() + xNewAngleCounterClock, sensorOri.y.value() + yNewAngleCounterClock);
                Position thirdPos(sensorOri.x.value() + xNewRangeNeg, sensorOri.y.value() + yNewRangeNeg);
                Position fourthPos(sensorOri.x.value() + xNewAngleClock, sensorOri.y.value() + yNewAngleClock);
                std::vector<Position> resolution = {firstPos, secondPos, thirdPos, fourthPos};//create box for resolution
                
                detection.objectPointResolutions.push_back(resolution);//draws the frame around each objectPoint

                bool selfCheck = false;
                for (auto& selfObjectResolution : noisyObjectPoints) {//check if objectPoints of same object are near each other
                    if (selfCheck == false && selfObjectResolution == noisyObjectPoint) {
                        selfCheck = true;//if multiple points are on the exact same positions ignore first appearance
                        continue;
                    }
                    if (bg::within(selfObjectResolution, resolution)) {
                        //removePoint = true;//delete the current objectpoint
                        break;
                    }
                }
                for (auto& objectResolutionMap : noisyMap)//check if objectPoints are near other objects
                {
                    auto& objectResolution = objectResolutionMap.first;
                    auto& objectResolutionNoisyOutline = objectResolutionMap.second;
                    if (objectResolution->getExternalId() != object->getExternalId() && bg::intersects(resolution, objectResolutionNoisyOutline)) {
                        auto object_vertex_decriptor = graphPropertyMap.find(object->getExternalId())->second;
                        auto objectResolution_vertex_descriptor = graphPropertyMap.find(objectResolution->getExternalId())->second;
                        boost::add_edge(object_vertex_decriptor, objectResolution_vertex_descriptor, graph);
                        ds.union_set(object_vertex_decriptor, objectResolution_vertex_descriptor);
                        //removePoint = true;//delete the current objectpoint
                    }
                }
                if (removePoint) {
                    auto noisyObjectIterator = std::find(noisyObjectPoints.begin(), noisyObjectPoints.end(), noisyObjectPoint); 
                    if (noisyObjectIterator != noisyObjectPoints.end()) {
                        noisyObjectPoints.erase(noisyObjectIterator);
                        //remove objectPoint from detection.visiblePoints, point not visible anymore (no LoS is drawn)
                        auto detectionIterator = std::find(detection.visiblePoints.begin(), detection.visiblePoints.end(), noisyObjectPoint);
                        if (detectionIterator != detection.visiblePoints.end()) {
                            detection.visiblePoints.erase(detectionIterator); 
                        }
                        --loopIterator;//without decrements, the iterator will be out of vector range in the next loop or skip an element
                    }
                }
            }
        }
        typedef boost::component_index< VertexIndex > Components;
        Components components(parent.begin(), parent.end());
        // Iterate through each disjoint set and access each set member (https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp)
        BOOST_FOREACH (VertexIndex disjointSet, components)
        {
            std::vector<std::shared_ptr<EnvironmentModelObject>> combinedObjects;
            std::vector<Position> visibleObjectPoints;
            BOOST_FOREACH (VertexIndex disjointSetMember, components[disjointSet])
            {
                auto& ptr = graph[disjointSetMember].objectPtr;
                const auto& noisyObjectPoints = noisyMap.find(ptr)->second;
                combinedObjects.push_back(std::shared_ptr<EnvironmentModelObject>(ptr));
                visibleObjectPoints.insert(visibleObjectPoints.end(), noisyObjectPoints.begin(), noisyObjectPoints.end());
            }
            //create objectWrapper for each disjoint set
            double centreX = 0, centreY = 0, newDimension1 = 0, newDimension2 = 0;
            measureDimensions(&visibleObjectPoints, &newDimension1, &newDimension2, &centreX, &centreY);
            Position newCentre(centreX,centreY);
            boost::units::quantity<boost::units::si::length> meterDimension1 = newDimension1 * boost::units::si::meters;
            boost::units::quantity<boost::units::si::length> meterDimension2 = newDimension2 * boost::units::si::meters;
            detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(combinedObjects, visibleObjectPoints, meterDimension1, meterDimension2, newCentre));
        }
        detection.obstacles.assign(blockingObstacles.begin(), blockingObstacles.end());
    } else {
        for (const auto& object : preselObjectsInSensorRange) {
            // preselection: object's bounding box and sensor cone's bounding box intersect
            // now: check if their actual geometries intersect somewhere
            // only creates noisy Positions, does not check resolution
            std::vector<Position> visibleNoisyObjectPoints;
            if (bg::intersects(object->getOutline(), detection.sensorCone)) {
                for (const auto& objectPoint : object->getOutline()) 
                {
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
                    visibleNoisyObjectPoints.push_back(newNoisyPosition);
                }
                std::vector<std::shared_ptr<EnvironmentModelObject>> objectWrapper;
                objectWrapper.emplace_back(std::shared_ptr<EnvironmentModelObject>(object));
                detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(objectWrapper, visibleNoisyObjectPoints, object->getWidth(), object->getLength(), object->getCentrePoint()));
                detection.objects.push_back(object);
            }
        }
    }

    return detection;
}
//dimension1 was width, dimension2 was length
void RealisticFovSensor::measureDimensions(std::vector<Position> *visibleObjectPoints, double *dimension1, double *dimension2, double *centreX, double *centreY) const
{
    switch (visibleObjectPoints->size()) {
        case 0:
        case 1://no information about vehicle with just one objectPoint
        {
            *dimension1 = 0;
            *dimension2 = 0;
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
                *dimension1 = distance;
                *dimension2 = 0;
            } else {
                *dimension2 = distance;
                *dimension1 = 0;
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
                    *dimension2 = maxDistance;
                    maxDistance = distance;
                    xdiagonal = xdist;
                    ydiagonal = ydist;
                    diagonalStartIndex = j;
                }
                if (distance < *dimension1 || *dimension1 == 0) {
                    *dimension1 = distance;
                }
                if (distance != maxDistance && distance > *dimension2) {
                    *dimension2 = distance;
                }
            }
            *centreX = visibleObjectPoints->at(diagonalStartIndex).x.value()+0.5*xdiagonal;
            *centreY = visibleObjectPoints->at(diagonalStartIndex).y.value()+0.5*ydiagonal;
            break;
        }
        default://with 4 or more objecpoints two cars could be mixed together so no clear rectangle can be drawn -> estimation
        {
            std::vector<Position> hull;
            boost::geometry::model::d2::point_xy<double> hullCentre;
            boost::geometry::convex_hull(*visibleObjectPoints, hull);//alternatively use envelope function for a "bounding box"
            boost::geometry::centroid(hull, hullCentre);//TODO: centroid exception
            visibleObjectPoints->clear();
            visibleObjectPoints->insert(visibleObjectPoints->end(), hull.begin(), hull.end());
            *centreX = hullCentre.x();
            *centreY = hullCentre.y();
            *dimension1 = 0;
            *dimension2 = 0;
            break;
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
            polygon->setFilled(false);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::RED);
            for (const auto& position : object->getOutline()) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObjectsFigure->addFigure(polygon);          
        }
        for (auto& objectWrapper : mLastDetection->objectWrapper) {
            auto polygon = new cPolygonFigure();
            polygon->setFilled(true);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::GREEN);
            for (const auto& position : objectWrapper->getNoisyOutline()) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObjectsFigure->addFigure(polygon);
        }
        for (auto& resolution : mLastDetection->objectPointResolutions) {
            auto polygon = new cPolygonFigure();
            polygon->setFilled(false);
            polygon->setLineColor(mColor);
            for (const auto& position : resolution) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObjectsFigure->addFigure(polygon);
        }
    }
}

} // namespace artery
