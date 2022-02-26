/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RealisticLidarSensor.h"

using namespace omnetpp;

using LineOfSight = std::array<artery::Position, 2>;
struct Node {std::shared_ptr<artery::EnvironmentModelObject> objectPtr;};
using AdjList = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node>;
BOOST_GEOMETRY_REGISTER_LINESTRING(LineOfSight)

namespace artery
{

Define_Module(RealisticLidarSensor);

const std::string& RealisticLidarSensor::getSensorCategory() const
{
    static const std::string category = "Lidar";
    return category;
}

SensorDetection RealisticLidarSensor::detectObjects() const
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

                // skip object points which are too close to sensor
                double xdist = objectPoint.x.value() - detection.sensorOrigin.x.value();
                double ydist = objectPoint.y.value() - detection.sensorOrigin.y.value();
                double distance = sqrt(pow(xdist, 2)+pow(ydist, 2));
                if (distance < mFovConfig.fieldOfView.minimalDistance.value()) {
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
                        removePoint = true;//delete the current objectpoint
                        break;
                    }
                }
                for (auto& objectResolutionMap : noisyMap)//check if objectPoints are near other objects
                {
                    auto& objectResolution = objectResolutionMap.first;
                    if (objectResolution->getExternalId() == object->getExternalId()) {//only check different vehicles
                        continue;
                    }

                    auto& objectResolutionNoisyOutline = objectResolutionMap.second;

                    if (bg::intersects(resolution, objectResolutionNoisyOutline)) {
                        auto object_vertex_decriptor = graphPropertyMap.find(object->getExternalId())->second;
                        auto objectResolution_vertex_descriptor = graphPropertyMap.find(objectResolution->getExternalId())->second;
                        boost::add_edge(object_vertex_decriptor, objectResolution_vertex_descriptor, graph);
                        ds.union_set(object_vertex_decriptor, objectResolution_vertex_descriptor);
                        removePoint = true;//delete the current objectpoint
                    }
                }
                if (removePoint) {//remove objectPoint from detection.visiblePoints and noisyObjectPoints
                    auto noisyObjectIterator = std::find(noisyObjectPoints.begin(), noisyObjectPoints.end(), noisyObjectPoint);
                    auto detectionIterator = std::find(detection.visiblePoints.begin(), detection.visiblePoints.end(), noisyObjectPoint);
                    if (noisyObjectIterator != noisyObjectPoints.end()) {
                        noisyObjectPoints.erase(noisyObjectIterator);
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
                const auto& noisyObjectPointsMapEntry = noisyMap.find(ptr);
                if (noisyObjectPointsMapEntry != noisyMap.end()) {
                    const auto& noisyObjectPoints = noisyObjectPointsMapEntry->second;
                    if (!noisyObjectPoints.empty()) {
                        combinedObjects.push_back(std::shared_ptr<EnvironmentModelObject>(ptr));
                        visibleObjectPoints.insert(visibleObjectPoints.end(), noisyObjectPoints.begin(), noisyObjectPoints.end());
                    }
                }
            }
            //create objectWrapper for each disjoint set 
            boost::units::quantity<boost::units::si::velocity> averageVelocity = DBL_MAX * boost::units::si::meters_per_second;
            double centreX = 0, centreY = 0, newDimension1 = 0, newDimension2 = 0;
            measureDimensions(&visibleObjectPoints, &newDimension1, &newDimension2, &centreX, &centreY);
            Position newCentre(centreX,centreY);
            boost::units::quantity<boost::units::si::length> meterDimension1 = newDimension1 * boost::units::si::meters;
            boost::units::quantity<boost::units::si::length> meterDimension2 = newDimension2 * boost::units::si::meters;
            detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(combinedObjects, visibleObjectPoints, meterDimension1, meterDimension2, newCentre, averageVelocity));
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
                boost::units::quantity<boost::units::si::velocity>  avgVelocity = DBL_MAX * boost::units::si::meters_per_second;
                std::vector<std::shared_ptr<EnvironmentModelObject>> objectWrapper;
                objectWrapper.emplace_back(std::shared_ptr<EnvironmentModelObject>(object));
                detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(objectWrapper, visibleNoisyObjectPoints, object->getWidth(), object->getLength(), object->getCentrePoint(), avgVelocity));
                detection.objects.push_back(object);
            }
        }
    }

    return detection;
}

} // namespace artery
