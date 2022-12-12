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
static const simsignal_t FovRangeErrSignal = cComponent::registerSignal("RealisticFovSensorRangErr");
static const simsignal_t FovErasedPointsSignal = cComponent::registerSignal("RealisticFovSensorErasedPoints");

namespace artery
{

Define_Module(RealisticLidarSensor);

const std::string& RealisticLidarSensor::getSensorCategory() const
{
    static const std::string category = "Lidar";
    return category;
}

// SensorDetection RealisticLidarSensor::detectObjects() 
// {

//     namespace bg = boost::geometry;
//     if (mFovConfig.fieldOfView.range <= 0.0 * boost::units::si::meter) {
//         throw std::runtime_error("sensor range is 0 meter or less");
//     } else if (mFovConfig.fieldOfView.angle > 360.0 * boost::units::degree::degrees) {
//         throw std::runtime_error("sensor opening angle exceeds 360 degree");
//     }

//     SensorDetection detection = createSensorCone();
//     auto preselObjectsInSensorRange = mGlobalEnvironmentModel->preselectObjects(mFovConfig.egoID, detection.sensorCone);
//     auto obstacleIntersections = mGlobalEnvironmentModel->preselectObstacles(detection.sensorCone);

//     // if (mFovConfig.doLineOfSightCheck)
//     // {
//         std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;
//         auto graph = AdjList();
//         std::map<std::string, boost::graph_traits<AdjList>::vertex_descriptor> graphPropertyMap;
//         std::map<std::shared_ptr<EnvironmentModelObject>, std::vector<Position>> noisyMap;
//         // check if objects in sensor cone are hidden by another object or an obstacle
//         for (const auto& object : preselObjectsInSensorRange)
//         {

//             // filter all outline points which are not in line of sight
//             std::vector<Position> visibleObjectPoints = filterLineOfSight(obstacleIntersections, 
//                                                             preselObjectsInSensorRange, 
//                                                             detection, 
//                                                             object->getOutline());

//             std::vector<Position> visibleNoisyObjectPoints = applyMeasurementInaccuracy(detection, visibleObjectPoints);

//             } // for each (corner) point of object polygon
//             if (!visibleNoisyObjectPoints.empty()) {
//                 if (detection.objects.empty() || detection.objects.back() != object) {
//                     detection.objects.push_back(object);
//                 }

//                 //calculate average velocity for object
//                 double averageNoisyVelocity = std::accumulate(noisyVelocities.begin(), noisyVelocities.end(), 0.0) / noisyVelocities.size();

//                 std::vector<Position> resolutionHull = computeResolutionBounds(detection, visibleNoisyObjectPoints);

//                 //create vertex for every detected object
//                 Node vertexNode;
//                 vertexNode.objectPtr = std::shared_ptr<EnvironmentModelObject>(object);
//                 auto vertex_descript = boost::add_vertex(vertexNode,graph);

//                 //store noisy outline, velocity and vertex_descriptor
//                 noisyObjects.emplace(std::shared_ptr<EnvironmentModelObject>(object), std::make_tuple(visibleNoisyObjectPoints, resolutionHull, averageNoisyVelocity, vertex_descript));
//             }
//         } // for each object real LOS

//         // int removedPoints = 0;
//         // for(auto& objectPair : noisyMap)//if two objectpoints are on the same angle lidar only detects closest point
//         // {
//         //     auto& object = objectPair.first;
//         //     auto& noisyObjectPoints = objectPair.second;
//         //     for (auto loopIterator = noisyObjectPoints.begin(); loopIterator != noisyObjectPoints.end(); ++loopIterator) {
//         //         auto& noisyObjectPoint = *loopIterator;
//         //         LineOfSight lineOfSight;
//         //         lineOfSight[0] = detection.sensorOrigin;
//         //         lineOfSight[1] = noisyObjectPoint;
//         //         bool noVehicleOccultation = std::none_of(noisyMap.begin(), noisyMap.end(),
//         //                 [&](const std::pair<std::shared_ptr<EnvironmentModelObject>, std::vector<artery::Position>>& objectNoisyMap) {
//         //                     for (auto& otherOutlinePoint : objectNoisyMap.second) {//check if another point on same beam blocks line of sight
//         //                         if (noisyObjectPoint != otherOutlinePoint && bg::distance(otherOutlinePoint, lineOfSight)<0.00001) {
//         //                             return true;
//         //                         }
//         //                     }
//         //                     return false;
//         //                 });

//         //         bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
//         //                 [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
//         //                     ASSERT(obstacle);
//         //                     if (bg::intersects(lineOfSight, obstacle->getOutline())) {
//         //                         blockingObstacles.insert(obstacle);
//         //                         return true;
//         //                     } else {
//         //                         return false;
//         //                     }
//         //                 });

//         //         if (!noObstacleOccultation || !noVehicleOccultation) {
//         //             auto noisyObjectIterator = std::find(noisyObjectPoints.begin(), noisyObjectPoints.end(), noisyObjectPoint);
//         //             auto detectionIterator = std::find(detection.visiblePoints.begin(), detection.visiblePoints.end(), noisyObjectPoint);
//         //             if (noisyObjectIterator != noisyObjectPoints.end()) {
//         //                 removedPoints++;
//         //                 noisyObjectPoints.erase(noisyObjectIterator);
//         //                 if (detectionIterator != detection.visiblePoints.end()) {
//         //                     detection.visiblePoints.erase(detectionIterator); 
//         //                 }
//         //                 --loopIterator;//without decrements, the iterator will be out of vector range in the next loop or skip an element
//         //             }
//         //         }
//         //     }
//         // }


//         // emit(FovErasedPointsSignal, removedPoints);

//         //check if distance between each point is bigger than resolution, connect objects in graph if distance is too small
//         //if other points are inside the resolution area, the point is removed
//         //graph: https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp
//         typedef boost::graph_traits<AdjList>::vertex_descriptor Vertex;
//         typedef boost::graph_traits<AdjList>::vertices_size_type VertexIndex; 
//         std::vector<VertexIndex> rank(num_vertices(graph));
//         std::vector<Vertex> parent(num_vertices(graph));
//         typedef VertexIndex* Rank;
//         typedef Vertex* Parent;
//         boost::disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);
//         initialize_incremental_components(graph, ds);
//         incremental_components(graph, ds);
        
//         for(auto& objectPair : noisyMap)
//         {
//             auto& object = objectPair.first;
//             auto& noisyObjectPoints = objectPair.second;
//             if (noisyObjectPoints.empty()) {
//                 continue;
//             }
            
//             for (auto loopIterator = noisyObjectPoints.begin(); loopIterator != noisyObjectPoints.end(); ++loopIterator) 
//             {
//                 auto& noisyObjectPoint = *loopIterator;
//                 Position sensorOri = detection.sensorOrigin;
//                 double xDistance = noisyObjectPoint.x.value() - sensorOri.x.value();
//                 double yDistance = noisyObjectPoint.y.value() - sensorOri.y.value();

//                 double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;//convert degree to rad for sin/cos
//                 double xNewAngleCounterClock = xDistance * cos(angleResolutionRad) - yDistance * sin(angleResolutionRad);//rotation matrix for angle
//                 double yNewAngleCounterClock = xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);

//                 double xNewAngleClock = xDistance * cos(angleResolutionRad) + yDistance * sin(angleResolutionRad);//rotation matrix for angle for other direction
//                 double yNewAngleClock = -xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);

//                 Position secondPos(sensorOri.x.value() + xNewAngleCounterClock, sensorOri.y.value() + yNewAngleCounterClock);
//                 Position thirdPos(noisyObjectPoint.x.value() , noisyObjectPoint.y.value());
//                 Position fourthPos(sensorOri.x.value() + xNewAngleClock, sensorOri.y.value() + yNewAngleClock);
//                 std::vector<Position> resolution = {secondPos, thirdPos, fourthPos};//for resolution
                
//                 detection.objectPointResolutions.push_back(resolution);//draws the frame around each objectPoint

//                 for (auto& objectResolutionMap : noisyMap)//check if objectPoints are near other objects
//                 {
//                     auto& objectResolution = objectResolutionMap.first;
//                     if (objectResolution->getExternalId() == object->getExternalId()) {//only check different vehicles
//                         continue;
//                     }

//                     auto& objectResolutionNoisyOutline = objectResolutionMap.second;

//                     if (bg::touches(resolution, objectResolutionNoisyOutline)) {
//                         auto object_vertex_decriptor = graphPropertyMap.find(object->getExternalId())->second;
//                         auto objectResolution_vertex_descriptor = graphPropertyMap.find(objectResolution->getExternalId())->second;
//                         boost::add_edge(object_vertex_decriptor, objectResolution_vertex_descriptor, graph);
//                         ds.union_set(object_vertex_decriptor, objectResolution_vertex_descriptor);
//                     }
//                 }
//             }
//         }
//         typedef boost::component_index< VertexIndex > Components;
//         Components components(parent.begin(), parent.end());
//         // Iterate through each disjoint set and access each set member (https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp)
//         BOOST_FOREACH (VertexIndex disjointSet, components)
//         {
//             std::vector<std::weak_ptr<EnvironmentModelObject>> combinedObjects;
//             std::vector<Position> visibleObjectPoints;
//             BOOST_FOREACH (VertexIndex disjointSetMember, components[disjointSet])
//             {
//                 auto& ptr = graph[disjointSetMember].objectPtr;
//                 const auto& noisyObjectPointsMapEntry = noisyMap.find(ptr);
//                 if (noisyObjectPointsMapEntry != noisyMap.end()) {
//                     const auto& noisyObjectPoints = noisyObjectPointsMapEntry->second;
//                     if (!noisyObjectPoints.empty()) {
//                         combinedObjects.push_back(std::weak_ptr<EnvironmentModelObject>(ptr));
//                         visibleObjectPoints.insert(visibleObjectPoints.end(), noisyObjectPoints.begin(), noisyObjectPoints.end());
//                     }
//                 }
//             }
//             //create objectWrapper for each disjoint set 
//             boost::units::quantity<boost::units::si::velocity> averageVelocity = DBL_MAX * boost::units::si::meters_per_second;
//             double centreX = 0, centreY = 0, newDimension1 = 0, newDimension2 = 0;
//             // measureDimensions(&visibleObjectPoints, &newDimension1, &newDimension2, &centreX, &centreY);
//             Position newCentre(centreX,centreY);
//             boost::units::quantity<boost::units::si::length> meterDimension1 = newDimension1 * boost::units::si::meters;
//             boost::units::quantity<boost::units::si::length> meterDimension2 = newDimension2 * boost::units::si::meters;
//             detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(combinedObjects, visibleObjectPoints, 0.0 * boost::units::si::meters_per_second));
//         }
//         detection.obstacles.assign(blockingObstacles.begin(), blockingObstacles.end());
//     // } else {
//     //     for (const auto& object : preselObjectsInSensorRange) {
//     //         // preselection: object's bounding box and sensor cone's bounding box intersect
//     //         // now: check if their actual geometries intersect somewhere
//     //         if (bg::intersects(object->getOutline(), detection.sensorCone)) {
//     //             detection.objects.push_back(object);
//     //         }
//     //     }
//     // }

//     return detection;
// }

std::vector<Position> RealisticLidarSensor::applyMeasurementInaccuracy(SensorDetection &detection, std::vector<Position> outline)
{
    namespace bg = boost::geometry;

    std::vector<Position> noisyOutline;
    Position sensorOri = detection.sensorOrigin;

    for (auto objectPoint : outline)
    {
        // skip objects points outside of sensor cone
        if (!bg::covered_by(objectPoint, detection.sensorCone)) {
            continue;
        }
        double xDistance = objectPoint.x.value() - sensorOri.x.value();
        double yDistance = objectPoint.y.value() - sensorOri.y.value();

        // skip object points which are too close to sensor
        double distance = sqrt(pow(xDistance, 2)+pow(yDistance, 2));
        if (distance < mFovConfig.fieldOfView.minimalDistance.value()) {
            continue;
        }

        //get random noise for sensor. 99% of all values are within the accuracy of the sensor attribute
        // double angleError = normal(0, mSensorAngleAccuracy3Sigma);
        double rangeError = normal(0, mSensorRangeAccuracy3Sigma);
    
        // precompute rotation values
        // double cosAngleError = cos(angleError*PI/180.0);
        // double sinAngleError = sin(angleError*PI/180.0);

        // double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
        // double xNewAngle = xDistance * cosAngleError - yDistance  * sinAngleError;//rotation matrix for angle
        // double yNewAngle = xDistance * sinAngleError + yDistance * cosAngleError;

        double relativeAngle = atan2(yDistance,xDistance);
        double xNewRange = rangeError * cos(relativeAngle);//transform matrix for range
        double yNewRange = rangeError * sin(relativeAngle);

        double xRoundedAngle = 0.0;
        double yRoundedAngle = 0.0;
        if (mFovConfig.fieldOfView.angleResolution.value() == 0) {
            xRoundedAngle = xDistance;
            yRoundedAngle = yDistance;
        } else {
            double relativeAngle = atan2(yDistance+yNewRange,xDistance+xNewRange);
            double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;
            double rounded = std::fmod(relativeAngle, angleResolutionRad);
            xRoundedAngle = (xDistance+xNewRange) * cos(rounded) + (yDistance+yNewRange) * sin(rounded);//rotation matrix for angle
            yRoundedAngle = -(xDistance+xNewRange) * sin(rounded) + (yDistance+yNewRange) * cos(rounded);
        }

        Position noisyObjectPoint = Position(sensorOri.x.value() + xRoundedAngle, sensorOri.y.value() + yRoundedAngle);
        noisyOutline.push_back(noisyObjectPoint);

    } // for each (corner) point of object polygon

    return noisyOutline;

    // Position sensorOri = detection.sensorOrigin;
    // double xDistance = objectPoint.x.value() - sensorOri.x.value();
    // double yDistance = objectPoint.y.value() - sensorOri.y.value();
    // // double rangeError = normal(0, mFovConfig.fieldOfView.rangeAccuracy.value());//get random noise for sensor

    // emit(FovRangeErrSignal, rangeError);

    // double relativeAngle = atan2(yDistance,xDistance);
    // double xNewRange = rangeError * cos(relativeAngle);//transform matrix for range
    // double yNewRange = rangeError * sin(relativeAngle);

    // double xRoundedAngle = 0.0;
    // double yRoundedAngle = 0.0;
    // if (mFovConfig.fieldOfView.angleResolution.value() == 0) {
    //     xRoundedAngle = xDistance;
    //     yRoundedAngle = yDistance;
    // } else {
    //     double relativeAngle = atan2(yDistance+yNewRange,xDistance+xNewRange);
    //     double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;
    //     double rounded = std::fmod(relativeAngle, angleResolutionRad);
    //     xRoundedAngle = (xDistance+xNewRange) * cos(rounded) + (yDistance+yNewRange) * sin(rounded);//rotation matrix for angle
    //     yRoundedAngle = -(xDistance+xNewRange) * sin(rounded) + (yDistance+yNewRange) * cos(rounded);
    // }
    // Position newNoisyPosition(sensorOri.x.value() + xRoundedAngle, sensorOri.y.value() + yRoundedAngle);
    // visibleNoisyObjectPoints.push_back(newNoisyPosition);

    // if (detection.objects.empty() || detection.objects.back() != object) {
    //     detection.objects.push_back(object);
    // }
    // if (mDrawLinesOfSight) {
    //     detection.visiblePoints.push_back(newNoisyPosition);

}

std::vector<double> RealisticLidarSensor::applyVelocityInaccuracy(std::vector<Position> outline, vanetza::units::Velocity velocity)
{
    std::vector<double> noisyVelocities;  
    for (auto objectPoint : outline)
    {
        noisyVelocities.push_back(DBL_MAX);
    }
    return noisyVelocities;
}


std::vector<Position> RealisticLidarSensor::filterLineOfSight(const std::vector<std::shared_ptr<EnvironmentModelObstacle>> &obstacleIntersections, 
                                                        const std::vector<std::shared_ptr<EnvironmentModelObject>> &preselObjectsInSensorRange, 
                                                        const SensorDetection &detection, 
                                                        const std::vector<Position> &outline)
{
   namespace bg = boost::geometry;

    // std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;
    std::vector<Position> visibleNoisyObjectPoints;

    for (auto &objectPoint : outline)
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
                    if (bg::crosses(lineOfSight, object->getOutline())) {
                        // object itself blocks LoS to point
                        return true;
                    }
                    for (auto& otherOutlinePoint : object->getOutline()) {//check if another point on same beam blocks line of sight
                        if (objectPoint != otherOutlinePoint && bg::distance(otherOutlinePoint, lineOfSight)<0.00001) {
                            return true;
                        }
                    }
                    return false;
                });

        bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
                [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
                    ASSERT(obstacle);
                    if (bg::intersects(lineOfSight, obstacle->getOutline())) {
                        // blockingObstacles.insert(obstacle);
                        return true;
                    } else {
                        return false;
                    }
                });
                
        //   bool noVehicleOccultation = std::none_of(noisyMap.begin(), noisyMap.end(),
        //             [&](const std::pair<std::shared_ptr<EnvironmentModelObject>, std::vector<artery::Position>>& objectNoisyMap) {
        //                 for (auto& otherOutlinePoint : objectNoisyMap.second) {//check if another point on same beam blocks line of sight
        //                     if (noisyObjectPoint != otherOutlinePoint && bg::distance(otherOutlinePoint, lineOfSight)<0.00001) {
        //                         return true;
        //                     }
        //                 }
        //                 return false;
        //             });

    //         bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
    //                 [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
    //                     ASSERT(obstacle);
    //                     if (bg::intersects(lineOfSight, obstacle->getOutline())) {
    //                         blockingObstacles.insert(obstacle);
    //                         return true;
    //                     } else {
    //                         return false;
    //                     }
    //                 });

        if (noVehicleOccultation && noObstacleOccultation)
        {
            visibleNoisyObjectPoints.push_back(objectPoint);
        }

    }

    // for(auto& objectPair : noisyMap)//if two objectpoints are on the same angle lidar only detects closest point
    // {
    //     auto& object = objectPair.first;
    //     auto& noisyObjectPoints = objectPair.second;
    //     for (auto loopIterator = noisyObjectPoints.begin(); loopIterator != noisyObjectPoints.end(); ++loopIterator) {
    //         auto& noisyObjectPoint = *loopIterator;
    //         LineOfSight lineOfSight;
    //         lineOfSight[0] = detection.sensorOrigin;
    //         lineOfSight[1] = noisyObjectPoint;
    //         bool noVehicleOccultation = std::none_of(noisyMap.begin(), noisyMap.end(),
    //                 [&](const std::pair<std::shared_ptr<EnvironmentModelObject>, std::vector<artery::Position>>& objectNoisyMap) {
    //                     for (auto& otherOutlinePoint : objectNoisyMap.second) {//check if another point on same beam blocks line of sight
    //                         if (noisyObjectPoint != otherOutlinePoint && bg::distance(otherOutlinePoint, lineOfSight)<0.00001) {
    //                             return true;
    //                         }
    //                     }
    //                     return false;
    //                 });

    //         bool noObstacleOccultation = std::none_of(obstacleIntersections.begin(), obstacleIntersections.end(),
    //                 [&](const std::shared_ptr<EnvironmentModelObstacle>& obstacle) {
    //                     ASSERT(obstacle);
    //                     if (bg::intersects(lineOfSight, obstacle->getOutline())) {
    //                         blockingObstacles.insert(obstacle);
    //                         return true;
    //                     } else {
    //                         return false;
    //                     }
    //                 });

    //         if (!noObstacleOccultation || !noVehicleOccultation) {
    //             auto noisyObjectIterator = std::find(noisyObjectPoints.begin(), noisyObjectPoints.end(), noisyObjectPoint);
    //             auto detectionIterator = std::find(detection.visiblePoints.begin(), detection.visiblePoints.end(), noisyObjectPoint);
    //             if (noisyObjectIterator != noisyObjectPoints.end()) {
    //                 removedPoints++;
    //                 noisyObjectPoints.erase(noisyObjectIterator);
    //                 if (detectionIterator != detection.visiblePoints.end()) {
    //                     detection.visiblePoints.erase(detectionIterator); 
    //                 }
    //                 --loopIterator;//without decrements, the iterator will be out of vector range in the next loop or skip an element
    //             }
            
    //     }
    // }


    return visibleNoisyObjectPoints;
}

std::vector<Position> RealisticLidarSensor::computeResolutionBounds(SensorDetection &detection, std::vector<Position> outline, std::vector<double> velocities)
{


        
        // for (std::vector<Position>::iterator objectPoint = outline.begin(); objectPoint != outline.end();)
        // {
        //     // determine x and y distance to object
        //     double xDistance = objectPoint->x.value() - detection.sensorOrigin.x.value();
        //     double yDistance = objectPoint->y.value() - detection.sensorOrigin.y.value();

        //     double angleResolutionRad = mFovConfig.fieldOfView.angleResolution.value()*PI/180.0;//convert degree to rad for sin/cos
        //     double xNewAngleCounterClock = xDistance * cos(angleResolutionRad) - yDistance * sin(angleResolutionRad);//rotation matrix for angle
        //     double yNewAngleCounterClock = xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);

        //     double xNewAngleClock = xDistance * cos(angleResolutionRad) + yDistance * sin(angleResolutionRad);//rotation matrix for angle for other direction
        //     double yNewAngleClock = -xDistance * sin(angleResolutionRad) + yDistance * cos(angleResolutionRad);

        //     Position secondPos(sensorOri.x.value() + xNewAngleCounterClock, sensorOri.y.value() + yNewAngleCounterClock);
        //     Position thirdPos(objectPoint->x.value() , objectPoint->y.value());
        //     Position fourthPos(sensorOri.x.value() + xNewAngleClock, sensorOri.y.value() + yNewAngleClock);
        //     std::vector<Position> resolution = {secondPos, thirdPos, fourthPos};//for resolution
            
        //     detection.objectPointResolutions.push_back(resolution);//draws the frame around each objectPoint

        //     for (auto& objectResolutionMap : noisyMap)//check if objectPoints are near other objects
        //     {
        //         auto& objectResolution = objectResolutionMap.first;
        //         if (objectResolution->getExternalId() == object->getExternalId()) {//only check different vehicles
        //             continue;
        //         }

        //         auto& objectResolutionNoisyOutline = objectResolutionMap.second;

        //         if (bg::touches(resolution, objectResolutionNoisyOutline)) {
        //             auto object_vertex_decriptor = graphPropertyMap.find(object->getExternalId())->second;
        //             auto objectResolution_vertex_descriptor = graphPropertyMap.find(objectResolution->getExternalId())->second;
        //             boost::add_edge(object_vertex_decriptor, objectResolution_vertex_descriptor, graph);
        //             ds.union_set(object_vertex_decriptor, objectResolution_vertex_descriptor);
        //         }
        //     }
        // }

    std::vector<Position> resolution;
    std::vector<Position> hull;
    // std::vector<std::tuple<Position, Position>> indistinguishablePoints;

    for (std::vector<Position>::iterator objectPoint = outline.begin(); objectPoint != outline.end();)
    {
        // determine x and y distance to object
        double xDistance = objectPoint->x.value() - detection.sensorOrigin.x.value();
        double yDistance = objectPoint->y.value() - detection.sensorOrigin.y.value();

        // calculate polar angle to object
        double relativeAngle = atan2(yDistance,xDistance);

        // precompute rotation values
        double cosRelAngle = cos(relativeAngle);
        double sinRelAngle = sin(relativeAngle);

        double objectDistance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));

        // calculate selectivity based on range to object
        // double rangeSelectivity = mFovConfig.fieldOfView.rangeResolution.value() / 2.0;
        double angularSelectivity = objectDistance * tan((mFovConfig.fieldOfView.angleResolution.value() / 2.0) * PI/180.0);

        // selectivity box
        std::vector<Position> selectivity = {
            Position(objectDistance, -angularSelectivity),
            Position(objectDistance, angularSelectivity),
            Position(objectDistance, -angularSelectivity),
        };

        // rotate and translate box
        std::vector<Position> box;
        for (Position p: selectivity) {
            double x_ = p.x.value() * cosRelAngle - p.y.value() * sinRelAngle;
            double y_ = p.x.value() * sinRelAngle + p.y.value() * cosRelAngle;
            box.push_back(Position(detection.sensorOrigin.x.value() + x_, detection.sensorOrigin.y.value() + y_));
        }

        // check if other objectPoints of same object are too close to each other
        bool selfCheck = false;
        bool mergedPoints = false;
        for (std::vector<Position>::iterator selfObjectResolution = outline.begin(); selfObjectResolution != outline.end();) {

            // check if point is in resolution box but not same point
            if (selfObjectResolution != objectPoint) {
                if (boost::geometry::within(*selfObjectResolution, box)) {
                    double avgXDist = (objectPoint->x.value() + selfObjectResolution->x.value()) / 2.0;
                    double avgYDist = (objectPoint->y.value() + selfObjectResolution->y.value()) / 2.0;

                    selfObjectResolution = outline.erase(selfObjectResolution);
                    objectPoint = outline.erase(objectPoint);

                    outline.push_back(Position(avgXDist,avgYDist));
                    mergedPoints = true;
                    break;
                }

            }
            ++selfObjectResolution;
        }

        if (!mergedPoints) {
            resolution.insert(resolution.end(),
                std::make_move_iterator(box.begin()),
                std::make_move_iterator(box.end()));
        }
        else {
            objectPoint--;
        }
        objectPoint++;
    }

    boost::geometry::convex_hull(resolution, hull);

    if (mDrawResolution) {
        //draws the frame around each objectPoint
        detection.objectPointResolutions.push_back(hull);
    }

    return hull;
}

} // namespace artery
