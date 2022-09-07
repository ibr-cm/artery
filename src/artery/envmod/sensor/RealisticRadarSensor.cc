/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RealisticRadarSensor.h"
#include "artery/envmod/sensor/SensorDetection.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
 #include <boost/geometry/algorithms/intersects.hpp> 
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graph_utility.hpp>

using LineOfSight = std::array<artery::Position, 2>;
BOOST_GEOMETRY_REGISTER_LINESTRING(LineOfSight)


namespace artery
{

Define_Module(RealisticRadarSensor)

const std::string& RealisticRadarSensor::getSensorCategory() const
{
    static const std::string category = "RealisticRadar";
    return category;
}

std::vector<Position> RealisticRadarSensor::applyMeasurementInaccuracy(SensorDetection &detection, std::vector<Position> outline) 
{
    namespace bg = boost::geometry;

    std::vector<Position> noisyOutline;

    for (auto objectPoint : outline)
    {
        // skip objects points outside of sensor cone
        if (!bg::covered_by(objectPoint, detection.sensorCone)) {
            continue;
        }
        Position sensorOri = detection.sensorOrigin;
        double xDistance = objectPoint.x.value() - sensorOri.x.value();
        double yDistance = objectPoint.y.value() - sensorOri.y.value();

        // skip object points which are too close to sensor
        double distance = sqrt(pow(xDistance, 2)+pow(yDistance, 2));
        if (distance < mFovConfig.fieldOfView.minimalDistance.value()) {
            continue;
        }

        //get random noise for sensor. 95% of all values are within the accuracy of the sensor attribute
        double angleError = normal(0, mFovConfig.fieldOfView.angleAccuracy.value() / 1.960);
        double rangeError = normal(0, mFovConfig.fieldOfView.rangeAccuracy.value() / 1.960);
    
        // precompute rotation values
        double cosAngleError = cos(angleError*PI/180.0);
        double sinAngleError = sin(angleError*PI/180.0);

        // double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
        double xNewAngle = xDistance * cosAngleError - yDistance  * sinAngleError;//rotation matrix for angle
        double yNewAngle = xDistance * sinAngleError + yDistance * cosAngleError;

        double relativeAngle = atan2(yNewAngle,xNewAngle);
        double xNewRange = rangeError * cos(relativeAngle);//transform matrix for range
        double yNewRange = rangeError * sin(relativeAngle);

        noisyOutline.push_back(Position(sensorOri.x.value() + xNewAngle + xNewRange, 
                                        sensorOri.y.value() + yNewAngle + yNewRange));
    } // for each (corner) point of object polygon

    return noisyOutline;
}

std::vector<double> RealisticRadarSensor::applyVelocityInaccuracy(std::vector<Position> outline, vanetza::units::Velocity velocity)
{
    std::vector<double> noisyVelocities;  
    for (auto objectPoint : outline)
    {
        double velocityError = normal(0, mFovConfig.fieldOfView.velocityAccuracy.value() / 1.960);
        noisyVelocities.push_back(velocity.value() + velocityError);
    }
    return noisyVelocities;
}

std::vector<Position> RealisticRadarSensor::filterLineOfSight(std::vector<std::shared_ptr<EnvironmentModelObstacle>> obstacleIntersections, 
                                std::vector<std::shared_ptr<EnvironmentModelObject>> preselObjectsInSensorRange, 
                                SensorDetection &detection, 
                                std::vector<Position> outline)
{
    namespace bg = boost::geometry;

    std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;
    std::vector<Position> visibleNoisyObjectPoints;

    for (auto objectPoint : outline)
    {
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

        if (noVehicleOccultation && noObstacleOccultation)
        {
            visibleNoisyObjectPoints.push_back(objectPoint);
            if (mDrawLinesOfSight) {
                detection.visiblePoints.push_back(objectPoint);
            }
        }

    }

    return visibleNoisyObjectPoints;
}

std::vector<Position> RealisticRadarSensor::applyResolution(SensorDetection &detection, std::vector<Position> outline)
{
    ;
}


} // namespace artery
