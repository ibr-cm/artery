/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RealisticRadarSensor.h"
#include "artery/envmod/sensor/SensorDetection.h"

#include <vector>
#include <iterator>
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

        //get random noise for sensor. 99% of all values are within the accuracy of the sensor attribute
        double angleError = normal(0, mSensorAngleAccuracy3Sigma);
        double rangeError = normal(0, mSensorRangeAccuracy3Sigma);
    
        // precompute rotation values
        double cosAngleError = cos(angleError*PI/180.0);
        double sinAngleError = sin(angleError*PI/180.0);

        // double angleErrorRad = angleError*PI/180.0;//convert degree to rad for sin/cos
        double xNewAngle = xDistance * cosAngleError - yDistance  * sinAngleError;//rotation matrix for angle
        double yNewAngle = xDistance * sinAngleError + yDistance * cosAngleError;

        double relativeAngle = atan2(yNewAngle,xNewAngle);
        double xNewRange = rangeError * cos(relativeAngle);//transform matrix for range
        double yNewRange = rangeError * sin(relativeAngle);

        Position noisyObjectPoint = Position(sensorOri.x.value() + xNewAngle + xNewRange, sensorOri.y.value() + yNewAngle + yNewRange);
        noisyOutline.push_back(noisyObjectPoint);

    } // for each (corner) point of object polygon

    return noisyOutline;
}

std::vector<double> RealisticRadarSensor::applyVelocityInaccuracy(std::vector<Position> outline, vanetza::units::Velocity velocity)
{
    std::vector<double> noisyVelocities;  
    for (auto objectPoint : outline)
    {
        double velocityError = normal(0, mSensorVelocityAccuracy3Sigma);
        noisyVelocities.push_back(velocity.value() + velocityError);
    }
    return noisyVelocities;
}

std::vector<Position> RealisticRadarSensor::filterLineOfSight(const std::vector<std::shared_ptr<EnvironmentModelObstacle>> &obstacleIntersections,
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
                    return bg::crosses(lineOfSight, object->getOutline()); 
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

        if (noVehicleOccultation && noObstacleOccultation)
        {
            visibleNoisyObjectPoints.push_back(objectPoint);
        }

    }

    return visibleNoisyObjectPoints;
}

std::vector<Position> RealisticRadarSensor::computeResolutionBounds(SensorDetection &detection, std::vector<Position> outline, std::vector<double> velocities)
{
    std::vector<Position> resolution;
    std::vector<Position> hull;
    // std::vector<std::tuple<Position, Position>> indistinguishablePoints;

    std::vector<double>::iterator objectPointVelocity = velocities.begin();
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
        double rangeSelectivity = mFovConfig.fieldOfView.rangeResolution.value() / 2.0;
        double angularSelectivity = objectDistance * tan((mFovConfig.fieldOfView.angleResolution.value() / 2.0) * PI/180.0);
        double velocitySelectivity = mFovConfig.fieldOfView.velocityResolution.value() / 2.0;

        // selectivity box
        std::vector<Position> selectivity = {
            Position(objectDistance - rangeSelectivity, -angularSelectivity),
            Position(objectDistance - rangeSelectivity, angularSelectivity),
            Position(objectDistance + rangeSelectivity, angularSelectivity),
            Position(objectDistance + rangeSelectivity, -angularSelectivity),
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

        std::vector<double>::iterator selfObjectResolutionVelocity = velocities.begin();
        for (std::vector<Position>::iterator selfObjectResolution = outline.begin(); selfObjectResolution != outline.end();) {

            // check if point is in resolution box but not same point
            if (selfObjectResolution != objectPoint) {
                if (boost::geometry::within(*selfObjectResolution, box)) {
                    if (
                        (selfObjectResolutionVelocity - velocitySelectivity) < objectPointVelocity
                        || (selfObjectResolutionVelocity + velocitySelectivity) > objectPointVelocity
                    ) {
                        double avgXDist = (objectPoint->x.value() + selfObjectResolution->x.value()) / 2.0;
                        double avgYDist = (objectPoint->y.value() + selfObjectResolution->y.value()) / 2.0;

                        selfObjectResolution = outline.erase(selfObjectResolution);
                        objectPoint = outline.erase(objectPoint);

                        outline.push_back(Position(avgXDist,avgYDist));
                        mergedPoints = true;
                        break;
                    }
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
        objectPointVelocity++;
    }

    boost::geometry::convex_hull(resolution, hull);

    if (mDrawResolution) {
        //draws the frame around each objectPoint
        detection.objectPointResolutions.push_back(hull);
    }

    return hull;
}


} // namespace artery
