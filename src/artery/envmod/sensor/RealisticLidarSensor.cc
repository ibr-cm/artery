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
        double rangeError = normal(0, mSensorRangeAccuracy3Sigma);
    
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

        if (noVehicleOccultation && noObstacleOccultation)
        {
            visibleNoisyObjectPoints.push_back(objectPoint);
        }

    }

    return visibleNoisyObjectPoints;
}

std::vector<Position> RealisticLidarSensor::computeResolutionBounds(SensorDetection &detection, std::vector<Position> outline, std::vector<double> velocities)
{
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
