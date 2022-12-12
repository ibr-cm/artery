/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ENVMOD_REALISTICRADARSENSOR_H_
#define ENVMOD_REALISTICRADARSENSOR_H_

#include "artery/envmod/sensor/RealisticFovSensor.h"

namespace artery
{

class RealisticRadarSensor : public RealisticFovSensor

{
public:
    const std::string& getSensorCategory() const override;
protected:
    std::vector<Position> applyMeasurementInaccuracy(SensorDetection &detection, std::vector<Position> outline) override;
    std::vector<double> applyVelocityInaccuracy(std::vector<Position> outline, vanetza::units::Velocity velocity) override;
    
    std::vector<Position> filterLineOfSight(const std::vector<std::shared_ptr<EnvironmentModelObstacle>> &obstacleIntersections,
                                    const std::vector<std::shared_ptr<EnvironmentModelObject>> &preselObjectsInSensorRange,
                                    const SensorDetection &detection,
                                    const std::vector<Position> &outline) override;
    std::vector<Position> computeResolutionBounds(SensorDetection &detection, std::vector<Position> outline, std::vector<double> velocities) override;
};

} // namespace artery

#endif /* ENVMOD_RADARSENSOR_H_ */
