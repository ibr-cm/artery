/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RealisticRadarSensor.h"

namespace artery
{

Define_Module(RealisticRadarSensor)

const std::string& RealisticRadarSensor::getSensorCategory() const
{
    static const std::string category = "RealisticRadar";
    return category;
}

} // namespace artery
