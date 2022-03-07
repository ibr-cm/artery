/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ENVMOD_RADARSENSOR_H_
#define ENVMOD_RADARSENSOR_H_

#include "artery/envmod/sensor/RealisticFovSensor.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graph_utility.hpp>

namespace artery
{

class RealisticLidarSensor : public RealisticFovSensor
{
public:
    const std::string& getSensorCategory() const override;

protected:
    SensorDetection detectObjects() override;
};

} // namespace artery

#endif /* ENVMOD_RADARSENSOR_H_ */
