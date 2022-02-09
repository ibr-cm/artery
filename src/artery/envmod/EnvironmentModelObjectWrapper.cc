/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/EnvironmentModelObjectWrapper.h"

namespace artery
{

/**
 * EnvironmentModelObjectWrapper
 */
using Length = traci::VehicleType::Length;


EnvironmentModelObjectWrapper::EnvironmentModelObjectWrapper(std::vector<std::shared_ptr<EnvironmentModelObject>> objectList, std::vector<Position> noisyPos, Length dimension1, Length dimension2, Position centre)
{
    mObjects = objectList;
    mNoisyOutline = noisyPos;
    mDimension2 = dimension2;
    mDimension1 = dimension1;
    mCentrePoint = centre;
}


} // namespace artery

