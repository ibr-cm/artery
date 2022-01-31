/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod//Geometry.h"
#include "artery/traci/VehicleType.h"
#include "artery/utility/Geometry.h"
#include <boost/optional/optional.hpp>
#include <cstdint>
#include <memory>
#include <vector>

namespace artery
{

/**
 * EnvironmentModelObjectWrapper
 */
class EnvironmentModelObjectWrapper
{
public:
    using Length = traci::VehicleType::Length;


    EnvironmentModelObjectWrapper(std::vector<std::shared_ptr<EnvironmentModelObject>> objectList, std::vector<Position> noisyPos, Length totalWidth, Length totalLength, Position centre);
    /**
     * Returns all EnvironmentModelObjects describing the wrapper object
     * @return vector of all included EnvironmentModelObjects
     */
    const std::vector<std::shared_ptr<EnvironmentModelObject>> getObjects() const { return mObjects; }

    /**
     * Returns the polygon describing the object's noisy outline
     * @return noisy polygon points
     */
    const std::vector<Position> getNoisyOutline() const { return mNoisyOutline; }

    /**
     * Return the centre point coord of this vehicle object
     * @return centre point
     */
    const Position& getCentrePoint() const { return mCentrePoint; }

    Length getLength() const { return mLength; }

    Length getWidth() const { return mWidth; }

    /**
     * Return outer object radius
     *
     * Object is guaranteed to lie completely in the circle described by getCentrePoint and getRadius().
     * @return outer radius
     */
    //Length getRadius() const { return mRadius; }

private:
    traci::VehicleType::Length mLength;
    traci::VehicleType::Length mWidth;
    //traci::VehicleType::Length mRadius;
    std::vector<Position> mNoisyOutline;
    std::vector<std::shared_ptr<EnvironmentModelObject>> mObjects;
    Position mCentrePoint;
};

} // namespace artery

