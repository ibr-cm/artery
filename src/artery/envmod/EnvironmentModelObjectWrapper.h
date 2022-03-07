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
    using Velocity = traci::VehicleType::Velocity;

    //lengthDimension1 was width, lengthDimension2 was length of an object
    EnvironmentModelObjectWrapper(std::vector<std::weak_ptr<EnvironmentModelObject>> objectList, std::vector<Position> noisyPos, Length dimension1, Length dimension2, Position centre, Velocity velocity);
    /**
     * Returns all EnvironmentModelObjects describing the wrapper object
     * @return vector of all included EnvironmentModelObjects
     */
    const std::vector<std::weak_ptr<EnvironmentModelObject>> getObjects() const { return mObjects; }

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

    Length getDimension2() const { return mDimension2; }

    Length getDimension1() const { return mDimension1; }

    Velocity getVelocity() const { return mVelocity; }
    /**
     * Return outer object radius
     *
     * Object is guaranteed to lie completely in the circle described by getCentrePoint and getRadius().
     * @return outer radius
     */
    //Length getRadius() const { return mRadius; }

private:
    traci::VehicleType::Length mDimension2;//length
    traci::VehicleType::Length mDimension1;//width
    traci::VehicleType::Velocity mVelocity;
    std::vector<Position> mNoisyOutline;
    std::vector<std::weak_ptr<EnvironmentModelObject>> mObjects;
    Position mCentrePoint;
};

    inline bool operator<(const EnvironmentModelObjectWrapper& left, const EnvironmentModelObjectWrapper& right) {
        if (left.getObjects().size() < right.getObjects().size()) {
            return true;
        } else if (left.getObjects().size() == right.getObjects().size()) {
            int i = 0;
            for (auto& lObj : left.getObjects()) {
                if (!lObj.expired() && !right.getObjects().at(i).expired()) {
                    auto lObjShr = lObj.lock();
                    auto rObjShr = right.getObjects().at(i).lock();
                    if (lObjShr->getExternalId() < rObjShr->getExternalId())
                        return true;
                //} else if (lObj.expired() && right.getObjects().at(i).expired()) {
                } else {
                    return true;
                }
                i++;
            }
            return false;
        } else {
            return false;
        }
    }
} // namespace artery

