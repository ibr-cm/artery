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
    EnvironmentModelObjectWrapper(std::vector<std::weak_ptr<EnvironmentModelObject>> objectList, std::vector<Position> visibleObjectPoints, boost::units::quantity<boost::units::si::velocity> averageVelocity);
    /**
     * Returns all EnvironmentModelObjects describing the wrapper object
     * @return vector of all included EnvironmentModelObjects
     */
    const std::vector<std::weak_ptr<EnvironmentModelObject>> getObjects() const { return mObjects; }

    const std::set<uint32_t> getStationIDs() const { return mStationIDs; }
    
    /**
     * Returns the polygon describing the object's noisy outline
     * @return noisy polygon points
     */
    const std::vector<Position> &getNoisyOutline() const { return mNoisyOutline; }

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
    std::vector<Position> mResolutionHull;
    std::vector<std::weak_ptr<EnvironmentModelObject>> mObjects;
    std::set<uint32_t> mStationIDs; //sorted station ids of the objects from mObjects
    Position mCentrePoint;

};

    inline bool operator<(const EnvironmentModelObjectWrapper& left, const EnvironmentModelObjectWrapper& right) {
        if (left.getStationIDs().size() < right.getStationIDs().size()) {
            return true;
        } else if (left.getStationIDs().size() == right.getStationIDs().size()) {
            auto rightIDSet = right.getStationIDs();
            auto rightID = rightIDSet.begin();
            for (auto leftID : left.getStationIDs()) {
                if (leftID < *rightID) {
                    return true;
                }
                rightID++;
            }
            return false;
        } else {
            return false;
        }
    }
} // namespace artery

