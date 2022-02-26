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
    EnvironmentModelObjectWrapper(std::vector<std::shared_ptr<EnvironmentModelObject>> objectList, std::vector<Position> noisyPos, Length dimension1, Length dimension2, Position centre, Velocity velocity);
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
    std::vector<std::shared_ptr<EnvironmentModelObject>> mObjects;
    Position mCentrePoint;
};

    inline bool operator<(const EnvironmentModelObjectWrapper& left, const EnvironmentModelObjectWrapper& right) {
        /*const auto& lCentre = left.getCentrePoint();
        const auto& rCentre = right.getCentrePoint();
        if (lCentre.x < rCentre.x) {
            return true;
        } else if (lCentre.x == rCentre.x && lCentre.y < rCentre.y) {
            return true;
        } else if (lCentre.x == 0 && lCentre.y == 0 && rCentre.x == 0 && rCentre.y == 0){
            return true;
        } else {
            return false;   
        }*/
        
        if (left.getObjects().size() < right.getObjects().size()) {
            return true;
        } else if (left.getObjects().size() == right.getObjects().size()) {
            int i = 0;
            for (auto& lObj : left.getObjects()) {
                if (lObj->getExternalId() < right.getObjects().at(i)->getExternalId())
                    return true;
                i++;
            }
            return false;
        } else {
            return false;
        }
    }
} // namespace artery

