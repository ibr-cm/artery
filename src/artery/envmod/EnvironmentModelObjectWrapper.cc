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
using Velocity = traci::VehicleType::Velocity;


EnvironmentModelObjectWrapper::EnvironmentModelObjectWrapper(std::vector<std::weak_ptr<EnvironmentModelObject>> objectList, std::vector<Position> noisyPos, Length dimension1, Length dimension2, Position centre, Velocity velocity)
{
    mObjects = objectList;
    mNoisyOutline = noisyPos;
    mDimension2 = dimension2;
    mDimension1 = dimension1;
    mCentrePoint = centre;
    mVelocity = velocity;
}

EnvironmentModelObjectWrapper::EnvironmentModelObjectWrapper(std::vector<std::weak_ptr<EnvironmentModelObject>> objectList, std::vector<Position> visibleObjectPoints, boost::units::quantity<boost::units::si::velocity> averageVelocity)
{
    mObjects = objectList;
    mNoisyOutline = visibleObjectPoints;
    mVelocity = averageVelocity;

    switch (mNoisyOutline.size()) {
        case 0:
        case 1://no information about vehicle with just one objectPoint
        {
            mDimension1 = 0;
            mDimension2 = 0;
            mCentrePoint = Position(0,0);
            break;
        }
        case 2: //determine whether two objectpoints are width or length of vehicle
        {
            double xdist = mNoisyOutline.at(0).x.value() - mNoisyOutline.at(1).x.value();
            double ydist = mNoisyOutline.at(0).y.value() - mNoisyOutline.at(1).y.value();
            boost::units::quantity<boost::units::si::length> distance = sqrt(pow(xdist, 2)+pow(ydist, 2))  * boost::units::si::meters;
            if (distance < 2.2 * boost::units::si::meters) {//real length of all simulated cars is 2.5, width is 1.8
                mDimension1 = distance;
                mDimension2 = 0;
            } else {
                mDimension2 = distance;
                mDimension1 = 0;
            }
            //boost::geometry::model::d2::point_xy<double> lineCentre;; //the centre is not between the 2 corners
            //boost::geometry::centroid(mNoisyOutline, lineCentre);
            // mCentrePoint = Position(centroid[0],centroid[1]);

            mCentrePoint = Position(0,0);
            break;
        }
        case 3://with 3 objectpoints, one distance is the diagonal line of the car. This line is longer than the actual length or width of the car
        {
            boost::units::quantity<boost::units::si::length>  maxDistance = 0.0 * boost::units::si::meters;
            double xdiagonal, ydiagonal;
            int diagonalStartIndex;
            for (int i = 0; i < mNoisyOutline.size(); i++) {
                int j = i + 1;
                if (j == 3) {
                    j = 0;
                }
                double xdist = mNoisyOutline.at(i).x.value() - mNoisyOutline.at(j).x.value();
                double ydist = mNoisyOutline.at(i).y.value() - mNoisyOutline.at(j).y.value();
                boost::units::quantity<boost::units::si::length> distance = sqrt(pow(xdist, 2)+pow(ydist, 2)) * boost::units::si::meters;
                if (distance > maxDistance) {
                    mDimension2 = maxDistance;
                    maxDistance = distance;
                    xdiagonal = xdist;
                    ydiagonal = ydist;
                    diagonalStartIndex = j;
                }
                if (distance < mDimension1 || mDimension1 == 0 * boost::units::si::meters) {
                    mDimension1 = distance;
                }
                if (distance != maxDistance && distance > mDimension2) {
                    mDimension2 = distance;
                }
            }
            mCentrePoint = Position(mNoisyOutline.at(diagonalStartIndex).x.value()+0.5*xdiagonal, 
                                    mNoisyOutline.at(diagonalStartIndex).y.value()+0.5*ydiagonal);
            break;
        }
        default://with 4 or more objecpoints two cars could be mixed together so no clear rectangle can be drawn -> estimation
        {
            std::vector<Position> hull;
            boost::geometry::model::d2::point_xy<double> hullCentre;
            boost::geometry::convex_hull(mNoisyOutline, hull);//alternatively use envelope function for a "bounding box"
            try
            {
                boost::geometry::centroid(hull, hullCentre);
                mNoisyOutline.clear();
                mNoisyOutline.insert(mNoisyOutline.end(), hull.begin(), hull.end());
                mCentrePoint = Position(hullCentre.x(), hullCentre.y());
            }
            catch(const boost::geometry::centroid_exception& e)
            {
                std::cerr << e.what() << " centroid error\n";
                mCentrePoint = Position(0, 0);
            }
            mDimension1 = 0;
            mDimension2 = 0;
            break;
        }
    }
}


} // namespace artery

