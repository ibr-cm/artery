#ifndef ENVMOD_FIELDOFVIEW_H_RW4CQUCV
#define ENVMOD_FIELDOFVIEW_H_RW4CQUCV

#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/angle/degrees.hpp>

namespace artery
{

struct FieldOfView
{
    boost::units::quantity<boost::units::si::length> range;
    boost::units::quantity<boost::units::degree::plane_angle> angle;
    double angleError = 0.0; /*< stddev for the normal distribution of the angle/azimuth noise */
    double rangeError = 0.0; /*< stddev for the normal distribution of the range noise */
};

} // namespace artery

#endif /* ENVMOD_FIELDOFVIEW_H_RW4CQUCV */
