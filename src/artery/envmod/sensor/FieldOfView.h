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
    boost::units::quantity<boost::units::degree::plane_angle> angleAccuracy; /*< stddev for the normal distribution of the angle/azimuth noise */
    boost::units::quantity<boost::units::si::length> rangeAccuracy; /*< stddev for the normal distribution of the range noise */
    boost::units::quantity<boost::units::degree::plane_angle> angleResolution; /*< minimal angle distance between two objects with the same range */
    boost::units::quantity<boost::units::si::length> rangeResolution; /*< minimal range distance between two objects with the same angle */
};

} // namespace artery

#endif /* ENVMOD_FIELDOFVIEW_H_RW4CQUCV */
