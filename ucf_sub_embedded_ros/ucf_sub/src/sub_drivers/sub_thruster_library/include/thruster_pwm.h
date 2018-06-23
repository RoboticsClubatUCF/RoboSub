#ifndef T200_Thruster_H
#include "i2c_interface.h"
#include "BlackPWM/BlackPWM.h"

namespace ThrusterPWMDirections
{
    enum ThrusterPWMDirection
    {
        Forward,
        Reverse
    };
}
typedef ThrusterPWMDirections::ThrusterPWMDirection ThrusterPWMDirection;

////////////////////////////////////////////////////////////////////////////////
/// \brief Class providing functionality for using the Blue Robotics Blue ESC T200
/// 
/// This class provides for control and monitoring of Blue Robotics T200 Thrusters
/// using the Blue ESC in I2C mode.  It supports velocity control as a ratio of 0.0
/// to 1.0 in forward and reverse directions.
///
////////////////////////////////////////////////////////////////////////////////
class ThrusterPWM : public GenericThruster
{
public:
////////////////////////////////////////////////////////////////////////////////
/// \brief Constructs a new ThrusterPWM interface.
/// 
/// \param bus_number   The I2C bus to use.
/// \param address      The 7-bit slave address of the thruster.
////////////////////////////////////////////////////////////////////////////////
ThrusterPWM(BlackLib::pwmName);

////////////////////////////////////////////////////////////////////////////////
// \brief Default ThrusterPWM destructor.
//
////////////////////////////////////////////////////////////////////////////////
~ThrusterPWM();

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Sets the thruster power as a ratio of the maximum supported velocity by the 
/// thruster and according to the direction supplied.
///
/// \param velocity_scale    The ratio between 0.0 and 1.0 for thruster power.
/// \param direction         The direction of the thruster.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_scale, ThrusterPWMDirection direction);

private:

BlackLib::BlackPWM pwm_interface_;

void setVelocity(double target_ratio);
};

#endif
