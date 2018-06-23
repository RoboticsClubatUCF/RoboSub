#ifndef T200_Thruster_H
#include "generic_thruster.h"

namespace T200ThrusterDirections
{
    enum T200ThrusterDirection
    {
        Forward,
        Reverse
    };
}
typedef T200ThrusterDirections::T200ThrusterDirection T200ThrusterDirection;

namespace T200ThrusterStatusIndices
{
    enum T200ThrusterStatusIndex
    {
        Pulse_Count_H,
        Pulse_Count_L,
        Voltage_H,
        Voltage_L,
        Temperature_H,
        Temperature_L,
        Current_H,
        Current_L,
        Identifier
    };
} 
typedef T200ThrusterStatusIndices::T200ThrusterStatusIndex T200ThrusterStatusIndex;

////////////////////////////////////////////////////////////////////////////////
/// \brief Class providing functionality for using the Blue Robotics Blue ESC T200
/// 
/// This class provides for control and monitoring of Blue Robotics T200 Thrusters
/// using the Blue ESC in I2C mode.  It supports velocity control as a ratio of 0.0
/// to 1.0 in forward and reverse directions.
///
////////////////////////////////////////////////////////////////////////////////
class T200Thruster : public GenericThruster
{
public:
////////////////////////////////////////////////////////////////////////////////
/// \brief Constructs a new T200Thruster interface.
/// 
/// \param bus_number   The I2C bus to use.
/// \param address      The 7-bit slave address of the thruster.
////////////////////////////////////////////////////////////////////////////////
T200Thruster(int bus_number, unsigned char address);

////////////////////////////////////////////////////////////////////////////////
// \brief Default T200Thruster destructor.
//
////////////////////////////////////////////////////////////////////////////////
~T200Thruster();

void setVelocityRatio(double velocity_ratio);
void updateStatus();
double getVoltage();
double getTemperature();
double getCurrent();
int getPulseCount();
bool isAlive();
std::string getType();
bool inLimits();

void setVelocityRatio(double velocity_ratio, T200ThrusterDirection direction);

double linearizeOutput(double velocity_desired);

private:

static const int STATUS_DATA_BYTES = 9;
static const int MAX_VELOCITY_VALUE = 32767;
static constexpr double VOLTAGE_SCALE_FACTOR = 0.0004921;
static constexpr double CURRENT_SCALE_FACTOR = 0.001122;
static const int CURRENT_SCALE_OFFSET = 32767;
static const int THERMISTOR_NOMINAL = 10000;
static const int TEMPERATURE_NOMINAL = 25;
static const int B_COEFFICIENT = 3900;
static const int SERIES_RESISTOR = 3300;
static const int MOTOR_POLE_COUNT = 14;
static const unsigned char THROTTLE_REGISTER_START = 0x00;
static const unsigned char DATA_REGISTER_START = 0x02;
static const unsigned char GOOD_IDENTIFIER = 0xAB;

void setVelocity(int velocity_command);
int getRawPulseCountMeasurement();
int getRawVoltageMeasurement();
int getRawTemperatureMeasurement();
int getRawCurrentMeasurement();
unsigned char getIdentifier();
};

#endif
