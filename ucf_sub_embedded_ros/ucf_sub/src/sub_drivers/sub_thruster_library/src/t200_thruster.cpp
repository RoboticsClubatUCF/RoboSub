#include "t200_thruster.h"
#include <cmath>
#include <cstdio>

#define LINEARIZE_OUTPUT

constexpr double T200Thruster::CURRENT_SCALE_FACTOR;
constexpr double T200Thruster::VOLTAGE_SCALE_FACTOR;

T200Thruster::T200Thruster(int bus_number, unsigned char address) :
    GenericThruster(bus_number, address)
{
    for (int i = 0; i < STATUS_DATA_BYTES; i++)
    {
        status_data_.appendByte(0);
    }

    i2c_interface_.openDevice();
    // Initialize thruster with 0 velocity command
    setVelocity(0);
}

T200Thruster::~T200Thruster()
{
    i2c_interface_.closeDevice();
}

void T200Thruster::setVelocityRatio(double velocity_ratio, T200ThrusterDirection direction)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < 0.0)
        velocity_ratio = 0.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

    // Negative values are interpreted by the thruster as reverse direction.
    if (direction == T200ThrusterDirections::Reverse)
        velocity_ratio *= -1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif

    // Compute true requested velocity value between -MAX_VELOCITY_VALUE and MAX_VELOCITY_VALUE
    short velocity_value = (short)(velocity_ratio * MAX_VELOCITY_VALUE);
    // Send command value to thruster
    setVelocity(velocity_value);
}

void T200Thruster::setVelocityRatio(double velocity_ratio)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < -1.0)
        velocity_ratio = -1.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif

    // Compute true requested velocity value between -MAX_VELOCITY_VALUE and MAX_VELOCITY_VALUE
    short velocity_value = (short)(velocity_ratio * MAX_VELOCITY_VALUE);
    // Send command value to thruster

    setVelocity(velocity_value);
}

double T200Thruster::linearizeOutput(double velocity_desired)
{
    double thruster_setting = 0.0;

    if(velocity_desired > 0.01)
        thruster_setting = velocity_desired*0.788+0.101;
    else if(velocity_desired < -0.01)
        thruster_setting = velocity_desired*0.9-0.112;
    else
        thruster_setting = 0.0;

    if(thruster_setting > 1.0)
        thruster_setting = 1.0;
    else if (thruster_setting < -1.0)
        thruster_setting = -1.0;

    return thruster_setting;

}

void T200Thruster::updateStatus()
{
    status_data_ = i2c_interface_.readBulkBytes(DATA_REGISTER_START, STATUS_DATA_BYTES);
}

int T200Thruster::getPulseCount()
{
    return getRawPulseCountMeasurement();
}

double T200Thruster::getVoltage()
{
    return VOLTAGE_SCALE_FACTOR*getRawVoltageMeasurement();
}

double T200Thruster::getTemperature()
{
    // From Blue Robotics example Arduino library
    int raw_temp = getRawTemperatureMeasurement();
    double resistance = SERIES_RESISTOR/(65535/((double)raw_temp - 1));
    double steinhart;

    steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    return steinhart;
}

double T200Thruster::getCurrent()
{
    return CURRENT_SCALE_FACTOR*(getRawCurrentMeasurement() - CURRENT_SCALE_OFFSET);
}

bool T200Thruster::isAlive()
{
    return status_data_[T200ThrusterStatusIndices::Identifier] == GOOD_IDENTIFIER;
}

void T200Thruster::setVelocity(int velocity_command)
{
    ByteBuffer output_buffer;
    // Move velocity byte value into output buffer.  Note that the cast automatically
    //  removes more significant bits so we don't need to mask.
    output_buffer.appendByte((unsigned char)(velocity_command >> 8));
    output_buffer.appendByte((unsigned char)(velocity_command));

    i2c_interface_.writeRegisterByteBuffer(THROTTLE_REGISTER_START, output_buffer);
}

int T200Thruster::getRawPulseCountMeasurement()
{
    int return_value = 0;
    // Add High byte
    return_value |= status_data_[T200ThrusterStatusIndices::Pulse_Count_H];
    return_value = return_value << 8;
    // Add Low byte
    return_value |= status_data_[T200ThrusterStatusIndices::Pulse_Count_L];
    return return_value;
}

int T200Thruster::getRawVoltageMeasurement()
{
    int return_value = 0;
    // Add High byte
    return_value |= status_data_[T200ThrusterStatusIndices::Voltage_H];
    return_value = return_value << 8;
    // Add Low byte
    return_value |= status_data_[T200ThrusterStatusIndices::Voltage_L];
    return return_value;
}

int T200Thruster::getRawTemperatureMeasurement()
{
    int return_value = 0;
    // Add High byte
    return_value |= status_data_[T200ThrusterStatusIndices::Temperature_H];
    return_value = return_value << 8;
    // Add Low byte
    return_value |= status_data_[T200ThrusterStatusIndices::Temperature_L];
    return return_value;
}

int T200Thruster::getRawCurrentMeasurement()
{
    int return_value = 0;
    // Add High byte
    return_value |= status_data_[T200ThrusterStatusIndices::Current_H];
    return_value = return_value << 8;
    // Add Low byte
    return_value |= status_data_[T200ThrusterStatusIndices::Current_L];
    return return_value;
}

unsigned char T200Thruster::getIdentifier()
{
    return status_data_[T200ThrusterStatusIndices::Identifier];
}

std::string T200Thruster::getType()
{
	return "BlueRobotics T200";
}

bool T200Thruster::inLimits()
{
	return true;
}
