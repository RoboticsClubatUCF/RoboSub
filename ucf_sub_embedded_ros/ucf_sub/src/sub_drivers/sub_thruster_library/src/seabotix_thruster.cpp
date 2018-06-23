#include <sub_thruster_library/seabotix_thruster.h>
#include <cmath>
#include <cstdio>

//#define LINEARIZE_OUTPUT

SeabotixThruster::SeabotixThruster(int bus_number, unsigned char address) :
    i2c_interface_(bus_number, address), GenericThruster()
{
    for (int i = 0; i < STATUS_DATA_BYTES; i++)
    {
        status_data_.appendByte(0);
    }

    i2c_interface_.openDevice();
    // Initialize thruster with 0 velocity command
    setVelocity(0);
}

SeabotixThruster::~SeabotixThruster()
{
    i2c_interface_.closeDevice();
}

void SeabotixThruster::setVelocityRatio(double velocity_ratio, SeabotixThrusterDirection direction)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < 0.0)
        velocity_ratio = 0.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

    // Negative values are interpreted by the thruster as reverse direction.
    if (direction == SeabotixThrusterDirections::Reverse)
        velocity_ratio *= -1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif

    setVelocity(velocity_ratio);
}

void SeabotixThruster::setVelocityRatio(double velocity_ratio)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < -1.0)
        velocity_ratio = -1.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif

    setVelocity(velocity_ratio);
}

double SeabotixThruster::linearizeOutput(double velocity_desired)
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

void SeabotixThruster::updateStatus()
{
    status_data_ = i2c_interface_.readBulkBytes(DATA_REGISTER_START, STATUS_DATA_BYTES);
}

double SeabotixThruster::getTemperature()
{
    int raw_temp = getRawTemperatureMeasurement();

    return raw_temp;
}

double SeabotixThruster::getVoltage()
{
	return 0.0;
}

double SeabotixThruster::getCurrent()
{
    return 0.1*getRawCurrentMeasurement();
}

bool SeabotixThruster::isAlive()
{
    return status_data_[SeabotixThrusterStatusIndices::Fault] == 0;
}


unsigned char SeabotixThruster::getFaultStatus() //Return thruster fault data
{
    return status_data_[SeabotixThrusterStatusIndices::Fault];
}

void SeabotixThruster::setVelocity(double velocity_ratio)
{
    unsigned char velocity_command = (velocity_ratio >= 0 ? ZERO_VELOCITY_VALUE : ZERO_VELOCITY_VALUE - 1) + (char) velocity_ratio * 0x66;
    
    ByteBuffer output_buffer;
    // Move velocity byte value into output buffer.  Note that the cast automatically
    //  removes more significant bits so we don't need to mask.
    output_buffer.appendByte((unsigned char)velocity_command); //Speed
    output_buffer.appendByte((unsigned char)100); //unused "additional" info
    output_buffer.appendByte((unsigned char)(0x52 + 100 + velocity_command)); //Checksum
    
    i2c_interface_.writeRegisterByteBuffer(THROTTLE_REGISTER_START, output_buffer);
}

int SeabotixThruster::getRawTemperatureMeasurement()
{
    int return_value = 0;
    return_value |= status_data_[SeabotixThrusterStatusIndices::Temperature];
    return return_value;
}

int SeabotixThruster::getRawCurrentMeasurement()
{
    int return_value = 0;
    return_value |= status_data_[SeabotixThrusterStatusIndices::Current];
    return return_value;
}

unsigned char SeabotixThruster::getIdentifier()
{
    return status_data_[SeabotixThrusterStatusIndices::Fault];
}

std::string SeabotixThruster::getType()
{
	return "Seabotix Brushed";
}

bool SeabotixThruster::inLimits()
{
	return true;
}
