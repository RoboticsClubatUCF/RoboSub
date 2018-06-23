#include <sub_thruster_library/thruster_pwm.h>
#include <cmath>
#include <cstdio>

ThrusterPWM::ThrusterPWM(BlackLib::pwmName pwm_pin):
    pwm_interface_(pwm_pin), GenericThruster()
{
    pwm_interface_.setPeriodTime(20, BlackLib::milliseconds);
    // Initialize thruster with 0 velocity command
    setVelocity(0);
}

ThrusterPWM::~ThrusterPWM()
{
    setVelocity(0);
}

void ThrusterPWM::setVelocityRatio(double velocity_ratio, ThrusterPWMDirection direction)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < 0.0)
        velocity_ratio = 0.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

    // Negative values are interpreted by the thruster as reverse direction.
    if (direction == ThrusterPWMDirections::Reverse)
        velocity_ratio *= -1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif

    // Send command value to thruster
    setVelocity(velocity_ratio);
}

void ThrusterPWM::setVelocityRatio(double velocity_ratio)
{
    // Constrain the scale the user requests.
    if (velocity_ratio < -1.0)
        velocity_ratio = -1.0;
    else if (velocity_ratio > 1.0)
        velocity_ratio = 1.0;

#ifdef LINEARIZE_OUTPUT
    velocity_ratio = linearizeOutput(velocity_ratio);
#endif
    // Send command value to thruster

    setVelocity(velocity_ratio);
}

#ifdef LINEARIZE_OUTPUT
double ThrusterPWM::linearizeOutput(double velocity_desired)
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
#endif

void ThrusterPWM::setVelocity(double target_ratio)
{
    pwm_interface_.setDutyPercent(7.5+target_ratio*0.02);
}

std::string ThrusterPWM::getType()
{
	return "PWM Thruster";
}

bool ThrusterPWM::inLimits()
{
	return true;
}

void ThrusterPWM::updateStatus()
{
    return;
}

double ThrusterPWM::getTemperature()
{
    return 0.0;
}

double ThrusterPWM::getVoltage()
{
	return 0.0;
}

double ThrusterPWM::getCurrent()
{
    return 0.0;
}

bool ThrusterPWM::isAlive()
{
    return true;
}