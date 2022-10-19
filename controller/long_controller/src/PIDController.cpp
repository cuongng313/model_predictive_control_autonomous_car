/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "PIDController.hpp"
#include <iomanip>

namespace adas
{

namespace dds
{

namespace mpc
{

PIDController::PIDController()
{

}

PIDController::~PIDController()
{

}

void PIDController::setParam(double dt, double max, double min, double Kp, double Ki, double Kd)
{
    dt_ = dt;
    max_ = max;
    min_ = min;
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

double PIDController::calculate(double error)
{
    // Proportional term
    double Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt_;
    double Iout = Ki_ * integral_;

    if (integral_ > max_/Ki_)
    {
        integral_ = max_/Ki_;
    }
    else if (integral_ < min_/Ki_)
    {
        integral_ = min_/Ki_;
    }

    // Derivative term
    double derivative = (error - preError_) / dt_;
    double Dout = Kd_ * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if(output > max_)
    {
        output = max_;
    }
    else if(output < min_)
    {
        output = min_;
    }

    // Save error to previous error
    preError_ = error;

    return output;
}

void PIDController::resetIntegral()
{
    integral_ = 0.0;
}


} // mpc

} // dds

} // adas