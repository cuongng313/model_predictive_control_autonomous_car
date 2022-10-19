/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>

namespace adas 
{

namespace dds 
{

namespace mpc 
{

class PIDController {
public:
    PIDController();
    ~PIDController();
    void setParam(double dt, double max, double min, double Kp, double Ki, double Kd);
    double calculate(double error);
    void resetIntegral();

private:
    double dt_{0.0};
    double max_{0.0};
    double min_{0.0};
    double Kp_{0.0};
    double Kd_{0.0};
    double Ki_{0.0};
    double preError_{0.0};
    double integral_{0.0};
};

} // namespace mpc

} // namespace dds

} // namespace adas

#endif /* PID_CONTROLLER_H */
