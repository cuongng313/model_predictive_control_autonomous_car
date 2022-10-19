/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef MPC_LONGITUDINAL_CONTROLLER_H
#define MPC_LONGITUDINAL_CONTROLLER_H

#include <common/node_handle_pro.h>

#include "PIDController.hpp"
#include "LowPass.hpp"
#include "Acado.hpp"

#include "VehicleStateSubscriber.hpp"
#include "TrajectorySubscriber.hpp"
#include "TwistPublisher.hpp"

#include "VehicleConfigReader.hpp"

#include "adas/common/filter/digital_filter.h"

#include <math.h>

namespace adas 
{

namespace dds 
{

namespace mpc 
{

class LongitudinalControllerPID 
{
public:
    LongitudinalControllerPID();
    ~LongitudinalControllerPID();
    double getDesiredAccelDecel();
    double getDesiredVel();
    void updateVehicleState(const adas::VehicleState&);
    void updateTrajectory(const adas::Trajectory&);
    void pitchAngleEstimation();
    double getPitchAngle();

private:
    inline double constrain(double amt, double low, double high)
    {
        return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
    }

    inline double kmh2Mps(double kmh)
    {
        return ((kmh * 1000.0) / 3600.0);
    }

    inline double mps2Kmh(double mps)
    {
        return ((mps * 3600.0) / 1000.0);
    }

    inline double deg2Rad(double deg)
    {
        return ((deg * M_PI) / 180.0);
    }

    inline double rad2Deg(double rad)
    {
        return ((rad * 180.0) / M_PI);
    }
        
    adas::VehicleState egoVehicleState_;
    adas::Trajectory desiredTrajectory_;

    VehicleConfigReader vehConfigReader_;

    adas::common::DigitalFilter ffAccFilter_;
    adas::common::DigitalFilter pitchAngleFilter_;

    PIDController pidController_;

    const unsigned int minWaypointInTraj_{1};
    const double g_{9.8};
    const double minPitchAngle_{-15.0};
    const double maxPitchAngle_{15.0};
    const double standStillDec_{-0.2};
    double pitchAngle_{0.0};
    double maxAcc_{3.0};
    double maxDec_{-4.0};
    double mass_{0.0};
    double kp_{0.1}; // 0.1
    double ki_{0.02}; //ki_{0.02};
    double kd_{0.0}; //kd_{0.0};
    double ts_{0.04};
    double desiredVel_{0.0};
};

class LongitudinalControllerMPC 
{
public:
    LongitudinalControllerMPC();
    ~LongitudinalControllerMPC();
    double getDesiredAccelDecel();
    double getDesiredVel();
    void updateVehicleState(const adas::VehicleState&);
    void updateTrajectory(const adas::Trajectory&);
    void pitchAngleEstimation();
    double getPitchAngle();

private:
    inline double constrain(double amt, double low, double high)
    {
        return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
    }
    inline double kmh2Mps(double kmh)
    {
        return ((kmh * 1000.0) / 3600.0);
    }

    inline double mps2Kmh(double mps)
    {
        return ((mps * 3600.0) / 1000.0);
    }

    inline double deg2Rad(double deg)
    {
        return ((deg * M_PI) / 180.0);
    }

    inline double rad2Deg(double rad)
    {
        return ((rad * 180.0) / M_PI);
    }

    std::vector<double> mpcSolver();
        
    adas::VehicleState egoVehicleState_;
    adas::Trajectory desiredTrajectory_;

    VehicleConfigReader vehConfigReader_;

    adas::common::DigitalFilter outputAccFilter_;
    adas::common::DigitalFilter pitchAngleFilter_;

    PIDController pidController_;

    std::vector<double> controlOutput_;
    std::vector<double> preControlOutput_;
    
    adas::dds::mpc::LowPass lpfAccel_;

    const unsigned int minWaypointInTraj_{1};
    const unsigned int mpcSelectPoint_{5}; // ~ 1s ahead
    const unsigned int frequency_{25};
    const double g_{9.8};
    const double minPitchAngle_{-15.0};
    const double maxPitchAngle_{15.0};
    double pitchAngle_{0.0};
    double maxAcc_{3.0};
    double maxDec_{-4.0};
    double mass_{1800.0};
    double kp_{0.5};
    double ki_{0.1};
    double kd_{0.0};
    double ts_{0.04};
    double desiredVel_{0.0};
};

} // namespace mpc

} // namespace dds

} // namespace adas

#endif /* MPC_LONGITUDINAL_CONTROLLER_H */
