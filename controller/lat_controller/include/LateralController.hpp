/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef MPC_LATERAL_CONTROLLER_H
#define MPC_LATERAL_CONTROLLER_H

#include <AcadoLat.hpp>
#include "PIDController.hpp"

#include <yaml-cpp/yaml.h>

#include "adas/common/filter/digital_filter.h"

#include <iostream>

namespace adas 
{

namespace dds 
{

namespace mpc 
{

class LateralControllerMPC 
{
public:
    LateralControllerMPC();
    ~LateralControllerMPC();
    void setConfig(YAML::Node&);
    void initialize();
    std::vector<double> getDesiredSteeringAngle();
    void updateVehicleState(adas::VehicleState&);
    void updateVehicleGlobalPosition(adas::VehicleState&);
    void updateTrajectory(adas::Trajectory&);

private:
    double vX = 0;
    double deltaRate = 0;
    double steeringAngle = 0;
    std::vector<double> preSteering;

    // input from config file
    YAML::Node config_;
    double wX_{0};
    double wY_{0};
    double wYaw_{300};
    double wDelta_{200};
    double wDeltarate_{5000};
    double L_f_{1.425};
    double L_r_{1.425};
    double minSteeringAngle_{-0.523};
    double maxSteeringAngle_{0.523};
    double kP{10};
    double kI{0.05};

    double pPart = 0;
    double iPart = 0;
    double addSteer = 0;
    double yawError = 0, preYawError = 0;


    Eigen::VectorXd polyfit(const adas::Trajectory& refTrajectory, int order);
    inline double constrain(double amt, double low, double high)
    {
        return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
    }

    static inline float normalizeHeadingRad(float t) 
    {
        if (t < 0) 
        {
            t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
            return 2.f * M_PI + t;
        }

        return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    }

    static inline adas::Pose2f convertMap2World(const adas::Pose2f& mapPose)
    {
        float x = mapPose.x();
        float y = -mapPose.y();
        float theta = 2 * M_PI - normalizeHeadingRad(mapPose.theta());
        return adas::Pose2f(x, y, theta);
    }
        
    std::vector<double> controlOutput;
    adas::VehicleState egoVehicleState_;
    adas::VehicleState vehicleGlobalPosition_;
    adas::Trajectory desiredTrajectory_;
    adas::Pose2f setInitialPosition();
};

class LateralControllerPP 
{
public:
    LateralControllerPP();
    ~LateralControllerPP();
    void setConfig(YAML::Node&);
    void initialize();
    std::vector<double> getDesiredSteeringAngle();
    void updateVehicleState(const adas::VehicleState&);
    void updateTrajectory(const adas::Trajectory&);

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

    YAML::Node config_;

    adas::common::DigitalFilter ffSteeringOutput_;

    PIDController pidLatErrController_;

    // Get from the configuration
    double latErrKp_{0.02};
    double latErrKi_{0.002};
    double latErrKd_{0.00};
    double latErrTs_{0.040};
    double lookAheadTime_{2.0};
    double lookAheadCompensateDistance_{3.0};
    double minLookAheadDistance_{0.5};
    double maxLookAheadDistance_{25.0};
    double wheelBase_{2.85};
    double maxSteeringWheelAngle_{+470.0};
    double minSteeringWheelAngle_{-470.0};
    double maxSteeringWheelAngleVel_{500.0};
    double maxLatErrCompensation_ = (5.0 / 16.0) * (M_PI/180);
    double minLatErrCompensation_ = (-5.0 / 16.0) * (M_PI/180);
    double kGain_{1.};
    double steeringRatio_{14.5};
};

} // namespace mpc

} // namespace dds

} // namespace adas

#endif /* MPC_LATERAL_CONTROLLER_H */
