/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef MPC_LATERAL_CONTROLLER_H
#define MPC_LATERAL_CONTROLLER_H

#include <AcadoUnified.hpp>

#include <yaml-cpp/yaml.h>

#include "adas/common/filter/digital_filter.h"
#include "PubSub.hpp"

#include <iostream>

namespace adas 
{

namespace dds 
{

namespace mpc 
{

class UnifiedControllerMPC 
{
public:
    UnifiedControllerMPC();
    ~UnifiedControllerMPC();
    void setConfig(YAML::Node&);
    void initialize();
    std::vector<double> getControlSignal();
    void updateVehicleState(adas::VehicleState&);
    void updateVehicleGlobalPosition(adas::VehicleState&);
    void updateTrajectory(adas::Trajectory&);
    void updateDesiredGear(adas::GearStatus&);

private:
    double vX = 0;
    double vY = 0;
    double steeringAngle = 0;
    double accelerationCmd = 0;
    std::vector<double> preSteering;

    // input from config file
    YAML::Node config_;
    double wX_{50};
    double wY_{50};
    double wYaw_{300};

    double wDelta_{200};
    double wDeltarate_{5000};
    double wV_{200};
    double wVRate_{100};
    double k_{1.0};

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
        
    std::vector<std::vector<double>> controlOutput;
    std::vector<std::vector<double>> preControl;
    double desSpeed = 0;
    adas::VehicleState egoVehicleState_;
    adas::VehicleState vehicleGlobalPosition_;
    adas::Trajectory desiredTrajectory_;
    adas::GearStatus desiredGear_;
    adas::Pose2f setInitialPosition();
};

} // namespace mpc

} // namespace dds

} // namespace adas

#endif /* MPC_LATERAL_CONTROLLER_H */
