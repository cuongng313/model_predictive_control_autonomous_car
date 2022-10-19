/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "UnifiedController.hpp"

#include "math.h"

namespace adas
{

namespace dds
{

namespace mpc
{
    
UnifiedControllerMPC::UnifiedControllerMPC()
{
    std::cout << "start lat acado" << std::endl;
    controlOutput = initUnifiedAcado();
    preControl = controlOutput;
}

UnifiedControllerMPC::~UnifiedControllerMPC()
{

}

void UnifiedControllerMPC::setConfig(YAML::Node& config)
{
    config_ = config;
}

/******************************************************************************/
/// @brief Init Acado algorithm
void UnifiedControllerMPC::initialize()
{
    wX_ = config_["parking"]["mpc"]["wX"].as<double>();
    wY_ = config_["parking"]["mpc"]["wY"].as<double>();
    wYaw_ = config_["parking"]["mpc"]["wYaw"].as<double>();
    

    wDelta_ = config_["parking"]["mpc"]["wDelta"].as<double>();
    wDeltarate_ = config_["parking"]["mpc"]["wDeltarate"].as<double>();
    wV_ = config_["parking"]["mpc"]["wV"].as<double>();
    wVRate_ = config_["parking"]["mpc"]["wVRate"].as<double>();
 
    k_ = config_["parking"]["mpc"]["k"].as<double>();

    L_f_ = config_["parking"]["mpc"]["L_f_"].as<double>();
    L_r_ = config_["parking"]["mpc"]["L_r_"].as<double>();
    minSteeringAngle_ = config_["parking"]["mpc"]["minSteeringAngle"].as<double>();
    maxSteeringAngle_ = config_["parking"]["mpc"]["maxSteeringAngle"].as<double>();
    
    setParam(wX_, wY_, wYaw_, wDelta_, wDeltarate_, wV_, wVRate_, L_f_, L_r_, k_);

    std::cout << "Start MPC lateral controller..." << std::endl;
    
    controlOutput = initUnifiedAcado();
    preControl = controlOutput;
}

/******************************************************************************/
/// @brief set initial position
adas::Pose2f UnifiedControllerMPC::setInitialPosition()
{
    Pose2f initialPose;
    initialPose = adas::Pose2f(0, 0, 0);

    if (desiredTrajectory_.header_.frame_id_ == Coord::map)
    {
        
        initialPose = adas::Pose2f(vehicleGlobalPosition_.g_pos_.x(), 
                                    vehicleGlobalPosition_.g_pos_.y(),
                                    vehicleGlobalPosition_.g_pos_.wz());
        // initialPose = convertMap2World(initialPose);
    }

    // initialPose = adas::Pose2f(vehicleGlobalPosition_.g_pos_.x(), 
    //                             - vehicleGlobalPosition_.g_pos_.y(),
    //                             - vehicleGlobalPosition_.g_pos_.wz());

    return initialPose;
}

/******************************************************************************/
/// @brief get desired steering angle
std::vector<double> UnifiedControllerMPC::getControlSignal()
{
    std::vector<double> result;
    std::vector<double> predictedStates;
    std::vector<double> curState;
    std::vector<double> refStates;
    std::vector<double> refStatesTr;

    adas::Pose2f initialPos;
    
    initialPos = setInitialPosition();

    if (desiredTrajectory_.empty() )
    {
        std::cout << "Empty trajectory" << std::endl;
        steeringAngle = 0;
        accelerationCmd = 0;
        desSpeed = 0;
        controlOutput = resetControl();
        preControl = controlOutput;
        return {0, 0, 0};
    }
    else
    {
        curState.push_back(initialPos.x());
        curState.push_back(initialPos.y());
        curState.push_back(initialPos.theta());

        vX = egoVehicleState_.b_vel_.linear().x();
        vY = egoVehicleState_.b_vel_.linear().y();
        double egoVel = sqrt(vX*vX + vY*vY);
  
        std::cout << "vx: " << vX << ", vy: " << vY << ", v: " << egoVel << std::endl;

        predictedStates = motionUnifiedPrediction(curState, preControl);
        // std::cout << "Calculated motion" << std::endl;
        refStates = calculateUnifiedRefStates(desiredTrajectory_);
        // std::cout << "Calculated refStates" << std::endl;

        controlOutput = runUnifiedMpcAcado(predictedStates, refStates, preControl, desiredTrajectory_);
        
        steeringAngle = controlOutput[0][0];
        desSpeed = controlOutput[1][0];

        steeringAngle = constrain(steeringAngle, -0.52, 0.52);
        desSpeed = constrain(desSpeed, -1.9444, 1.9444);

        std::cout << "PREDES SPEED: " << desSpeed << std::endl;
        desSpeed = fabs(desSpeed);

        result.push_back(steeringAngle);
        result.push_back(accelerationCmd);
        result.push_back(desSpeed);

        preControl = controlOutput;

        return result;
    }   
}

/******************************************************************************/
/// @brief update vehicle state
void UnifiedControllerMPC::updateVehicleState(adas::VehicleState& vehicleState)
{
    egoVehicleState_ = vehicleState;
}

/******************************************************************************/
/// @brief update global position
void UnifiedControllerMPC::updateVehicleGlobalPosition(adas::VehicleState& vehicleGlobalPos)
{
    vehicleGlobalPosition_ = vehicleGlobalPos;
}

/******************************************************************************/
/// @brief update the desired trajectory
void UnifiedControllerMPC::updateTrajectory(adas::Trajectory& trajectory)
{
    desiredTrajectory_ = trajectory;
}

void UnifiedControllerMPC::updateDesiredGear(adas::GearStatus& desiredGear)
{
    desiredGear_ = desiredGear;
}

} // mpc

} // dds

} // adas
