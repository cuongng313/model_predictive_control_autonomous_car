/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "LateralController.hpp"

#include "math.h"

namespace adas
{

namespace dds
{

namespace mpc
{
    
LateralControllerMPC::LateralControllerMPC()
{
    std::cout << "start lat acado" << std::endl;
    // preSteering.resize(ACADO_N, 0);
    controlOutput = initLatAcado();
    preSteering = controlOutput;
}

LateralControllerMPC::~LateralControllerMPC()
{

}

void LateralControllerMPC::setConfig(YAML::Node& config)
{
    config_ = config;
}

/******************************************************************************/
/// @brief Init Acado algorithm
void LateralControllerMPC::initialize()
{
    wX_ = config_["parking"]["mpc"]["wX"].as<double>();
    wY_ = config_["parking"]["mpc"]["wY"].as<double>();
    wYaw_ = config_["parking"]["mpc"]["wYaw"].as<double>();
    wDelta_ = config_["parking"]["mpc"]["wDelta"].as<double>();
    wDeltarate_ = config_["parking"]["mpc"]["wDeltarate"].as<double>();
    L_f_ = config_["parking"]["mpc"]["L_f_"].as<double>();
    L_r_ = config_["parking"]["mpc"]["L_r_"].as<double>();
    minSteeringAngle_ = config_["parking"]["mpc"]["minSteeringAngle"].as<double>();
    maxSteeringAngle_ = config_["parking"]["mpc"]["maxSteeringAngle"].as<double>();
    kP = config_["parking"]["mpc"]["kP"].as<double>();
    kI = config_["parking"]["mpc"]["kI"].as<double>();
    
    mpcLatSetParam(wX_, wY_, wYaw_, wDelta_,wDeltarate_, L_f_, L_r_);

    std::cout << "Start MPC lateral controller..." << std::endl;
    
    controlOutput = initLatAcado();
    preSteering = controlOutput;
}

/******************************************************************************/
/// @brief fit the way points to polynominal function
Eigen::VectorXd LateralControllerMPC::polyfit(const adas::Trajectory& refTrajectory, int order) 
{

    int index = refTrajectory.trajectory_.size();
    Eigen::VectorXd xvals(index);
    Eigen::VectorXd yvals(index);
    for (unsigned int i = 0; i < refTrajectory.trajectory_.size(); i++)
    {
        // Convert from rear to COG
        double theta = refTrajectory.trajectory_.at(i).theta_;
        xvals(i) = refTrajectory.trajectory_.at(i).x_;
        yvals(i) = refTrajectory.trajectory_.at(i).y_;
    }
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    auto Q = A.householderQr();
    auto resultEigen = Q.solve(yvals);

    return resultEigen;
}



/******************************************************************************/
/// @brief set initial position
adas::Pose2f LateralControllerMPC::setInitialPosition()
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
std::vector<double> LateralControllerMPC::getDesiredSteeringAngle()
{
    std::vector<double> result;
    std::vector<double> predictedStates;
    std::vector<double> curState;
    Eigen::VectorXd polyCoeff;
    std::vector<double> refStates;
    std::vector<double> refStatesTr;

    adas::Pose2f initialPos;
    
    initialPos = setInitialPosition();

    if (desiredTrajectory_.empty() )
    {
        std::cout << "Empty trajectory" << std::endl;
        steeringAngle = 0;
        deltaRate = 0;
        controlOutput = resetControl();
        preSteering = controlOutput;
        return {0, 0};
    }
    else
    {
        // curState.push_back(initialPos.x());
        // curState.push_back(initialPos.y());
        // curState.push_back(initialPos.theta());

        curState.push_back(0);
        curState.push_back(0);
        curState.push_back(0);


        vX = egoVehicleState_.b_vel_.linear().x();

        double vY = egoVehicleState_.b_vel_.linear().y();
        double beta = atan(vY/vX);

        std::cout << "vx: " << vX << std::endl;
        std::cout << "vy: " << vY << std::endl;
        std::cout << "beta state: " << beta << std::endl;

        predictedStates = motionLatPrediction(curState, controlOutput, vX, desiredTrajectory_);
        std::cout << "trajectory size: " <<  desiredTrajectory_.trajectory_.size() << std::endl;
        

        for (int i = 0; i < desiredTrajectory_.trajectory_.size(); i ++)
        {
            double xGlobal = desiredTrajectory_.trajectory_.at(i).x_ - initialPos.x();
            double yGlobal = desiredTrajectory_.trajectory_.at(i).y_ - initialPos.y();
            double theta = desiredTrajectory_.trajectory_.at(i).theta_;

            double xLocal = cos(theta)*xGlobal - sin(theta)*yGlobal;
            double yLocal = sin(theta)*xGlobal + cos(theta)*yGlobal;
            // double headingAngle = atan2(yLocal, xLocal);
            theta = theta - initialPos.theta();

            desiredTrajectory_.trajectory_.at(i).x_ = xLocal;
            desiredTrajectory_.trajectory_.at(i).y_ = yLocal;
            desiredTrajectory_.trajectory_.at(i).theta_ = theta;

            if (egoVehicleState_.gear_status_ == adas::GEAR_STATUS::R)
            {
                desiredTrajectory_.trajectory_.at(i).v_ = -desiredTrajectory_.trajectory_.at(i).v_;
            }

        }

        if (vX != 0.0)
        {
            refStates = calculateLatRefStates(desiredTrajectory_);

            controlOutput = runLatMpcAcado(predictedStates, refStates, vX, desiredTrajectory_, preSteering);
            steeringAngle = controlOutput.at(0);
            steeringAngle = constrain(steeringAngle, -0.52, 0.52);
        }
        else 
        {
            for (int i = 0; i < controlOutput.size(); i++)
            {
                controlOutput.at(i) = desiredTrajectory_.trajectory_.at(i+1).vtheta_;
            }   
            steeringAngle = controlOutput.at(0);   
            steeringAngle = constrain(steeringAngle, -0.52, 0.52);
        }

        yawError = desiredTrajectory_.trajectory_.at(1).theta_;
        pPart = kP*yawError;
        iPart += kI*yawError*0.04;
        // if (abs(yawError) < 0.001)
        // {
        //     addSteer = 0;
        //     iPart = 0;
        // }
        // else
        // {
            addSteer = pPart + iPart;
        // }
        
        std::cout << "add steer: " << addSteer << std::endl;


        steeringAngle = steeringAngle + addSteer;
  
        steeringAngle = constrain(steeringAngle, -0.55, 0.55);

        result.push_back(steeringAngle);
        result.push_back(deltaRate);

        // std::cout << "latControl: " << std::endl;
        // std::cout << "Steering: " << steeringAngle  << std::endl;
        // std::cout << "Steering - preSteering: " << steeringAngle - preSteering.at(0) << std::endl << std::endl;

        preSteering = controlOutput;

        return result;
    }   
}

/******************************************************************************/
/// @brief update vehicle state
void LateralControllerMPC::updateVehicleState(adas::VehicleState& vehicleState)
{
    egoVehicleState_ = vehicleState;
}

/******************************************************************************/
/// @brief update global position
void LateralControllerMPC::updateVehicleGlobalPosition(adas::VehicleState& vehicleGlobalPos)
{
    vehicleGlobalPosition_ = vehicleGlobalPos;
}

/******************************************************************************/
/// @brief update the desired trajectory
void LateralControllerMPC::updateTrajectory(adas::Trajectory& trajectory)
{
    desiredTrajectory_ = trajectory;
}

LateralControllerPP::LateralControllerPP()
{

}

LateralControllerPP::~LateralControllerPP()
{

}

void LateralControllerPP::setConfig(YAML::Node& config)
{
    config_ = config;
}

/******************************************************************************/
/// @brief Init Acado algorithm
void LateralControllerPP::initialize()
{
    std::vector<float> numerators = {0.5};
    std::vector<float> denominators = {1.0, -0.5};

    latErrKp_ = config_["parking"]["pp"]["latErrKp"].as<double>();
    latErrKi_ = config_["parking"]["pp"]["latErrKi"].as<double>();
    latErrKd_ = config_["parking"]["pp"]["latErrKd"].as<double>();
    latErrTs_ = config_["parking"]["pp"]["latErrTs"].as<double>();
    lookAheadTime_ = config_["parking"]["pp"]["lookAheadTime"].as<double>();
    //lookAheadCompensateDistance_ = config_["parking"]["pp"]["lookAheadCompensateDistance"].as<double>();
    minLookAheadDistance_ = config_["parking"]["pp"]["minLookAheadDistance"].as<double>();
    maxLookAheadDistance_ = config_["parking"]["pp"]["maxLookAheadDistance"].as<double>();
    wheelBase_ = config_["parking"]["pp"]["wheelBase"].as<double>();
    minSteeringWheelAngle_ = config_["parking"]["pp"]["minSteeringWheelAngle"].as<double>();
    maxSteeringWheelAngle_ = config_["parking"]["pp"]["maxSteeringWheelAngle"].as<double>();
    maxSteeringWheelAngleVel_ = config_["parking"]["pp"]["maxSteeringWheelAngleVel"].as<double>();
    minLatErrCompensation_ = config_["parking"]["pp"]["minLatErrCompensation"].as<double>();
    maxLatErrCompensation_ = config_["parking"]["pp"]["maxLatErrCompensation"].as<double>();
    kGain_ = config_["parking"]["pp"]["kGain"].as<double>();
    steeringRatio_ = config_["parking"]["pp"]["steeringRatio"].as<double>();

    ffSteeringOutput_.set_coefficients(denominators, numerators);

    pidLatErrController_.setParam(latErrTs_,
                                  minLatErrCompensation_,
                                  maxLatErrCompensation_,
                                  latErrKp_,
                                  latErrKi_,
                                  latErrKd_);

}

/******************************************************************************/
/// @brief get desired steering angle
std::vector<double> LateralControllerPP::getDesiredSteeringAngle()
{
    std::vector<double> result;
    double curSpeed = 0.0;
    double lookAheadDistance = 0.0;
    double headingAngle = 0.0;
    double steeringAngle = 0.0;
    static double prevSteeringAngle = 0.0;
    unsigned int index = 1;
    double latError = 0;
    std::vector<double> curState;

    if (desiredTrajectory_.trajectory_.size())
    {
        curSpeed = fabs(egoVehicleState_.b_vel_.linear().x());

        if (curSpeed >= kmh2Mps(0.01))
        {
            // lookAheadDistance = curSpeed *lookAheadTime_;
            // lookAheadDistance = constrain(lookAheadDistance, minLookAheadDistance_, maxLookAheadDistance_);

            // for (index = 0; index < desiredTrajectory_.trajectory_.size(); index++)
            // {
            //     if (lookAheadDistance <= desiredTrajectory_.trajectory_.at(index).s_)
            //     {
            //         break;
            //     }
            // }

            // if (index >= desiredTrajectory_.trajectory_.size())
            // {
            //     index = desiredTrajectory_.trajectory_.size() - 1;
            // }

            if (index == 0)
            {
                steeringAngle = 0.0;
                prevSteeringAngle = steeringAngle;
            }
            else
            {
                adas::TrajectoryPoint startPoint = desiredTrajectory_.trajectory_[0];
                adas::TrajectoryPoint lookAheadPoint;
                lookAheadPoint.x_ = desiredTrajectory_.trajectory_[1].vx_;
                lookAheadPoint.y_ = desiredTrajectory_.trajectory_[1].vy_;
                lookAheadPoint.theta_ = desiredTrajectory_.trajectory_[1].vz_;

                std::cout << "start point: x=" << startPoint.x_ << ", "
                          << "y=" << startPoint.y_ << ", "
                          << "theta=" << startPoint.theta_ << std::endl;

                std::cout << "lookahead point: x=" << lookAheadPoint.x_ << ", "
                          << "y=" << lookAheadPoint.y_ << ", "
                          << "theta=" << lookAheadPoint.theta_ << std::endl;

                double deltaX = lookAheadPoint.x_ - startPoint.x_;
                double deltaY = lookAheadPoint.y_ - startPoint.y_;
                lookAheadDistance = std::sqrt(deltaX*deltaX + deltaY*deltaY);

                std::cout << "lookAheadDistance = " << lookAheadDistance << std::endl;

                if (lookAheadPoint.x_ - startPoint.x_ == 0.0)
                {
                    steeringAngle = 0.0;
                    prevSteeringAngle = steeringAngle;
                }
                else
                {
                    // headingAngle = atan((lookAheadPoint.y_ - startPoint.y_) / (lookAheadPoint.x_ - startPoint.x_));
                    // headingAngle = atan2((lookAheadPoint.y_ - startPoint.y_) , (lookAheadPoint.x_ - startPoint.x_));
                    double x0 = startPoint.x_;
                    double y0 = startPoint.y_;
                    double q0 = startPoint.theta_;

                    double x1 = lookAheadPoint.x_;
                    double y1 = lookAheadPoint.y_;
                    double q1 = lookAheadPoint.theta_;

                    double x = x1 * cos(q0) - x0 * cos(q0) - y0 * sin(q0) + y1 * sin(q0);
                    double y = y1 * cos(q0) - y0 * cos(q0) + x0 * sin(q0) - x1 * sin(q0);
                    double q = q1 - q0;

                    headingAngle = atan2(y, x);
                    if (egoVehicleState_.gear_status_ == adas::GEAR_STATUS::R)
                    {
                        steeringAngle = kGain_ * atan((2 * wheelBase_ * sin(headingAngle)) / lookAheadDistance);
                    }
                    else
                    {
                        steeringAngle = kGain_ * atan((2 * wheelBase_ * sin(headingAngle)) / lookAheadDistance);
                    }
                    
                    latError = startPoint.d_;
                    // steeringAngle = steeringAngle - pidLatErrController_.calculate(latError);
                    steeringAngle = ffSteeringOutput_.Filter(steeringAngle);
                    steeringAngle = constrain(steeringAngle, deg2Rad(minSteeringWheelAngle_) / steeringRatio_,
                                                             deg2Rad(maxSteeringWheelAngle_) / steeringRatio_);
                    prevSteeringAngle = steeringAngle;
                }
            }
        }
        else
        {
            steeringAngle = prevSteeringAngle;
        }

        result.push_back(steeringAngle);
        result.push_back(0.0);
    }
    else
    {
        result.push_back(0.0);
        result.push_back(0.0);
    }

    return result;
}

/******************************************************************************/
/// @brief update vehicle state
void LateralControllerPP::updateVehicleState(const adas::VehicleState& vehicleState)
{
    egoVehicleState_ = vehicleState;
}

/******************************************************************************/
/// @brief update the desired trajectory
void LateralControllerPP::updateTrajectory(const adas::Trajectory& trajectory)
{
    desiredTrajectory_ = trajectory;
}

} // mpc

} // dds

} // adas
