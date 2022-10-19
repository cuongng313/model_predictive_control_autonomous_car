/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "LongitudinalController.hpp"

namespace adas
{

namespace dds
{

namespace mpc
{

LongitudinalControllerPID::LongitudinalControllerPID()
{
    std::vector<float> numerators = {0.5};
    std::vector<float> denominators = {1.0, -0.5};
    ffAccFilter_.set_coefficients(denominators, numerators);
    pitchAngleFilter_.set_coefficients(denominators, numerators);

    maxAcc_ = vehConfigReader_.vehicleConstraints.maxAcc;
    maxDec_ = vehConfigReader_.vehicleConstraints.maxDec;
    mass_ = vehConfigReader_.dynamicsParams.m;

    pidController_.setParam(ts_, maxAcc_, maxDec_, kp_, ki_, kd_);
}

LongitudinalControllerPID::~LongitudinalControllerPID()
{

}

/******************************************************************************/
/// @brief get desired acceleration/deceleration
double LongitudinalControllerPID::getDesiredAccelDecel()
{
    double result = 0.0;
    double curSpeed = 0.0;
    double desSpeed = 0.0;
    double ffAcc = 0.0;
    double errorSpeed = 0.0;
    unsigned int i = 1;
    double tmpSpeed = 0.0;
    bool onLCCMode = false; 
    adas::TrajectoryPoint startPoint;
    adas::TrajectoryPoint lookAheadPoint;

    // pitchAngleEstimation();
    if (onLCCMode)
    {
        if (desiredTrajectory_.trajectory_.size() >= minWaypointInTraj_)
        {
            startPoint = desiredTrajectory_.trajectory_.at(0);
            curSpeed = egoVehicleState_.b_vel_.linear().x();

            while (tmpSpeed == 0.0 && i < desiredTrajectory_.trajectory_.size())
            {
                lookAheadPoint = desiredTrajectory_.trajectory_.at(desiredTrajectory_.trajectory_.size() - i);
                tmpSpeed = lookAheadPoint.v_;
                i++;
            }

            if (desiredTrajectory_.trajectory_.size() - i <= minWaypointInTraj_)
            {
                if (curSpeed <= kmh2Mps(1.0))
                {
                    result = standStillDec_;
                }
                else
                {
                    result = maxDec_;
                }
            }
            else
            {
                desSpeed = lookAheadPoint.v_;
                desiredVel_ = desSpeed;
                errorSpeed = desSpeed - curSpeed;

                if (fabs(errorSpeed) >= kmh2Mps(3.0) || curSpeed <= kmh2Mps(2.0))
                {
                    pidController_.resetIntegral();
                }

                ffAcc = (lookAheadPoint.v_ - curSpeed) / lookAheadPoint.t_;
                // ffAcc = ffAccFilter_.Filter((lookAheadPoint.v_ * lookAheadPoint.v_ - startPoint.v_ * startPoint.v_) / 
                //                             (2 * lookAheadPoint.s_));

                result = ffAcc + pidController_.calculate(errorSpeed);// + g_ * mass_ * sin(pitchAngle_);
                result = constrain(result, maxDec_, maxAcc_);
            }
        }
        else
        {
            result = ffAccFilter_.Filter(0.0);
        }
    }
    else //on ParkingMode
    { 
        if (desiredTrajectory_.trajectory_.size())
        {      
            curSpeed = egoVehicleState_.b_vel_.linear().x();
            std::cout <<"\n => curSpeed : "<< curSpeed << std::endl;
            if (desiredTrajectory_.trajectory_.size())
            {
                // desSpeed = desiredTrajectory_.trajectory_.at(desiredTrajectory_.trajectory_.size()/2 - 1).v_;
                desSpeed = desiredTrajectory_.trajectory_.at(50).v_;
                // desSpeed = desiredTrajectory_.trajectory_.at(1).v_;
            }
            else
            {
                desSpeed = 0.0;
            }
            std::cout <<"\n => desSpeed : "<< desSpeed << std::endl;
            
            desiredVel_ = desSpeed;
            errorSpeed = desSpeed - curSpeed;
            if (fabs(errorSpeed) >= kmh2Mps(3.0) || curSpeed <= kmh2Mps(2.0))
            {
                pidController_.resetIntegral();
            }

            ffAcc = 0.2;
            // ffAcc = desiredTrajectory_.trajectory_.at(1).a_;
            // ffAcc = (desSpeed - curSpeed) / ts_;
            result = ffAcc + pidController_.calculate(errorSpeed);// + g_ * mass_ * sin(pitchAngle_);
            result = constrain(result, maxDec_, maxAcc_);
        }
    }
    
    return result;
}

double LongitudinalControllerPID::getDesiredVel()
{
    return desiredVel_;
}

/******************************************************************************/
/// @brief update vehicle state
void LongitudinalControllerPID::updateVehicleState(const adas::VehicleState& vehicleState)
{
    egoVehicleState_ = vehicleState;
}

/******************************************************************************/
/// @brief update the desired trajectory
void LongitudinalControllerPID::updateTrajectory(const adas::Trajectory& trajectory)
{
    desiredTrajectory_ = trajectory;
}

/******************************************************************************/
/// @brief estimate the pitch angle
void LongitudinalControllerPID::pitchAngleEstimation()
{
    if (egoVehicleState_.b_acc_.linear().x() != 0 &&
        egoVehicleState_.b_acc_.linear().y() != 0 && 
        egoVehicleState_.b_acc_.linear().z() != 0)
    {
        pitchAngle_ = pow(tan(-egoVehicleState_.b_acc_.linear().x()/
                         (sqrt(egoVehicleState_.b_acc_.linear().y() * egoVehicleState_.b_acc_.linear().y()
                               + egoVehicleState_.b_acc_.linear().z() * egoVehicleState_.b_acc_.linear().z()))), -1);
        pitchAngle_ = pitchAngleFilter_.Filter(pitchAngle_);
        pitchAngle_ = constrain(pitchAngle_, deg2Rad(minPitchAngle_), deg2Rad(maxPitchAngle_));
    }
    else
    {
        pitchAngle_ = 0.0;
    }
}

double LongitudinalControllerPID::getPitchAngle()
{
    return pitchAngle_;
}

LongitudinalControllerMPC::LongitudinalControllerMPC()
{
    std::vector<float> numerators = {0.5};
    std::vector<float> denominators = {1.0, -0.5};
    outputAccFilter_.set_coefficients(denominators, numerators);
    pitchAngleFilter_.set_coefficients(denominators, numerators);
    lpfAccel_.setParams(0.5, 0.033);
    pidController_.setParam(ts_, maxAcc_, maxDec_, kp_, ki_, kd_);

    maxAcc_ = vehConfigReader_.vehicleConstraints.maxAcc;
    maxDec_ = vehConfigReader_.vehicleConstraints.maxDec;
    mass_ = vehConfigReader_.dynamicsParams.m;

    controlOutput_ = initAcado();
    preControlOutput_ = controlOutput_;
}

LongitudinalControllerMPC::~LongitudinalControllerMPC()
{

}

/******************************************************************************/
/// @brief get desired acceleration/deceleration
double LongitudinalControllerMPC::getDesiredAccelDecel()
{
    double result = 0.0;
    double curX = 0.0;
    double curSpeed = 0.0;
    double curAcc = 0.0;
    static double preAcc = 0.0;
    static double preSpeed = 0.0;
    std::vector<double> predictedStates;
    std::vector<double> curState;
    std::vector<double> refStates;

    pitchAngleEstimation();

    if (desiredTrajectory_.trajectory_.size() >= minWaypointInTraj_)
    {
        curX = 0.0;
        curSpeed = (egoVehicleState_.b_vel_.linear().x());
        desiredVel_ = desiredTrajectory_.trajectory_.at(desiredTrajectory_.trajectory_.size() - 1).v_;
        lpfAccel_.filt(constrain((curSpeed - preSpeed) * frequency_, -10.0, 10.0));
        curAcc = lpfAccel_.get();
        curState.push_back(curX);
        curState.push_back(curSpeed);
        curState.push_back(curAcc);

        predictedStates = motionPrediction(curState, controlOutput_);
        refStates = calculateRefStates(desiredTrajectory_);
        try
        {
            controlOutput_ = runMpcAcado(predictedStates, refStates);
            preControlOutput_ = controlOutput_;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            controlOutput_ = preControlOutput_;
        }
        
        result = controlOutput_.at(mpcSelectPoint_) + pidController_.calculate(preAcc - curAcc);// + g_ * mass_ * sin(pitchAngle_);
        result = outputAccFilter_.Filter(result);
        result = constrain(result, maxDec_, maxAcc_);

        preAcc = curAcc;
        preSpeed = curSpeed;

        // std::cout << "curSpeed = " << curSpeed << ", curAcc = " << curAcc << ", desControl = " << controlOutput_.at(mpcSelectPoint_) << std::endl;
    }
    else
    {
        result = outputAccFilter_.Filter(0.0);
    }
    
    return result;
}

double LongitudinalControllerMPC::getDesiredVel()
{
    return desiredVel_;
}

/******************************************************************************/
/// @brief update vehicle state
void LongitudinalControllerMPC::updateVehicleState(const adas::VehicleState& vehicleState)
{
    egoVehicleState_ = vehicleState;
}

/******************************************************************************/
/// @brief update the desired trajectory
void LongitudinalControllerMPC::updateTrajectory(const adas::Trajectory& trajectory)
{
    desiredTrajectory_ = trajectory;
}

/******************************************************************************/
/// @brief estimate the pitch angle
void LongitudinalControllerMPC::pitchAngleEstimation()
{
    if (egoVehicleState_.b_acc_.linear().x() != 0 &&
        egoVehicleState_.b_acc_.linear().y() != 0 && 
        egoVehicleState_.b_acc_.linear().z() != 0)
    {
        pitchAngle_ = pow(tan(-egoVehicleState_.b_acc_.linear().x()/
                         (sqrt(egoVehicleState_.b_acc_.linear().y() * egoVehicleState_.b_acc_.linear().y()
                               + egoVehicleState_.b_acc_.linear().z() * egoVehicleState_.b_acc_.linear().z()))), -1);
        pitchAngle_ = pitchAngleFilter_.Filter(pitchAngle_);
        pitchAngle_ = constrain(pitchAngle_, deg2Rad(minPitchAngle_), deg2Rad(maxPitchAngle_));
    }
    else
    {
        pitchAngle_ = 0.0;
    }
}

double LongitudinalControllerMPC::getPitchAngle()
{
    return pitchAngle_;
}

} // mpc

} // dds

} // adas
