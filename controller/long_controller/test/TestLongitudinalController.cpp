/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#include "../include/LongitudinalController.hpp"
#include <gtest/gtest.h>

using namespace adas;
TEST(LongitudinalControllerTest, PitchAngleEstimationTest1)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 0.0;
    egoVehicleState.b_acc_.linear().y() = 0.0;
    egoVehicleState.b_acc_.linear().z() = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest2)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 0.0;
    egoVehicleState.b_acc_.linear().y() = 1.0;
    egoVehicleState.b_acc_.linear().z() = 1.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest3)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 1.0;
    egoVehicleState.b_acc_.linear().y() = 0.0;
    egoVehicleState.b_acc_.linear().z() = 1.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest4)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 1.0;
    egoVehicleState.b_acc_.linear().y() = 1.0;
    egoVehicleState.b_acc_.linear().z() = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest5)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 1.0;
    egoVehicleState.b_acc_.linear().y() = 0.0;
    egoVehicleState.b_acc_.linear().z() = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest6)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 0.0;
    egoVehicleState.b_acc_.linear().y() = 1.0;
    egoVehicleState.b_acc_.linear().z() = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest7)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 1.0;
    egoVehicleState.b_acc_.linear().y() = 0.0;
    egoVehicleState.b_acc_.linear().z() = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, 0.0);
}

TEST(LongitudinalControllerTest, PitchAngleEstimationTest8)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    double pitchAngle = 0.0;

    egoVehicleState.b_acc_.linear().x() = 1.5;
    egoVehicleState.b_acc_.linear().y() = 0.4;
    egoVehicleState.b_acc_.linear().z() = 0.2;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.pitchAngleEstimation();
    pitchAngle = longitudinalController.getPitchAngle();

    ASSERT_FLOAT_EQ(pitchAngle, -0.2617994);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel1)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 0.0);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel2)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);
    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 0.0);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel3)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;
    adas::TrajectoryPoint waypoint;

    waypoint.s_ = 0.0;
    waypoint.v_ = 0.0;
    waypoint.a_ = 0.0;
    desiredTrajectory.trajectory_.push_back(waypoint);

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);
    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 0.0);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel4)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    for (unsigned int i = 0; i < 50; i++)
    {
        adas::TrajectoryPoint waypoint;

        waypoint.s_ = 0.0;
        waypoint.v_ = 0.0;
        waypoint.a_ = 0.0;

        desiredTrajectory.trajectory_.push_back(waypoint);
    }

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);
    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 0.0);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel5)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    for (unsigned int i = 0; i < 100; i++)
    {
        adas::TrajectoryPoint waypoint;

        waypoint.s_ = 0.0;
        waypoint.v_ = 0.0;
        waypoint.a_ = 0.0;

        desiredTrajectory.trajectory_.push_back(waypoint);
    }

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);
    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 0.0);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel6)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    for (unsigned int i = 0; i < 100; i++)
    {
        adas::TrajectoryPoint waypoint;

        waypoint.s_ = i * 3.0;
        waypoint.v_ = i * 1.0;
        waypoint.a_ = i * 1.0;

        desiredTrajectory.trajectory_.push_back(waypoint);
    }

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);
    acceleration = longitudinalController.getDesiredAccelDecel();

    ASSERT_FLOAT_EQ(acceleration, 1.5);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel7)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    for (unsigned int i = 0; i < 100; i++)
    {
        adas::TrajectoryPoint waypoint;

        waypoint.s_ = i * 3.0;
        waypoint.v_ = i * 1.0;
        waypoint.a_ = i * 1.0;

        desiredTrajectory.trajectory_.push_back(waypoint);
    }

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);

    for (unsigned int i = 0; i < 5; i++)
    {
        acceleration = longitudinalController.getDesiredAccelDecel();
    }

    ASSERT_NEAR(acceleration, 3.0, 0.1);
}

TEST(LongitudinalControllerTest, GetDesiredAccelDecel8)
{
    adas::dds::mpc::LongitudinalControllerMPC longitudinalController;
    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    double acceleration = 0.0;

    for (unsigned int i = 0; i < 100; i++)
    {
        adas::TrajectoryPoint waypoint;

        waypoint.s_ = i * 3.0;
        waypoint.v_ = (50 - i/2) * 1.0;
        waypoint.a_ = -1.0 * (i * 1.0);

        desiredTrajectory.trajectory_.push_back(waypoint);
    }

    egoVehicleState.b_vel_.linear().x() = 100.0;

    longitudinalController.updateVehicleState(egoVehicleState);
    longitudinalController.updateTrajectory(desiredTrajectory);

    for (unsigned int i = 0; i < 25; i++)
    {
        acceleration = longitudinalController.getDesiredAccelDecel();
    }

    ASSERT_NEAR(acceleration, -4.0, 0.1);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}