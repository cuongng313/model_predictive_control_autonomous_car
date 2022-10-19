#include <common/node_handle_pro.h>

#include "LongitudinalController.hpp"

#include "MotionPlanningTopics.hpp"

#include <iostream>
#include <mutex>
#include <chrono>
#include <unistd.h>

int main(int argc, char *argv[])
{
    adas::dds::NodeHandle nh(0);
    nh.init_params(argc, argv);
    int hz = 25;
    double longControl = 0.0;
    double desVel = 0.0;

    adas::VehicleState egoVehicleState;
    adas::Trajectory desiredTrajectory;
    adas_dds::Twist* twist = adas_dds::TwistTypeSupport::create_data();
    twist->linear.x = 0.0;
    twist->linear.z = 0.0;
    // Subscribers
    adas::dds::mpc::VehicleStateSubscriber vehicleStateSubscriber(nh, ADAS_DDS_MPC_TOPICS_INPUT_INTERFACES_VEHI_STATE);
    adas::dds::mpc::TrajectorySubscriber trajectorySubscriber(nh, ADAS_DDS_MPC_TOPICS_LOCAL_PLANNER_TRAJECTORY);

    // Publisher
    adas::dds::mpc::TwistPublisher twistPublisher(nh, ADAS_DDS_MPC_TOPICS_MOTION_CONTROLLER_LONG_TWIST);

    adas::dds::mpc::LongitudinalControllerPID longitudinalController;

    while (true)
    {
        if (vehicleStateSubscriber.dataIsAvail() && trajectorySubscriber.dataIsAvail())
        {
            auto start = std::chrono::steady_clock::now();
            egoVehicleState = vehicleStateSubscriber.getData();
            desiredTrajectory = trajectorySubscriber.getData();

            longitudinalController.updateVehicleState(egoVehicleState);
            longitudinalController.updateTrajectory(desiredTrajectory);

            longControl = longitudinalController.getDesiredAccelDecel();
            desVel = longitudinalController.getDesiredVel();
            twist->linear.x = desVel;
            twist->linear.z = longControl;
            std::cout << "desVel: " << desVel << std::endl;
            std::cout << "longControl: " << longControl << std::endl;

            auto end = std::chrono::steady_clock::now();        
            std::cout << "Elapsed time in microseconds: "
                      << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                      << " µs" << std::endl;

            twistPublisher.publishMsg(*twist);
        }
        else
        {
            twist->linear.x = 0;
            twist->linear.z = 0;
            twistPublisher.publishMsg(*twist);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / hz));
    }

    adas_dds::TwistTypeSupport::delete_data(twist);

    return 0;
}
