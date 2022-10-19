#include <common/node_handle_pro.h>

#include "UnifiedController.hpp"

#include "output_interfaces/include/LowPass.hpp"

#include "PubSub.hpp"

#include <iostream>
#include <mutex>
#include <chrono>
#include <unistd.h>

int main(int argc, char *argv[])
{
    if (argc != 2) 
    {
        std::cout << "Usage: ./lat_controller <path/to/config/file>" << std::endl;
        return 0;
    }
    YAML::Node config = YAML::LoadFile(argv[1]);

    adas::dds::NodeHandle nh(0);
    nh.init_params(argc, argv);
    int hz = 25;
    std::vector<double> unifiedController;
    adas::dds::mpc::LowPass lpfSteering;

    adas::VehicleState egoVehicleState;
    adas::VehicleState vehicleGlobalPosition;

    adas::GearStatus desiredGear = adas::GearStatus::P;
    adas::Trajectory desiredTrajectory;
    adas_dds::Twist twist;
    adas_dds::Twist_initialize(&twist);

    // lpfSteering.setParams(0.1, 0.03);
    lpfSteering.setParams(0.27, 0.03);

    // Subscribers
    adas::dds::mpc::VehicleStateSubscriber vehicleStateSubscriber(nh, ADAS_DDS_MPC_TOPICS_INPUT_INTERFACES_VEHI_STATE);
    adas::dds::mpc::VehicleStateSubscriber vehiclePosSubscriber(nh, ADAS_DDS_MPC_TOPICS_INPUT_INTERFACES_VEHI_LOCALIZATION);
    adas::dds::mpc::TrajectorySubscriber trajectorySubscriber(nh, ADAS_DDS_MPC_TOPICS_LOCAL_PLANNER_TRAJECTORY);
    adas::dds::mpc::GearSubscriber gearSubscriber(nh, ADAS_DDS_MPC_TOPICS_REQUIRED_GEAR);

    // Publisher
    adas::dds::mpc::TwistPublisher twistPublisher(nh, ADAS_DDS_MPC_TOPICS_MOTION_CONTROLLER_LAT_TWIST);

    adas::dds::mpc::UnifiedControllerMPC unifiedControllerMPC;

    adas::dds::mpc::GuiPlotPublisher plotPub(nh, ADAS_DDS_MPC_TOPICS_GUI_PLOT);

    unifiedControllerMPC.setConfig(config);
    unifiedControllerMPC.initialize();

    while (true)
    {
        if (vehicleStateSubscriber.dataIsAvail() && vehiclePosSubscriber.dataIsAvail() && trajectorySubscriber.dataIsAvail())
        {
            auto start = std::chrono::steady_clock::now();
            egoVehicleState = vehicleStateSubscriber.getData();
            vehicleGlobalPosition = vehiclePosSubscriber.getData();
            desiredTrajectory = trajectorySubscriber.getData();
            desiredGear = static_cast<adas::GearStatus>(gearSubscriber.getData().data);

            unifiedControllerMPC.updateVehicleState(egoVehicleState);
            unifiedControllerMPC.updateVehicleGlobalPosition(vehicleGlobalPosition);
            unifiedControllerMPC.updateTrajectory(desiredTrajectory);
            unifiedControllerMPC.updateDesiredGear(desiredGear);

            unifiedController = unifiedControllerMPC.getControlSignal();

            twist.angular.x = unifiedController.at(0);

            twist.linear.x = unifiedController.at(2);
            twist.linear.z = unifiedController.at(1);


            std::cout << "Controller [steeringAngle, Acceleration, Velocity]: " << std::endl;
            std::cout << "steeringAngle: " << twist.angular.x << std::endl;
            std::cout << "Acceleration: " << twist.linear.z << std::endl;  
            std::cout << "Velocity: " << twist.linear.x << std::endl << std::endl; 
            
            auto end = std::chrono::steady_clock::now();        
            // std::cout << "Elapsed time in microseconds: "
            //           << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            //           << " µs" << std::endl;
                      
            twistPublisher.publishMsg(twist);

            std::vector<float> value;
            value.push_back(twist.angular.x * 180 * 16.0 / M_PI);
            plotPub.createAndPublish(&value.front(), value.size());
        }
        else
        {
            twist.angular.x = 0.0;
            twist.angular.z = 0.0; 
            twistPublisher.publishMsg(twist);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / hz));
    }
    
    return 0;
}