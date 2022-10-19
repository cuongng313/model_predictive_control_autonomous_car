#include <common/node_handle_pro.h>

#include "LateralController.hpp"

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
    std::vector<double> latControl;
    adas::dds::mpc::LowPass lpfSteering;

    adas::VehicleState egoVehicleState;
    adas::VehicleState vehicleGlobalPosition;
    adas::Trajectory desiredTrajectory;
    adas_dds::Twist twist;
    adas_dds::Twist_initialize(&twist);

    // lpfSteering.setParams(0.1, 0.03);
    lpfSteering.setParams(0.27, 0.03);

    // Subscribers
    adas::dds::mpc::VehicleStateSubscriber vehicleStateSubscriber(nh, ADAS_DDS_MPC_TOPICS_INPUT_INTERFACES_VEHI_STATE);
    adas::dds::mpc::VehicleStateSubscriber vehiclePosSubscriber(nh, ADAS_DDS_MPC_TOPICS_INPUT_INTERFACES_VEHI_LOCALIZATION);
    adas::dds::mpc::TrajectorySubscriber trajectorySubscriber(nh, ADAS_DDS_MPC_TOPICS_LOCAL_PLANNER_TRAJECTORY);

    // Publisher
    adas::dds::mpc::TwistPublisher twistPublisher(nh, ADAS_DDS_MPC_TOPICS_MOTION_CONTROLLER_LAT_TWIST);

    // adas::dds::mpc::LateralControllerPP lateralController;
    adas::dds::mpc::LateralControllerMPC lateralController;

    adas::dds::mpc::GuiPlotPublisher plotPub(nh, ADAS_DDS_MPC_TOPICS_GUI_PLOT);

    lateralController.setConfig(config);
    lateralController.initialize();

    while (true)
    {
        if (vehicleStateSubscriber.dataIsAvail() && vehiclePosSubscriber.dataIsAvail() && trajectorySubscriber.dataIsAvail())
        {
            auto start = std::chrono::steady_clock::now();
            egoVehicleState = vehicleStateSubscriber.getData();
            vehicleGlobalPosition = vehiclePosSubscriber.getData();
            desiredTrajectory = trajectorySubscriber.getData();

            lateralController.updateVehicleState(egoVehicleState);
            lateralController.updateVehicleGlobalPosition(vehicleGlobalPosition);
            lateralController.updateTrajectory(desiredTrajectory);

            latControl = lateralController.getDesiredSteeringAngle();

            twist.angular.x = latControl.at(0);
            twist.angular.z = latControl.at(1);

            // if (desiredTrajectory.trajectory_.size() > 2)
            // {
            //     lpfSteering.filt(desiredTrajectory.trajectory_.at(1).vtheta_);
            //     twist.angular.x = lpfSteering.get();
            // }
            // else
            // {
            //     twist.angular.x = lpfSteering.get();
            // }     

            // std::cout << "latControl: " << std::endl;
            // std::cout << "angular: " << twist.angular.x << std::endl;
            // std::cout << "angular rate: " << twist.angular.z << std::endl << std::endl;   
            
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