/******************************************************************************
 * OCS2 Wheel Humanoid ROS - MPC Node
 * 
 * Main MPC controller node for wheel humanoid robot.
 ******************************************************************************/

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <whole_body_mpc/WholeBodyInterface.h>

using namespace ocs2;
using namespace ocs2::whole_body_mpc;

int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create node
    auto node = std::make_shared<rclcpp::Node>("whole_body_mpc",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));
    
    // Get parameters
    std::string taskFile, urdfFile, libraryFolder;
    node->get_parameter_or<std::string>("task_file", taskFile, "");
    node->get_parameter_or<std::string>("urdf_file", urdfFile, "");
    node->get_parameter_or<std::string>("library_folder", libraryFolder, "/tmp/whole_body_mpc");
    
    if (taskFile.empty() || urdfFile.empty()) {
        RCLCPP_ERROR(node->get_logger(), 
            "task_file and urdf_file parameters are required!");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Task file: %s", taskFile.c_str());
    RCLCPP_INFO(node->get_logger(), "URDF file: %s", urdfFile.c_str());
    RCLCPP_INFO(node->get_logger(), "Library folder: %s", libraryFolder.c_str());
    
    // Create wheel humanoid interface
    WholeBodyInterface interface(taskFile, libraryFolder, urdfFile);
    
    // Get reference manager and wrap with ROS interface
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
        "whole_body", interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(node);
    
    // Create MPC
    auto mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(
        interface.mpcSettings(),
        interface.ddpSettings(),
        interface.getRollout(),
        interface.getOptimalControlProblem(),
        interface.getInitializer());
    mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    
    // Create MPC ROS interface
    MPC_ROS_Interface mpcNode(*mpcPtr, "whole_body");
    mpcNode.launchNodes(node);
    
    RCLCPP_INFO(node->get_logger(), "Wheel Humanoid MPC node started");
    
    // Spin
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

