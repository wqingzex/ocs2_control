/******************************************************************************
 * OCS2 Wheel Humanoid ROS - Dummy Node
 * 
 * Simulates robot state for testing MPC without real hardware.
 * Publishes joint states and TF for RViz visualization.
 ******************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <whole_body_mpc/WholeBodyInterface.h>

using namespace ocs2;
using namespace ocs2::whole_body_mpc;
using namespace ocs2::mobile_manipulator;

/******************************************************************************
 * Visualization Observer - publishes joint states, TF, and collision distances
 ******************************************************************************/
class WholeBodyVisualization : public DummyObserver {
public:
    WholeBodyVisualization(rclcpp::Node::SharedPtr node, 
                                const ManipulatorModelInfo& modelInfo,
                                const std::vector<std::string>& removeJointNames,
                                const std::string& taskFile,
                                const std::string& urdfFile)
        : node_(node)
        , modelInfo_(modelInfo)
        , removeJointNames_(removeJointNames)
        , tfBroadcaster_(node) {
        
        jointPublisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        
        // Initialize collision visualization if self-collision is enabled
        initializeCollisionVisualization(taskFile, urdfFile);
        
        RCLCPP_INFO(node_->get_logger(), "Visualization initialized with %zu DOFs", 
                    modelInfo_.dofNames.size());
    }
    
    void update(const SystemObservation& observation, 
                const PrimalSolution& policy,
                const CommandData& command) override {
        
        const rclcpp::Time timeStamp = node_->get_clock()->now();
        publishObservation(timeStamp, observation);
        
        // Publish collision distances
        if (geometryVisualization_ != nullptr) {
            geometryVisualization_->publishDistances(observation.state);
        }
    }
    
private:
    void initializeCollisionVisualization(const std::string& taskFile,
                                          const std::string& urdfFile) {
        try {
            // Read if self-collision checking active
            boost::property_tree::ptree pt;
            boost::property_tree::read_info(taskFile, pt);
            bool activateSelfCollision = true;
            loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
            
            if (!activateSelfCollision) {
                RCLCPP_INFO(node_->get_logger(), "Self-collision visualization disabled");
                return;
            }
            
            // Read manipulator model type
            ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(
                taskFile, "model_information.manipulatorModelType");
            
            // Create Pinocchio interface (with removeJoints for consistency)
            PinocchioInterface pinocchioInterface(
                mobile_manipulator::createPinocchioInterface(urdfFile, modelType, removeJointNames_));
            
            // Load collision pairs - both link pairs (string) and object pairs (index)
            std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
            std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
            loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs",
                                          collisionLinkPairs, true);
            loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs",
                                          collisionObjectPairs, true);
            
            if (collisionLinkPairs.empty() && collisionObjectPairs.empty()) {
                RCLCPP_WARN(node_->get_logger(), "No collision pairs defined, skipping visualization");
                return;
            }
            
            // Create geometry interface with URDF for collision geometry
            PinocchioGeometryInterface geomInterface(pinocchioInterface, urdfFile,
                                                     collisionLinkPairs, collisionObjectPairs);
            
            // Markers on this node so distance_markers follows its namespace.
            geometryVisualization_ = std::make_shared<GeometryInterfaceVisualization>(
                node_->get_node_topics_interface(), node_->get_clock(),
                std::move(pinocchioInterface), geomInterface);
            
            RCLCPP_INFO(node_->get_logger(), "Self-collision visualization enabled with %zu link pairs + %zu object pairs",
                        collisionLinkPairs.size(), collisionObjectPairs.size());
                        
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to initialize collision visualization: %s", e.what());
        }
    }
    
    void publishObservation(const rclcpp::Time& timeStamp, 
                            const SystemObservation& observation) {
        // Publish world -> base transform
        const auto r_world_base = getBasePosition(observation.state, modelInfo_);
        const Eigen::Quaternion<scalar_t> q_world_base = 
            getBaseOrientation(observation.state, modelInfo_);
        
        geometry_msgs::msg::TransformStamped base_tf;
        base_tf.header.stamp = timeStamp;
        base_tf.header.frame_id = "world";
        base_tf.child_frame_id = modelInfo_.baseFrame;
        base_tf.transform.translation.x = r_world_base.x();
        base_tf.transform.translation.y = r_world_base.y();
        base_tf.transform.translation.z = r_world_base.z();
        base_tf.transform.rotation.w = q_world_base.w();
        base_tf.transform.rotation.x = q_world_base.x();
        base_tf.transform.rotation.y = q_world_base.y();
        base_tf.transform.rotation.z = q_world_base.z();
        tfBroadcaster_.sendTransform(base_tf);
        
        // Publish joint states
        const auto j_arm = getArmJointAngles(observation.state, modelInfo_);
        
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = timeStamp;
        
        const auto dofNames_count = modelInfo_.dofNames.size();
        const auto joint_count = dofNames_count + removeJointNames_.size();
        joint_state.name.resize(joint_count);
        joint_state.position.resize(joint_count);
        
        // Active joints
        for (size_t i = 0; i < dofNames_count; i++) {
            joint_state.name[i] = modelInfo_.dofNames[i];
            joint_state.position[i] = j_arm(i);
        }
        
        // Fixed joints (removed from model)
        auto joint_state_index = dofNames_count;
        for (const auto& name : removeJointNames_) {
            joint_state.name[joint_state_index] = name;
            joint_state.position[joint_state_index] = 0.0;
            joint_state_index++;
        }
        
        jointPublisher_->publish(joint_state);
    }
    
    rclcpp::Node::SharedPtr node_;
    const ManipulatorModelInfo& modelInfo_;
    std::vector<std::string> removeJointNames_;
    
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;
    
    // Collision visualization
    std::shared_ptr<GeometryInterfaceVisualization> geometryVisualization_;
};

/******************************************************************************
 * Main
 ******************************************************************************/
int main(int argc, char* argv[]) {
    const std::string robotName = "whole_body";
    
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create node
    auto node = rclcpp::Node::make_shared(
        robotName + "_mrt",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));
    
    // Get parameters
    std::string taskFile, urdfFile, libraryFolder;
    node->get_parameter_or<std::string>("task_file", taskFile, "");
    node->get_parameter_or<std::string>("urdf_file", urdfFile, "");
    node->get_parameter_or<std::string>("library_folder", libraryFolder, "/tmp/whole_body_mpc");
    
    if (taskFile.empty() || urdfFile.empty()) {
        RCLCPP_ERROR(node->get_logger(), "task_file and urdf_file parameters are required!");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Task file: %s", taskFile.c_str());
    RCLCPP_INFO(node->get_logger(), "URDF file: %s", urdfFile.c_str());
    RCLCPP_INFO(node->get_logger(), "Library folder: %s", libraryFolder.c_str());
    
    // Create interface
    WholeBodyInterface interface(taskFile, libraryFolder, urdfFile);
    
    // Load removed joints from task file
    std::vector<std::string> removeJointNames;
    loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", 
                                          removeJointNames, false);
    
    // Create MRT interface
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(node);
    
    // Create visualization
    auto visualization = std::make_shared<WholeBodyVisualization>(
        node, interface.getModelInfo(), removeJointNames, taskFile, urdfFile);
    
    // Create Dummy Loop
    MRT_ROS_Dummy_Loop dummy(mrt, 
                             interface.mpcSettings().mrtDesiredFrequency_,
                             interface.mpcSettings().mpcDesiredFrequency_);
    dummy.subscribeObservers({visualization});
    
    // Initial observation
    SystemObservation initObservation;
    initObservation.state = interface.getInitialState();
    initObservation.input.setZero(interface.getModelInfo().inputDim);
    initObservation.time = 0.0;
    
    // Compute initial target from initial state using forward kinematics
    vector_t initTarget(21);
    {
        const auto& pinocchioInterface = interface.getPinocchioInterface();
        const auto& model = pinocchioInterface.getModel();
        auto data = pinocchioInterface.getData();
        
        // Forward kinematics
        pinocchio::forwardKinematics(model, data, initObservation.state);
        pinocchio::updateFramePlacements(model, data);
        
        const auto& modelInfo = interface.getModelInfo();
        
        // Left arm end-effector
        // Quaternion storage order: [x, y, z, w] (Eigen coeffs() order)
        auto leftEEFrameId = model.getFrameId(modelInfo.eeFrame);
        if (leftEEFrameId < model.nframes) {
            const auto& leftPose = data.oMf[leftEEFrameId];
            initTarget.segment<3>(0) = leftPose.translation();
            Eigen::Quaterniond leftQuat(leftPose.rotation());
            initTarget.segment<4>(3) << leftQuat.x(), leftQuat.y(), leftQuat.z(), leftQuat.w();
            RCLCPP_INFO(node->get_logger(), "Left EE initial: pos[%.3f, %.3f, %.3f] quat[x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                leftPose.translation().x(), leftPose.translation().y(), leftPose.translation().z(),
                leftQuat.x(), leftQuat.y(), leftQuat.z(), leftQuat.w());
        } else {
            RCLCPP_WARN(node->get_logger(), "Left EE frame not found: %s", modelInfo.eeFrame.c_str());
            initTarget.segment<3>(0) << 0.4, 0.3, 1.0;
            initTarget.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;  // [x, y, z, w] identity
        }
        
        // Right arm end-effector
        auto rightEEFrameId = model.getFrameId(modelInfo.eeFrame1);
        if (rightEEFrameId < model.nframes) {
            const auto& rightPose = data.oMf[rightEEFrameId];
            initTarget.segment<3>(7) = rightPose.translation();
            Eigen::Quaterniond rightQuat(rightPose.rotation());
            initTarget.segment<4>(10) << rightQuat.x(), rightQuat.y(), rightQuat.z(), rightQuat.w();
            RCLCPP_INFO(node->get_logger(), "Right EE initial: pos[%.3f, %.3f, %.3f] quat[x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                rightPose.translation().x(), rightPose.translation().y(), rightPose.translation().z(),
                rightQuat.x(), rightQuat.y(), rightQuat.z(), rightQuat.w());
        } else {
            RCLCPP_WARN(node->get_logger(), "Right EE frame not found: %s", modelInfo.eeFrame1.c_str());
            initTarget.segment<3>(7) << 0.4, -0.3, 1.0;
            initTarget.segment<4>(10) << 0.0, 0.0, 0.0, 1.0;  // [x, y, z, w] identity
        }
        
        // Body frame
        auto bodyFrameId = model.getFrameId(modelInfo.bodyFrame);
        if (bodyFrameId < model.nframes) {
            const auto& bodyPose = data.oMf[bodyFrameId];
            initTarget.segment<3>(14) = bodyPose.translation();
            Eigen::Quaterniond bodyQuat(bodyPose.rotation());
            initTarget.segment<4>(17) << bodyQuat.x(), bodyQuat.y(), bodyQuat.z(), bodyQuat.w();
            RCLCPP_INFO(node->get_logger(), "Body initial: pos[%.3f, %.3f, %.3f] quat[x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                bodyPose.translation().x(), bodyPose.translation().y(), bodyPose.translation().z(),
                bodyQuat.x(), bodyQuat.y(), bodyQuat.z(), bodyQuat.w());
        } else {
            RCLCPP_WARN(node->get_logger(), "Body frame not found: %s", modelInfo.bodyFrame.c_str());
            initTarget.segment<3>(14) << 0.0, 0.0, 1.0;
            initTarget.segment<4>(17) << 0.0, 0.0, 0.0, 1.0;  // [x, y, z, w] identity
        }
    }
    
    const vector_t zeroInput = vector_t::Zero(interface.getModelInfo().inputDim);
    const TargetTrajectories initTargetTrajectories(
        {initObservation.time}, {initTarget}, {zeroInput});
    
    RCLCPP_INFO(node->get_logger(), "Wheel Humanoid Dummy node started");
    
    // Run dummy loop
    dummy.run(initObservation, initTargetTrajectories);
    
    rclcpp::shutdown();
    return 0;
}
