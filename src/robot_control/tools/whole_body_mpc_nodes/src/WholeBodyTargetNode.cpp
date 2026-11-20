/******************************************************************************
 * OCS2 Wheel Humanoid ROS - Target Publisher Node
 * 
 * Publishes target trajectories and mode schedules for the MPC.
 * Provides interactive markers for setting end-effector targets.
 ******************************************************************************/

#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <whole_body_mpc/mode/HumanoidModeSchedule.h>
#include <whole_body_mpc/WholeBodyInterface.h>

using namespace ocs2;
using namespace ocs2::whole_body_mpc;

class WholeBodyTargetNode : public rclcpp::Node {
public:
    WholeBodyTargetNode()
        : Node("whole_body_target",
               rclcpp::NodeOptions()
                   .allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)) {
        
        // Publishers
        targetPub_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
            "whole_body_mpc_target", 10);
        modePub_ = create_publisher<ocs2_msgs::msg::ModeSchedule>(
            "whole_body_mode_schedule", 10);
        
        // Subscribers for interactive target setting
        leftTargetSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_arm_target", 10,
            std::bind(&WholeBodyTargetNode::leftTargetCallback, this, std::placeholders::_1));
        rightTargetSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "right_arm_target", 10,
            std::bind(&WholeBodyTargetNode::rightTargetCallback, this, std::placeholders::_1));
        bodyTargetSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "body_target", 10,
            std::bind(&WholeBodyTargetNode::bodyTargetCallback, this, std::placeholders::_1));
        
        // Mode command subscriber
        modeCommandSub_ = create_subscription<std_msgs::msg::String>(
            "mode_command", 10,
            std::bind(&WholeBodyTargetNode::modeCommandCallback, this, std::placeholders::_1));
        
        // Initialize targets with default values
        initializeDefaultTargets();
        
        // Interactive marker server
        markerServer_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
            "whole_body_targets", this);
        createInteractiveMarkers();
        
        // Timer to publish targets periodically
        publishTimer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WholeBodyTargetNode::publishTargets, this));
        
        RCLCPP_INFO(get_logger(), "Wheel Humanoid Target node started");
        RCLCPP_INFO(get_logger(), "Full presets (HM_*): HM_LEFT_ARM, HM_RIGHT_ARM, HM_DUAL_IDLE, HM_DUAL_BODY_FREE, HM_DUAL_RELATIVE_FIXED_BASE, HM_DUAL_BODY_TRACKING, HM_DUAL_RELATIVE_MOBILE_BASE");
        RCLCPP_INFO(get_logger(), "Send mode command to /mode_command topic");
    }

private:
    void initializeDefaultTargets() {
        // Initialize with defaults first
        setDefaultTargetValues();
        
        // Try to load task file and compute targets from initial state
        std::string taskFile, libraryFolder, urdfFile;
        
        if (get_parameter("task_file", taskFile) &&
            get_parameter("urdf_file", urdfFile)) {
            get_parameter_or("library_folder", libraryFolder, taskFile.substr(0, taskFile.rfind('/')) + "/auto_generated");
            
            try {
                // Create interface to access Pinocchio and initial state
                WholeBodyInterface interface(taskFile, libraryFolder, urdfFile);
                const auto& pinocchioInterface = interface.getPinocchioInterface();
                const auto& model = pinocchioInterface.getModel();
                auto data = pinocchioInterface.getData();
                
                vector_t initState = interface.getInitialState();
                
                // Forward kinematics
                pinocchio::forwardKinematics(model, data, initState);
                pinocchio::updateFramePlacements(model, data);
                
                const auto& modelInfo = interface.getModelInfo();
                
                // Left arm end-effector
                auto leftEEFrameId = model.getFrameId(modelInfo.eeFrame);
                if (leftEEFrameId < model.nframes) {
                    const auto& leftPose = data.oMf[leftEEFrameId];
                    leftTarget_.position.x = leftPose.translation().x();
                    leftTarget_.position.y = leftPose.translation().y();
                    leftTarget_.position.z = leftPose.translation().z();
                    Eigen::Quaterniond q(leftPose.rotation());
                    leftTarget_.orientation.w = q.w();
                    leftTarget_.orientation.x = q.x();
                    leftTarget_.orientation.y = q.y();
                    leftTarget_.orientation.z = q.z();
                    RCLCPP_INFO(get_logger(), "Left EE from FK: [%.3f, %.3f, %.3f]",
                        leftPose.translation().x(), leftPose.translation().y(), leftPose.translation().z());
                }
                
                // Right arm end-effector
                auto rightEEFrameId = model.getFrameId(modelInfo.eeFrame1);
                if (rightEEFrameId < model.nframes) {
                    const auto& rightPose = data.oMf[rightEEFrameId];
                    rightTarget_.position.x = rightPose.translation().x();
                    rightTarget_.position.y = rightPose.translation().y();
                    rightTarget_.position.z = rightPose.translation().z();
                    Eigen::Quaterniond q(rightPose.rotation());
                    rightTarget_.orientation.w = q.w();
                    rightTarget_.orientation.x = q.x();
                    rightTarget_.orientation.y = q.y();
                    rightTarget_.orientation.z = q.z();
                    RCLCPP_INFO(get_logger(), "Right EE from FK: [%.3f, %.3f, %.3f]",
                        rightPose.translation().x(), rightPose.translation().y(), rightPose.translation().z());
                }
                
                // Body frame
                auto bodyFrameId = model.getFrameId(modelInfo.bodyFrame);
                if (bodyFrameId < model.nframes) {
                    const auto& bodyPose = data.oMf[bodyFrameId];
                    bodyTarget_.position.x = bodyPose.translation().x();
                    bodyTarget_.position.y = bodyPose.translation().y();
                    bodyTarget_.position.z = bodyPose.translation().z();
                    Eigen::Quaterniond q(bodyPose.rotation());
                    bodyTarget_.orientation.w = q.w();
                    bodyTarget_.orientation.x = q.x();
                    bodyTarget_.orientation.y = q.y();
                    bodyTarget_.orientation.z = q.z();
                    RCLCPP_INFO(get_logger(), "Body from FK: [%.3f, %.3f, %.3f]",
                        bodyPose.translation().x(), bodyPose.translation().y(), bodyPose.translation().z());
                }
                
                RCLCPP_INFO(get_logger(), "Initialized targets from initial state using FK");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Failed to compute FK: %s. Using default targets.", e.what());
            }
        } else {
            RCLCPP_INFO(get_logger(), "task_file/urdf_file not provided, using default targets");
        }
        
        // Get initial mode from parameter
        std::string initialModeStr;
        get_parameter_or<std::string>("initial_mode", initialModeStr, "HM_DUAL_RELATIVE_MOBILE_BASE");
        currentMode_ = stringToMode(initialModeStr);  // an unknown name throws: the node does not start
        RCLCPP_INFO(get_logger(), "Initial mode: %s", initialModeStr.c_str());
    }
    
    void setDefaultTargetValues() {
        // Default left arm target
        leftTarget_.position.x = 0.4;
        leftTarget_.position.y = 0.3;
        leftTarget_.position.z = 1.0;
        leftTarget_.orientation.w = 1.0;
        leftTarget_.orientation.x = 0.0;
        leftTarget_.orientation.y = 0.0;
        leftTarget_.orientation.z = 0.0;
        
        // Default right arm target
        rightTarget_.position.x = 0.4;
        rightTarget_.position.y = -0.3;
        rightTarget_.position.z = 1.0;
        rightTarget_.orientation.w = 1.0;
        rightTarget_.orientation.x = 0.0;
        rightTarget_.orientation.y = 0.0;
        rightTarget_.orientation.z = 0.0;
        
        // Default body target
        bodyTarget_.position.x = 0.0;
        bodyTarget_.position.y = 0.0;
        bodyTarget_.position.z = 1.0;
        bodyTarget_.orientation.w = 1.0;
        bodyTarget_.orientation.x = 0.0;
        bodyTarget_.orientation.y = 0.0;
        bodyTarget_.orientation.z = 0.0;
    }
    
    void createInteractiveMarkers() {
        // Left arm marker
        createPoseMarker("left_arm", leftTarget_, 0.0, 1.0, 0.0);  // green
        
        // Right arm marker  
        createPoseMarker("right_arm", rightTarget_, 0.0, 0.0, 1.0);  // blue
        
        // Body marker
        createPoseMarker("body", bodyTarget_, 1.0, 0.5, 0.0);  // orange
        
        markerServer_->applyChanges();
    }
    
    void createPoseMarker(const std::string& name, 
                          const geometry_msgs::msg::Pose& pose,
                          float r, float g, float b) {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header.frame_id = "world";
        marker.name = name + "_target";
        marker.description = name + " target";
        marker.pose = pose;
        marker.scale = 0.15;
        
        // Visual
        visualization_msgs::msg::Marker visual;
        visual.type = visualization_msgs::msg::Marker::SPHERE;
        visual.scale.x = 0.08;
        visual.scale.y = 0.08;
        visual.scale.z = 0.08;
        visual.color.r = r;
        visual.color.g = g;
        visual.color.b = b;
        visual.color.a = 0.8;
        
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(visual);
        marker.controls.push_back(control);
        
        // 6-DOF controls (translation and rotation)
        control = visualization_msgs::msg::InteractiveMarkerControl();
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
        
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
        
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
        
        markerServer_->insert(marker,
            std::bind(&WholeBodyTargetNode::markerFeedback, this, 
                      std::placeholders::_1, name));
    }
    
    void markerFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        const std::string& name) {
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        HumanoidMode mode = HumanoidMode::decode(currentMode_);
        bool isCoupled = (mode.bimanualMode == BimanualMode::COUPLED);
        
        if (name == "left_arm") {
            geometry_msgs::msg::Pose oldLeft = leftTarget_;
            leftTarget_ = feedback->pose;
            
            // In COUPLED mode, update right arm to maintain relative pose
            if (isCoupled && couplingCaptured_) {
                updateCoupledRightArm();
            }
        } else if (name == "right_arm") {
            geometry_msgs::msg::Pose oldRight = rightTarget_;
            rightTarget_ = feedback->pose;
            
            // In COUPLED mode, update left arm to maintain relative pose
            if (isCoupled && couplingCaptured_) {
                updateCoupledLeftArm();
            }
        } else if (name == "body") {
            bodyTarget_ = feedback->pose;
        }
    }
    
    void updateCoupledRightArm() {
        // Right = Left * relativeTransform
        Eigen::Vector3d leftPos(leftTarget_.position.x, leftTarget_.position.y, leftTarget_.position.z);
        Eigen::Quaterniond leftQuat(leftTarget_.orientation.w, leftTarget_.orientation.x,
                                     leftTarget_.orientation.y, leftTarget_.orientation.z);
        
        // Apply relative transform (in left arm's frame)
        Eigen::Vector3d rightPos = leftPos + leftQuat * capturedRelPos_;
        Eigen::Quaterniond rightQuat = leftQuat * capturedRelQuat_;
        
        rightTarget_.position.x = rightPos.x();
        rightTarget_.position.y = rightPos.y();
        rightTarget_.position.z = rightPos.z();
        rightTarget_.orientation.w = rightQuat.w();
        rightTarget_.orientation.x = rightQuat.x();
        rightTarget_.orientation.y = rightQuat.y();
        rightTarget_.orientation.z = rightQuat.z();
        
        // Update marker visualization
        markerServer_->setPose("right_arm_target", rightTarget_);
        markerServer_->applyChanges();
    }
    
    void updateCoupledLeftArm() {
        // Left = Right * inverse(relativeTransform)
        Eigen::Vector3d rightPos(rightTarget_.position.x, rightTarget_.position.y, rightTarget_.position.z);
        Eigen::Quaterniond rightQuat(rightTarget_.orientation.w, rightTarget_.orientation.x,
                                      rightTarget_.orientation.y, rightTarget_.orientation.z);
        
        // Inverse relative transform
        Eigen::Quaterniond invRelQuat = capturedRelQuat_.inverse();
        Eigen::Vector3d invRelPos = -(invRelQuat * capturedRelPos_);
        
        // Apply inverse transform
        Eigen::Quaterniond leftQuat = rightQuat * invRelQuat;
        Eigen::Vector3d leftPos = rightPos + rightQuat * invRelPos;
        
        leftTarget_.position.x = leftPos.x();
        leftTarget_.position.y = leftPos.y();
        leftTarget_.position.z = leftPos.z();
        leftTarget_.orientation.w = leftQuat.w();
        leftTarget_.orientation.x = leftQuat.x();
        leftTarget_.orientation.y = leftQuat.y();
        leftTarget_.orientation.z = leftQuat.z();
        
        // Update marker visualization
        markerServer_->setPose("left_arm_target", leftTarget_);
        markerServer_->applyChanges();
    }
    
    void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        leftTarget_ = msg->pose;
        updateMarker("left_arm", leftTarget_);
    }
    
    void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        rightTarget_ = msg->pose;
        updateMarker("right_arm", rightTarget_);
    }
    
    void bodyTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        bodyTarget_ = msg->pose;
        updateMarker("body", bodyTarget_);
    }
    
    void updateMarker(const std::string& name, const geometry_msgs::msg::Pose& pose) {
        markerServer_->setPose(name + "_target", pose);
        markerServer_->applyChanges();
    }
    
    void modeCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        try {
            size_t newMode = stringToMode(msg->data);
            HumanoidMode oldDecoded = HumanoidMode::decode(currentMode_);
            HumanoidMode newDecoded = HumanoidMode::decode(newMode);
            
            // Check if switching to COUPLED mode
            if (newDecoded.bimanualMode == BimanualMode::COUPLED && 
                oldDecoded.bimanualMode != BimanualMode::COUPLED) {
                // Capture current relative pose
                captureCouplingFromTargets();
            }
            
            currentMode_ = newMode;
            
            RCLCPP_INFO(get_logger(), "Mode changed to: %s (encoded: %zu)", msg->data.c_str(), newMode);
            RCLCPP_INFO(get_logger(), "  Body: %s, Left: %s, Right: %s, Bimanual: %s, Base: %s",
                bodyModeToString(newDecoded.bodyMode).c_str(),
                newDecoded.leftArmActive ? "active" : "inactive",
                newDecoded.rightArmActive ? "active" : "inactive",
                bimanualModeToString(newDecoded.bimanualMode).c_str(),
                newDecoded.baseLocked ? "LOCKED" : "mobile");
            
            // Publish mode schedule immediately
            publishModeSchedule();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Invalid mode: %s. Error: %s", 
                msg->data.c_str(), e.what());
        }
    }
    
    void captureCouplingFromTargets() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Get left arm pose
        Eigen::Vector3d leftPos(leftTarget_.position.x, leftTarget_.position.y, leftTarget_.position.z);
        Eigen::Quaterniond leftQuat(leftTarget_.orientation.w, leftTarget_.orientation.x,
                                     leftTarget_.orientation.y, leftTarget_.orientation.z);
        
        // Get right arm pose
        Eigen::Vector3d rightPos(rightTarget_.position.x, rightTarget_.position.y, rightTarget_.position.z);
        Eigen::Quaterniond rightQuat(rightTarget_.orientation.w, rightTarget_.orientation.x,
                                      rightTarget_.orientation.y, rightTarget_.orientation.z);
        
        // Compute relative position in left arm's local frame
        Eigen::Vector3d worldRelPos = rightPos - leftPos;
        capturedRelPos_ = leftQuat.inverse() * worldRelPos;
        
        // Compute relative orientation
        capturedRelQuat_ = leftQuat.inverse() * rightQuat;
        if (capturedRelQuat_.w() < 0) {
            capturedRelQuat_.coeffs() = -capturedRelQuat_.coeffs();
        }
        
        couplingCaptured_ = true;
        
        RCLCPP_INFO(get_logger(), "Captured coupling (in left arm's local frame):");
        RCLCPP_INFO(get_logger(), "  Relative position: [%.3f, %.3f, %.3f]",
            capturedRelPos_.x(), capturedRelPos_.y(), capturedRelPos_.z());
        RCLCPP_INFO(get_logger(), "  Relative orientation (w,x,y,z): [%.3f, %.3f, %.3f, %.3f]",
            capturedRelQuat_.w(), capturedRelQuat_.x(), capturedRelQuat_.y(), capturedRelQuat_.z());
    }
    
    void publishTargets() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Create 21-dimensional target state
        // [0:3] left pos, [3:7] left quat (x,y,z,w), [7:10] right pos, [10:14] right quat (x,y,z,w),
        // [14:17] body pos, [17:21] body quat (x,y,z,w)
        // Quaternion order: [x, y, z, w] to match Eigen::Quaternion coeffs() order
        vector_t targetState(21);
        
        // Left arm
        targetState(0) = leftTarget_.position.x;
        targetState(1) = leftTarget_.position.y;
        targetState(2) = leftTarget_.position.z;
        targetState(3) = leftTarget_.orientation.x;
        targetState(4) = leftTarget_.orientation.y;
        targetState(5) = leftTarget_.orientation.z;
        targetState(6) = leftTarget_.orientation.w;
        
        // Right arm
        targetState(7) = rightTarget_.position.x;
        targetState(8) = rightTarget_.position.y;
        targetState(9) = rightTarget_.position.z;
        targetState(10) = rightTarget_.orientation.x;
        targetState(11) = rightTarget_.orientation.y;
        targetState(12) = rightTarget_.orientation.z;
        targetState(13) = rightTarget_.orientation.w;
        
        // Body
        targetState(14) = bodyTarget_.position.x;
        targetState(15) = bodyTarget_.position.y;
        targetState(16) = bodyTarget_.position.z;
        targetState(17) = bodyTarget_.orientation.x;
        targetState(18) = bodyTarget_.orientation.y;
        targetState(19) = bodyTarget_.orientation.z;
        targetState(20) = bodyTarget_.orientation.w;
        
        // Create target trajectories
        double currentTime = now().seconds();
        TargetTrajectories targets;
        targets.timeTrajectory = {currentTime, currentTime + 1.0};
        targets.stateTrajectory = {targetState, targetState};
        
        // Input trajectory (zero inputs) - required by OCS2
        vector_t zeroInput = vector_t::Zero(20);  // 2 (base) + 18 (arm) = 20 inputs
        targets.inputTrajectory = {zeroInput, zeroInput};
        
        // Convert to ROS message and publish
        auto msg = ros_msg_conversions::createTargetTrajectoriesMsg(targets);
        targetPub_->publish(msg);
    }
    
    void publishModeSchedule() {
        ocs2_msgs::msg::ModeSchedule msg;
        msg.event_times = {};  // No events
        // Mode encoding fits in 0-63 range (max 6 bits used)
        msg.mode_sequence.push_back(static_cast<int8_t>(currentMode_));
        modePub_->publish(msg);
    }
    
    // Publishers
    rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetPub_;
    rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr modePub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leftTargetSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rightTargetSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bodyTargetSub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modeCommandSub_;
    
    // Interactive markers
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> markerServer_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr publishTimer_;
    
    // State
    std::mutex mutex_;
    geometry_msgs::msg::Pose leftTarget_;
    geometry_msgs::msg::Pose rightTarget_;
    geometry_msgs::msg::Pose bodyTarget_;
    size_t currentMode_;
    
    // Captured coupling parameters (in left arm's local frame)
    bool couplingCaptured_ = false;
    Eigen::Vector3d capturedRelPos_;        // Right pos - Left pos in left arm's frame
    Eigen::Quaterniond capturedRelQuat_;    // Relative orientation
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WholeBodyTargetNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

