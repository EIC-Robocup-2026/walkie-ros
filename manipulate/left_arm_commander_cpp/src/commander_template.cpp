#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <my_robot_interfaces/action/go_to_pose.hpp>
#include <my_robot_interfaces/action/go_to_pose_relative.hpp>
#include <my_robot_interfaces/action/control_gripper.hpp>
#include <my_robot_interfaces/action/go_to_home.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GoToPose = my_robot_interfaces::action::GoToPose;
using GoToPoseRelative = my_robot_interfaces::action::GoToPoseRelative;
using ControlGripper = my_robot_interfaces::action::ControlGripper;
using GoToHome = my_robot_interfaces::action::GoToHome;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using GoalHandleGoToPoseRelative = rclcpp_action::ServerGoalHandle<GoToPoseRelative>;
using GoalHandleControlGripper = rclcpp_action::ServerGoalHandle<ControlGripper>;
using GoalHandleGoToHome = rclcpp_action::ServerGoalHandle<GoToHome>;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;

        // 1. Parallel Callbacks
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 2. Initialize Groups (LEFT ONLY)
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        left_gripper_ = std::make_shared<MoveGroupInterface>(node_, "left_hand");


        // Set scaling
        left_arm_->setMaxVelocityScalingFactor(1.0);
        left_arm_->setMaxAccelerationScalingFactor(1.0);

        left_arm_->setPlanningTime(1.0); 
        left_gripper_->setPlanningTime(1.0);

        // Set Planning Attempts
        left_arm_->setNumPlanningAttempts(10000);     // <--- Added
        left_gripper_->setNumPlanningAttempts(100); // <--- Added
        
        // Initialize Gripper Action Client (LEFT ONLY)
        left_gripper_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, "/left_hand_controller/follow_joint_trajectory");

        // 3. Action Servers
        go_to_pose_server_ = rclcpp_action::create_server<GoToPose>(
            node_,
            "go_to_pose",
            std::bind(&Commander::handle_goal_pose, this, _1, _2),
            std::bind(&Commander::handle_cancel_pose, this, _1),
            std::bind(&Commander::handle_accepted_pose, this, _1));

        go_to_pose_relative_server_ = rclcpp_action::create_server<GoToPoseRelative>(
            node_,
            "go_to_pose_relative",
            std::bind(&Commander::handle_goal_pose_relative, this, _1, _2),
            std::bind(&Commander::handle_cancel_pose_relative, this, _1),
            std::bind(&Commander::handle_accepted_pose_relative, this, _1));

        control_gripper_server_ = rclcpp_action::create_server<ControlGripper>(
            node_,
            "control_gripper",
            std::bind(&Commander::handle_goal_gripper, this, _1, _2),
            std::bind(&Commander::handle_cancel_gripper, this, _1),
            std::bind(&Commander::handle_accepted_gripper, this, _1));

        go_to_home_server_ = rclcpp_action::create_server<GoToHome>(
            node_,
            "go_to_home",
            std::bind(&Commander::handle_goal_home, this, _1, _2),
            std::bind(&Commander::handle_cancel_home, this, _1),
            std::bind(&Commander::handle_accepted_home, this, _1));
            
        RCLCPP_INFO(node_->get_logger(), "Commander Node Initialized (Left Arm Only)");
    }
    
    // --- Helper for group selection ---
    std::shared_ptr<MoveGroupInterface> getGroup(const std::string &group_name) {
        if (group_name == "left_arm") return left_arm_;
        if (group_name == "left_gripper") return left_gripper_;
        return nullptr;
    }

    bool goToPoseTarget(std::shared_ptr<MoveGroupInterface> group, double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path)
    {
        if (!group) return false;

        // 1. Create Orientation
        tf2::Quaternion q; 
        q.setRPY(roll, pitch, yaw); 
        q.normalize();
        
        // 2. Create Target Pose
        geometry_msgs::msg::Pose target_pose; 
        target_pose.position.x = x; 
        target_pose.position.y = y; 
        target_pose.position.z = z; 
        target_pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(node_->get_logger(), "Target -> X:%.2f Y:%.2f Z:%.2f", x, y, z);
        
        return executeTarget(group, target_pose, cartesian_path);
    }

    
    
    bool controlGripper(std::shared_ptr<MoveGroupInterface> gripper, const std::string& state) 
    { 
        if (!gripper) return false;

        // Fixed to left side
        std::string controller_name = "left_hand_controller";
        std::string joint_name = "left_gripper_worm_gear_joint";
        std::string action_topic = "/" + controller_name + "/follow_joint_trajectory";

        // Determine target position
        double target_pos = 0.0;
        if (state.find("open") != std::string::npos) {
            target_pos = -10.0; // Adjust based on your URDF limits
        } else if (state.find("half") != std::string::npos) {
            target_pos = -5.0;
        } else if (state.find("close") != std::string::npos) {
            target_pos = 0.0;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown gripper state '%s', defaulting to 0.0", state.c_str());
        }

        RCLCPP_INFO(node_->get_logger(), "Setting left gripper to %.2f...", target_pos);

        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

        if (!left_gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper action server %s not available", action_topic.c_str());
             return false;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.header.frame_id = "base_footprint";
        goal.trajectory.joint_names = {joint_name};
        
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = {target_pos};
        p.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal.trajectory.points.push_back(p);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        auto goal_handle_future = left_gripper_client_->async_send_goal(goal, send_goal_options);
        
        if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal send timed out");
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
             return false;
        }

        auto result_future = left_gripper_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
             auto result = result_future.get();
             if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                 return true;
             } else {
                 return false;
             }
        }
        return false;
    }


    bool executeTarget(std::shared_ptr<MoveGroupInterface> group, const geometry_msgs::msg::Pose& target, bool cartesian)
    {
        if (!group) return false;
        
        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");

        // 2. But require it to end VERY close to the target (Fixes "Ghost Success")
        group->setGoalTolerance(0.01);

        if (cartesian) {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target);
            moveit_msgs::msg::RobotTrajectory trajectory;
            
            double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);
            
            if (fraction > 0.9) {
                robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
                
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(trajectory);
                }

                auto err = group->execute(trajectory);
                return (err == moveit::core::MoveItErrorCode::SUCCESS);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Cartesian path failed (%.2f%%)", fraction * 100.0);
                return false;
            }
        } 
        else {
            group->setPoseTarget(target);
            MoveGroupInterface::Plan plan;
            if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                group->execute(plan);
                return true;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
                return false;
            }
        }
    }


    // --- Action Headers ---

    rclcpp_action::GoalResponse handle_goal_pose(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal) {
        (void)uuid;
        if (!getGroup(goal->group_name)) {
             RCLCPP_ERROR(node_->get_logger(), "Invalid Group Name: %s", goal->group_name.c_str());
             return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel GoToPose");
        (void)goal_handle;
        
        // --- ADD THIS BLOCK ---
        if (left_arm_) {
            left_arm_->stop(); // Forces the blocking execute() to return immediately
        }
        // ----------------------

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_pose, this, goal_handle)}.detach();
    }

    void execute_go_to_pose(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        auto result = std::make_shared<GoToPose::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false;
            result->status = "Invalid Group";
            goal_handle->abort(result);
            return;
        }

        // 1. Setup
        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.01);

        // 2. Prepare Target Pose (Math logic moved here)
        tf2::Quaternion q;
        q.setRPY(goal->roll, goal->pitch, goal->yaw);
        q.normalize();
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = goal->x;
        target_pose.position.y = goal->y;
        target_pose.position.z = goal->z;
        target_pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(node_->get_logger(), "Planning to X:%.2f Y:%.2f Z:%.2f", goal->x, goal->y, goal->z);

        // 3. Plan
        bool plan_success = false;
        MoveGroupInterface::Plan plan;

        if (goal->cartesian_path) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);
            
            if (fraction > 0.9) {
                // Convert to full plan
                robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(plan.trajectory); // Note: .trajectory_ or .trajectory depending on version
                    plan_success = true;
                }
            }
        } else {
            group->setPoseTarget(target_pose);
            plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        // 4. CRITICAL: Check for Cancel BEFORE Executing
        if (goal_handle->is_canceling()) {
            RCLCPP_WARN(node_->get_logger(), "Goal Canceled during planning phase");
            result->success = false;
            result->status = "Canceled";
            goal_handle->canceled(result);
            return;
        }

        // 5. Execute
        if (plan_success) {
            auto err = group->execute(plan);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true;
                result->status = "Success";
                goal_handle->succeed(result);
            } else {
                result->success = false;
                result->status = "Execution Failed";
                goal_handle->abort(result);
            }
        } else {
            result->success = false;
            result->status = "Planning Failed";
            goal_handle->abort(result);
        }
    }

    // --- Pose Relative ---

    rclcpp_action::GoalResponse handle_goal_pose_relative(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPoseRelative::Goal> goal) {
        (void)uuid;
        if (!getGroup(goal->group_name)) {
             return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose_relative(const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel GoToPoseRelative");
        (void)goal_handle;

        // --- ADD THIS TO FORCE STOP ---
        if (left_arm_) {
            left_arm_->stop();
        }
        // ------------------------------

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose_relative(const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_pose_relative, this, goal_handle)}.detach();
    }

    void execute_go_to_pose_relative(const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle) {
        auto result = std::make_shared<GoToPoseRelative::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false;
            result->status = "Invalid Group";
            goal_handle->abort(result);
            return;
        }

        // ---------------------------------------------------------
        // 1. Get Current Pose from TF (More reliable than MoveIt)
        // ---------------------------------------------------------
        std::string target_frame = "base_footprint";
        std::string source_frame = group->getEndEffectorLink(); // Gets the tip link name automatically

        geometry_msgs::msg::Pose current_pose;

        try {
            // Look up the transform from EEF -> Base
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform(
                target_frame, 
                source_frame, 
                tf2::TimePointZero // Get the very latest transform
            );

            // Convert to Pose
            current_pose.position.x = t.transform.translation.x;
            current_pose.position.y = t.transform.translation.y;
            current_pose.position.z = t.transform.translation.z;
            current_pose.orientation = t.transform.rotation;
            
            RCLCPP_INFO(node_->get_logger(), "TF Current Pose -> X:%.2f Y:%.2f Z:%.2f", 
                current_pose.position.x, current_pose.position.y, current_pose.position.z);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Lookup Failed: %s", ex.what());
            result->success = false;
            result->status = "TF Lookup Failed";
            goal_handle->abort(result);
            return;
        }

        // Setup Group for Planning
        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame(target_frame); // Must match the frame we used for TF!
        group->setGoalTolerance(0.01);

        // ---------------------------------------------------------
        // 2. Do the Math (Relative -> Absolute)
        // ---------------------------------------------------------
        tf2::Quaternion q_current;
        tf2::fromMsg(current_pose.orientation, q_current);
        
        // Rotation relative to current tool orientation
        tf2::Quaternion q_increment;
        q_increment.setRPY(goal->roll, goal->pitch, goal->yaw);
        tf2::Quaternion q_final = q_current * q_increment; 
        q_final.normalize();

        // Translation relative to current tool orientation
        tf2::Matrix3x3 mat_rot(q_current);
        tf2::Vector3 v_offset(goal->x, goal->y, goal->z);
        tf2::Vector3 v_offset_rotated = mat_rot * v_offset;

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + v_offset_rotated.x();
        target_pose.position.y = current_pose.position.y + v_offset_rotated.y();
        target_pose.position.z = current_pose.position.z + v_offset_rotated.z();
        target_pose.orientation = tf2::toMsg(q_final);

        RCLCPP_INFO(node_->get_logger(), "Relative Target -> X:%.2f Y:%.2f Z:%.2f", 
            target_pose.position.x, target_pose.position.y, target_pose.position.z);

        // ---------------------------------------------------------
        // 3. Plan (Cartesian with Fallback)
        // ---------------------------------------------------------
        bool plan_success = false;
        bool using_cartesian_result = false;
        
        MoveGroupInterface::Plan plan;
        moveit_msgs::msg::RobotTrajectory cartesian_traj;

        // A. Try Cartesian Path First
        if (goal->cartesian_path) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
            
            double eef_step = 0.01;      // 1cm resolution
            
            double fraction = group->computeCartesianPath(waypoints, eef_step , cartesian_traj);
            RCLCPP_INFO(node_->get_logger(), "Cartesian path fraction: %.2f", fraction);

            if (fraction >= 0.90) {
                // Apply Time Parameterization
                robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), cartesian_traj);
                
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(cartesian_traj);
                    plan_success = true;
                    using_cartesian_result = true;
                }
            }
        }

        // B. Fallback to Standard PTP
        if (!plan_success) {
            if (goal->cartesian_path) {
                RCLCPP_WARN(node_->get_logger(), "Cartesian failed. Falling back to PTP.");
            }
            
            group->setPoseTarget(target_pose);
            moveit::core::MoveItErrorCode plan_result = group->plan(plan);
            
            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                plan_success = true;
                using_cartesian_result = false;
            }
        }

        // 4. Check Cancel
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->status = "Canceled";
            goal_handle->canceled(result);
            return;
        }

        // 5. Execute
        moveit::core::MoveItErrorCode err = moveit::core::MoveItErrorCode::FAILURE;
        
        if (plan_success) {
            if (using_cartesian_result) {
                err = group->execute(cartesian_traj);
            } else {
                err = group->execute(plan);
            }
        }

        // 6. Result
        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->status = "Success";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->status = plan_success ? "Execution Failed" : "Planning Failed";
            goal_handle->abort(result);
        }
    }

    // --- Gripper ---

    rclcpp_action::GoalResponse handle_goal_gripper(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlGripper::Goal> goal) {
        (void)uuid;
        if (!getGroup(goal->group_name)) {
             return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle) {
        std::thread{std::bind(&Commander::execute_control_gripper, this, goal_handle)}.detach();
    }

    // Updated: Now accepts specific 'target_pos' instead of string state
    bool controlGripper(std::shared_ptr<MoveGroupInterface> gripper, double target_pos) 
    { 
        if (!gripper) return false;

        std::string controller_name = "left_hand_controller";
        std::string joint_name = "left_gripper_worm_gear_joint";
        std::string action_topic = "/" + controller_name + "/follow_joint_trajectory";

        RCLCPP_INFO(node_->get_logger(), "Setting left gripper to %.2f...", target_pos);

        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

        if (!left_gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper action server %s not available", action_topic.c_str());
             return false;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.header.frame_id = "base_footprint";
        goal.trajectory.joint_names = {joint_name};
        
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = {target_pos}; // Use the specific value passed from the action
        p.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal.trajectory.points.push_back(p);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        auto goal_handle_future = left_gripper_client_->async_send_goal(goal, send_goal_options);
        
        if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal send timed out");
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
             return false;
        }

        auto result_future = left_gripper_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
             auto result = result_future.get();
             if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                 return true;
             }
        }
        return false;
    }

    void execute_control_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle) {
        auto result = std::make_shared<ControlGripper::Result>();
        const auto goal = goal_handle->get_goal();
        
        // Pass the raw position value from the Action Goal directly to the controller
        bool success = controlGripper(getGroup(goal->group_name), goal->position);

        if (goal_handle->is_canceling()) {
             result->success = false;
             result->status = "Canceled";
             goal_handle->canceled(result);
             return;
        }

        if (success) {
            result->success = true;
            result->status = "Success";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->status = "Failed";
            goal_handle->abort(result);
        }
    }

    // --- GoToHome Action ---

    rclcpp_action::GoalResponse handle_goal_home(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToHome::Goal> goal) {
        (void)uuid;
        // Accept only "left_arm"
        if (goal->group_name != "left_arm") {
             RCLCPP_ERROR(node_->get_logger(), "Invalid Group Name (Only 'left_arm' supported): %s", goal->group_name.c_str());
             return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_home(const std::shared_ptr<GoalHandleGoToHome> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel GoToHome");
        (void)goal_handle;

        // --- ADD THIS BLOCK ---
        if (left_arm_) {
            left_arm_->stop();
        }
        // ----------------------

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_home(const std::shared_ptr<GoalHandleGoToHome> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_home, this, goal_handle)}.detach();
    }

    void execute_go_to_home(const std::shared_ptr<GoalHandleGoToHome> goal_handle) {
        auto result = std::make_shared<GoToHome::Result>();
        
        RCLCPP_INFO(node_->get_logger(), "Executing GoToHome Action");

        if (!left_arm_) {
             goal_handle->abort(result);
             return;
        }

        // Setup
        left_arm_->setStartStateToCurrentState();
        left_arm_->setPoseReferenceFrame("base_footprint");
        left_arm_->setGoalTolerance(0.01); 

        // Set Target (All Zeros)
        std::vector<double> home_joints(left_arm_->getVariableCount(), 0.0);
        left_arm_->setJointValueTarget(home_joints);

        // 1. Plan
        MoveGroupInterface::Plan plan;
        bool plan_success = (left_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // 2. CRITICAL: Check for Cancel
        if (goal_handle->is_canceling()) {
            RCLCPP_WARN(node_->get_logger(), "Home Goal Canceled during planning");
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // 3. Execute
        if (plan_success) {
             auto err = left_arm_->execute(plan);
             if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                 result->success = true;
                 goal_handle->succeed(result);
             } else {
                 result->success = false;
                 goal_handle->abort(result);
             }
        } else {
             RCLCPP_ERROR(node_->get_logger(), "Failed to plan path to Home!");
             result->success = false;
             goal_handle->abort(result);
        }
    }

private:
   
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    // Define Groups (LEFT ONLY)
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> left_gripper_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Action Client for Gripper (LEFT ONLY)
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr left_gripper_client_;

    // Action Server Objects
    rclcpp_action::Server<GoToPose>::SharedPtr go_to_pose_server_;
    rclcpp_action::Server<GoToPoseRelative>::SharedPtr go_to_pose_relative_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_server_;
    rclcpp_action::Server<GoToHome>::SharedPtr go_to_home_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto node = std::make_shared<rclcpp::Node>("commander", node_options);
    auto commander = std::make_shared<Commander>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}