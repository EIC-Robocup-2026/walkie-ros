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
#include <my_robot_interfaces/action/go_to_pose_quaternion.hpp>
#include <my_robot_interfaces/action/set_joint_position.hpp>
#include <control_msgs/action/gripper_command.hpp>
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
using GoToPoseQuaternion = my_robot_interfaces::action::GoToPoseQuaternion;
using GoalHandleGoToPoseQuaternion = rclcpp_action::ServerGoalHandle<GoToPoseQuaternion>;
using SetJointPosition = my_robot_interfaces::action::SetJointPosition;
using GoalHandleSetJointPosition = rclcpp_action::ServerGoalHandle<SetJointPosition>;
using GripperCommand = control_msgs::action::GripperCommand;

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

        // 2. Initialize Groups (both arms and grippers)
        left_arm_      = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        right_arm_     = std::make_shared<MoveGroupInterface>(node_, "right_arm");
        left_gripper_  = std::make_shared<MoveGroupInterface>(node_, "left_gripper");
        right_gripper_ = std::make_shared<MoveGroupInterface>(node_, "right_gripper");
        left_arm_lift_  = std::make_shared<MoveGroupInterface>(node_, "left_arm_lift");
        right_arm_lift_ = std::make_shared<MoveGroupInterface>(node_, "right_arm_lift");
        both_arms_      = std::make_shared<MoveGroupInterface>(node_, "both_arms");
        both_arms_lift_ = std::make_shared<MoveGroupInterface>(node_, "both_arms_lift");

        // Set scaling  (1.0 = full speed; lower for safety on real hardware)
        left_arm_->setMaxVelocityScalingFactor(1.0);
        left_arm_->setMaxAccelerationScalingFactor(1.0);
        right_arm_->setMaxVelocityScalingFactor(1.0);
        right_arm_->setMaxAccelerationScalingFactor(1.0);
        left_arm_lift_->setMaxVelocityScalingFactor(1.0);
        left_arm_lift_->setMaxAccelerationScalingFactor(1.0);
        right_arm_lift_->setMaxVelocityScalingFactor(1.0);
        right_arm_lift_->setMaxAccelerationScalingFactor(1.0);
        both_arms_->setMaxVelocityScalingFactor(1.0);
        both_arms_->setMaxAccelerationScalingFactor(1.0);
        both_arms_lift_->setMaxVelocityScalingFactor(1.0);
        both_arms_lift_->setMaxAccelerationScalingFactor(1.0);

        left_arm_->setPlanningTime(1.0);
        right_arm_->setPlanningTime(1.0);
        left_gripper_->setPlanningTime(1.0);
        right_gripper_->setPlanningTime(1.0);
        left_arm_lift_->setPlanningTime(1.0);
        right_arm_lift_->setPlanningTime(1.0);
        both_arms_->setPlanningTime(1.0);
        both_arms_lift_->setPlanningTime(1.0);

        left_arm_->setNumPlanningAttempts(10000);
        right_arm_->setNumPlanningAttempts(10000);
        left_gripper_->setNumPlanningAttempts(100);
        right_gripper_->setNumPlanningAttempts(100);
        left_arm_lift_->setNumPlanningAttempts(10000);
        right_arm_lift_->setNumPlanningAttempts(10000);
        both_arms_->setNumPlanningAttempts(10000);
        both_arms_lift_->setNumPlanningAttempts(10000);

        // 3. Gripper Action Clients (GripperActionController)
        left_gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            node_, "/left_gripper_controller/gripper_action");
        right_gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            node_, "/right_gripper_controller/gripper_action");

        // 4. Action Servers
        go_to_pose_server_ = rclcpp_action::create_server<GoToPose>(
            node_, "go_to_pose",
            std::bind(&Commander::handle_goal_pose, this, _1, _2),
            std::bind(&Commander::handle_cancel_pose, this, _1),
            std::bind(&Commander::handle_accepted_pose, this, _1));

        go_to_pose_relative_server_ = rclcpp_action::create_server<GoToPoseRelative>(
            node_, "go_to_pose_relative",
            std::bind(&Commander::handle_goal_pose_relative, this, _1, _2),
            std::bind(&Commander::handle_cancel_pose_relative, this, _1),
            std::bind(&Commander::handle_accepted_pose_relative, this, _1));

        control_gripper_server_ = rclcpp_action::create_server<ControlGripper>(
            node_, "control_gripper",
            std::bind(&Commander::handle_goal_gripper, this, _1, _2),
            std::bind(&Commander::handle_cancel_gripper, this, _1),
            std::bind(&Commander::handle_accepted_gripper, this, _1));

        go_to_home_server_ = rclcpp_action::create_server<GoToHome>(
            node_, "go_to_home",
            std::bind(&Commander::handle_goal_home, this, _1, _2),
            std::bind(&Commander::handle_cancel_home, this, _1),
            std::bind(&Commander::handle_accepted_home, this, _1));

        go_to_pose_quat_server_ = rclcpp_action::create_server<GoToPoseQuaternion>(
            node_, "go_to_pose_quat",
            std::bind(&Commander::handle_goal_pose_quat, this, _1, _2),
            std::bind(&Commander::handle_cancel_pose_quat, this, _1),
            std::bind(&Commander::handle_accepted_pose_quat, this, _1));

        set_joint_position_server_ = rclcpp_action::create_server<SetJointPosition>(
            node_, "set_joint_position",
            std::bind(&Commander::handle_goal_set_joint_position, this, _1, _2),
            std::bind(&Commander::handle_cancel_set_joint_position, this, _1),
            std::bind(&Commander::handle_accepted_set_joint_position, this, _1));

        RCLCPP_INFO(node_->get_logger(), "Bimanual Commander Node Initialized");
    }

    // --- Helper: group selection ---
    std::shared_ptr<MoveGroupInterface> getGroup(const std::string & group_name)
    {
        if (group_name == "left_arm")       return left_arm_;
        if (group_name == "right_arm")      return right_arm_;
        if (group_name == "left_gripper")   return left_gripper_;
        if (group_name == "right_gripper")  return right_gripper_;
        if (group_name == "left_arm_lift")  return left_arm_lift_;
        if (group_name == "right_arm_lift") return right_arm_lift_;
        if (group_name == "both_arms")      return both_arms_;
        if (group_name == "both_arms_lift") return both_arms_lift_;
        return nullptr;
    }

    // --- Helper: arm group only (excludes grippers) ---
    std::shared_ptr<MoveGroupInterface> getArmGroup(const std::string & group_name)
    {
        if (group_name == "left_arm")       return left_arm_;
        if (group_name == "right_arm")      return right_arm_;
        if (group_name == "left_arm_lift")  return left_arm_lift_;
        if (group_name == "right_arm_lift") return right_arm_lift_;
        if (group_name == "both_arms")      return both_arms_;
        if (group_name == "both_arms_lift") return both_arms_lift_;
        return nullptr;
    }

    // --- Helper: gripper client selection ---
    rclcpp_action::Client<GripperCommand>::SharedPtr getGripperClient(const std::string & group_name)
    {
        if (group_name == "left_gripper")  return left_gripper_client_;
        if (group_name == "right_gripper") return right_gripper_client_;
        return nullptr;
    }

    bool executeTarget(std::shared_ptr<MoveGroupInterface> group,
                       const geometry_msgs::msg::Pose & target, bool cartesian)
    {
        if (!group) return false;

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.01);

        if (cartesian) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target};
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);

            if (fraction > 0.9) {
                robot_trajectory::RobotTrajectory rt(
                    group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(trajectory);
                }
                return (group->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Cartesian path failed (%.1f%%)", fraction * 100.0);
                return false;
            }
        } else {
            group->setPoseTarget(target);
            MoveGroupInterface::Plan plan;
            if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                return (group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
            return false;
        }
    }

    // --- Gripper control via GripperActionController ---
    bool controlGripper(const std::string & group_name, double position)
    {
        auto client = getGripperClient(group_name);
        if (!client) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown gripper group: %s", group_name.c_str());
            return false;
        }

        if (!client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available for %s",
                         group_name.c_str());
            return false;
        }

        GripperCommand::Goal goal;
        goal.command.position   = position;
        goal.command.max_effort = 50.0;

        RCLCPP_INFO(node_->get_logger(), "Setting %s to position %.3f", group_name.c_str(), position);

        auto send_opts = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        auto future = client->async_send_goal(goal, send_opts);

        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal send timed out");
            return false;
        }

        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
            return false;
        }

        auto result_future = client->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto result = result_future.get();
            return (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
        return false;
    }

    // ========== GoToPose ==========

    rclcpp_action::GoalResponse handle_goal_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPose::Goal> goal)
    {
        (void)uuid;
        if (!getGroup(goal->group_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid group: %s", goal->group_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received cancel GoToPose");
        (void)goal_handle;
        left_arm_->stop();
        right_arm_->stop();
        left_arm_lift_->stop();
        right_arm_lift_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_pose, this, goal_handle)}.detach();
    }

    void execute_go_to_pose(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        auto result = std::make_shared<GoToPose::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false; result->status = "Invalid Group";
            goal_handle->abort(result); return;
        }

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.01);

        tf2::Quaternion q;
        q.setRPY(goal->roll, goal->pitch, goal->yaw);
        q.normalize();

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = goal->x;
        target_pose.position.y = goal->y;
        target_pose.position.z = goal->z;
        target_pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(node_->get_logger(), "[%s] Planning to X:%.2f Y:%.2f Z:%.2f",
                    goal->group_name.c_str(), goal->x, goal->y, goal->z);

        bool plan_success = false;
        MoveGroupInterface::Plan plan;

        if (goal->cartesian_path) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction > 0.9) {
                robot_trajectory::RobotTrajectory rt(
                    group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(plan.trajectory);
                    plan_success = true;
                }
            }
        } else {
            group->setPoseTarget(target_pose);
            plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        if (plan_success) {
            auto err = group->execute(plan);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true; result->status = "Success";
                goal_handle->succeed(result);
            } else {
                result->success = false; result->status = "Execution Failed";
                goal_handle->abort(result);
            }
        } else {
            result->success = false; result->status = "Planning Failed";
            goal_handle->abort(result);
        }
    }

    // ========== GoToPoseRelative ==========

    rclcpp_action::GoalResponse handle_goal_pose_relative(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPoseRelative::Goal> goal)
    {
        (void)uuid;
        if (!getGroup(goal->group_name)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose_relative(
        const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received cancel GoToPoseRelative");
        (void)goal_handle;
        left_arm_->stop();
        right_arm_->stop();
        left_arm_lift_->stop();
        right_arm_lift_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose_relative(const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_pose_relative, this, goal_handle)}.detach();
    }

    void execute_go_to_pose_relative(const std::shared_ptr<GoalHandleGoToPoseRelative> goal_handle)
    {
        auto result = std::make_shared<GoToPoseRelative::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false; result->status = "Invalid Group";
            goal_handle->abort(result); return;
        }

        std::string target_frame = "base_footprint";
        std::string source_frame = group->getEndEffectorLink();
        geometry_msgs::msg::Pose current_pose;

        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
            current_pose.position.x = t.transform.translation.x;
            current_pose.position.y = t.transform.translation.y;
            current_pose.position.z = t.transform.translation.z;
            current_pose.orientation = t.transform.rotation;
            RCLCPP_INFO(node_->get_logger(), "[%s] TF Current -> X:%.2f Y:%.2f Z:%.2f",
                        goal->group_name.c_str(),
                        current_pose.position.x, current_pose.position.y, current_pose.position.z);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Lookup Failed: %s", ex.what());
            result->success = false; result->status = "TF Lookup Failed";
            goal_handle->abort(result); return;
        }

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame(target_frame);
        group->setGoalTolerance(0.01);

        tf2::Quaternion q_current;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::Quaternion q_increment;
        q_increment.setRPY(goal->roll, goal->pitch, goal->yaw);
        tf2::Quaternion q_final = q_current * q_increment;
        q_final.normalize();

        tf2::Matrix3x3 mat_rot(q_current);
        tf2::Vector3 v_offset(goal->x, goal->y, goal->z);
        tf2::Vector3 v_offset_rotated = mat_rot * v_offset;

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + v_offset_rotated.x();
        target_pose.position.y = current_pose.position.y + v_offset_rotated.y();
        target_pose.position.z = current_pose.position.z + v_offset_rotated.z();
        target_pose.orientation = tf2::toMsg(q_final);

        RCLCPP_INFO(node_->get_logger(), "[%s] Relative Target -> X:%.2f Y:%.2f Z:%.2f",
                    goal->group_name.c_str(),
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        bool plan_success = false;
        bool using_cartesian_result = false;
        MoveGroupInterface::Plan plan;
        moveit_msgs::msg::RobotTrajectory cartesian_traj;

        if (goal->cartesian_path) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
            double fraction = group->computeCartesianPath(waypoints, 0.01, cartesian_traj);
            RCLCPP_INFO(node_->get_logger(), "Cartesian fraction: %.2f", fraction);
            if (fraction >= 0.90) {
                robot_trajectory::RobotTrajectory rt(
                    group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), cartesian_traj);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(cartesian_traj);
                    plan_success = true;
                    using_cartesian_result = true;
                }
            }
        }

        if (!plan_success) {
            if (goal->cartesian_path)
                RCLCPP_WARN(node_->get_logger(), "Cartesian failed. Falling back to PTP.");
            group->setPoseTarget(target_pose);
            plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            using_cartesian_result = false;
        }

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        moveit::core::MoveItErrorCode err = moveit::core::MoveItErrorCode::FAILURE;
        if (plan_success) {
            err = using_cartesian_result ? group->execute(cartesian_traj) : group->execute(plan);
        }

        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true; result->status = "Success";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->status = plan_success ? "Execution Failed" : "Planning Failed";
            goal_handle->abort(result);
        }
    }

    // ========== GoToPoseQuaternion ==========

    rclcpp_action::GoalResponse handle_goal_pose_quat(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPoseQuaternion::Goal> goal)
    {
        (void)uuid;
        if (!getGroup(goal->group_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid group: %s", goal->group_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose_quat(
        const std::shared_ptr<GoalHandleGoToPoseQuaternion> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received cancel GoToPoseQuaternion");
        (void)goal_handle;
        left_arm_->stop();
        right_arm_->stop();
        left_arm_lift_->stop();
        right_arm_lift_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose_quat(const std::shared_ptr<GoalHandleGoToPoseQuaternion> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_pose_quat, this, goal_handle)}.detach();
    }

    void execute_go_to_pose_quat(const std::shared_ptr<GoalHandleGoToPoseQuaternion> goal_handle)
    {
        auto result = std::make_shared<GoToPoseQuaternion::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false; result->status = "Invalid Group";
            goal_handle->abort(result); return;
        }

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.01);

        tf2::Quaternion q(goal->qx, goal->qy, goal->qz, goal->qw);
        q.normalize();

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = goal->x;
        target_pose.position.y = goal->y;
        target_pose.position.z = goal->z;
        target_pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(node_->get_logger(),
                    "[%s] Planning to X:%.2f Y:%.2f Z:%.2f | Q:[%.2f,%.2f,%.2f,%.2f]",
                    goal->group_name.c_str(), goal->x, goal->y, goal->z,
                    q.x(), q.y(), q.z(), q.w());

        bool plan_success = false;
        MoveGroupInterface::Plan plan;

        if (goal->cartesian_path) {
            std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction > 0.9) {
                robot_trajectory::RobotTrajectory rt(
                    group->getCurrentState()->getRobotModel(), group->getName());
                rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                if (totg.computeTimeStamps(rt, 1.0, 1.0)) {
                    rt.getRobotTrajectoryMsg(plan.trajectory);
                    plan_success = true;
                }
            } else {
                RCLCPP_WARN(node_->get_logger(), "Cartesian path only %.1f%% complete",
                            fraction * 100.0);
            }
        } else {
            group->setPoseTarget(target_pose);
            plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        if (plan_success) {
            auto err = group->execute(plan);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true; result->status = "Success";
                goal_handle->succeed(result);
            } else {
                result->success = false; result->status = "Execution Failed";
                goal_handle->abort(result);
            }
        } else {
            result->success = false; result->status = "Planning Failed";
            goal_handle->abort(result);
        }
    }

    // ========== ControlGripper ==========

    rclcpp_action::GoalResponse handle_goal_gripper(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlGripper::Goal> goal)
    {
        (void)uuid;
        if (!getGripperClient(goal->group_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid gripper group: %s", goal->group_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle) {
        std::thread{std::bind(&Commander::execute_control_gripper, this, goal_handle)}.detach();
    }

    void execute_control_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        auto result = std::make_shared<ControlGripper::Result>();
        const auto goal = goal_handle->get_goal();

        bool success = controlGripper(goal->group_name, goal->position);

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        if (success) {
            result->success = true; result->status = "Success";
            goal_handle->succeed(result);
        } else {
            result->success = false; result->status = "Failed";
            goal_handle->abort(result);
        }
    }

    // ========== GoToHome ==========

    rclcpp_action::GoalResponse handle_goal_home(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToHome::Goal> goal)
    {
        (void)uuid;
        if (!getArmGroup(goal->group_name)) {
            RCLCPP_ERROR(node_->get_logger(), "GoToHome does not support group: %s",
                         goal->group_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_home(
        const std::shared_ptr<GoalHandleGoToHome> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received cancel GoToHome");
        (void)goal_handle;
        left_arm_->stop();
        right_arm_->stop();
        left_arm_lift_->stop();
        right_arm_lift_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_home(const std::shared_ptr<GoalHandleGoToHome> goal_handle) {
        std::thread{std::bind(&Commander::execute_go_to_home, this, goal_handle)}.detach();
    }

    void execute_go_to_home(const std::shared_ptr<GoalHandleGoToHome> goal_handle)
    {
        auto result = std::make_shared<GoToHome::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            goal_handle->abort(result); return;
        }

        RCLCPP_INFO(node_->get_logger(), "Executing GoToHome for %s", goal->group_name.c_str());

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.01);

        std::vector<double> home_joints(group->getVariableCount(), 0.0);
        group->setJointValueTarget(home_joints);

        MoveGroupInterface::Plan plan;
        bool plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result); return;
        }

        if (plan_success) {
            auto err = group->execute(plan);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true;
                goal_handle->succeed(result);
            } else {
                result->success = false;
                goal_handle->abort(result);
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan path to Home for %s",
                         goal->group_name.c_str());
            result->success = false;
            goal_handle->abort(result);
        }
    }

    // ========== SetJointPosition ==========

    rclcpp_action::GoalResponse handle_goal_set_joint_position(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SetJointPosition::Goal> goal)
    {
        (void)uuid;
        auto group = getGroup(goal->group_name);
        if (!group) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid group: %s", goal->group_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        size_t expected = group->getVariableCount();
        if (goal->joint_positions.size() != expected) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Joint count mismatch for %s: expected %zu, got %zu",
                         goal->group_name.c_str(), expected, goal->joint_positions.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_set_joint_position(
        const std::shared_ptr<GoalHandleSetJointPosition> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received cancel SetJointPosition");
        (void)goal_handle;
        left_arm_->stop();
        right_arm_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_set_joint_position(
        const std::shared_ptr<GoalHandleSetJointPosition> goal_handle)
    {
        std::thread{std::bind(&Commander::execute_set_joint_position, this, goal_handle)}.detach();
    }

    void execute_set_joint_position(const std::shared_ptr<GoalHandleSetJointPosition> goal_handle)
    {
        auto result = std::make_shared<SetJointPosition::Result>();
        const auto goal = goal_handle->get_goal();
        auto group = getGroup(goal->group_name);

        if (!group) {
            result->success = false; result->status = "Invalid Group";
            goal_handle->abort(result); return;
        }

        group->setStartStateToCurrentState();
        group->setGoalTolerance(0.01);

        RCLCPP_INFO(node_->get_logger(), "SetJointPosition for %s:", goal->group_name.c_str());
        for (size_t i = 0; i < goal->joint_positions.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %.3f rad", i, goal->joint_positions[i]);
        }

        group->setJointValueTarget(goal->joint_positions);

        MoveGroupInterface::Plan plan;
        bool plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        if (plan_success) {
            auto err = group->execute(plan);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                result->success = true; result->status = "Success";
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "SetJointPosition succeeded for %s",
                            goal->group_name.c_str());
            } else {
                result->success = false; result->status = "Execution Failed";
                goal_handle->abort(result);
            }
        } else {
            result->success = false; result->status = "Planning Failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(node_->get_logger(), "SetJointPosition planning failed for %s",
                         goal->group_name.c_str());
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;
    std::shared_ptr<MoveGroupInterface> left_gripper_;
    std::shared_ptr<MoveGroupInterface> right_gripper_;
    std::shared_ptr<MoveGroupInterface> left_arm_lift_;
    std::shared_ptr<MoveGroupInterface> right_arm_lift_;
    std::shared_ptr<MoveGroupInterface> both_arms_;
    std::shared_ptr<MoveGroupInterface> both_arms_lift_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_client_;

    rclcpp_action::Server<GoToPose>::SharedPtr go_to_pose_server_;
    rclcpp_action::Server<GoToPoseRelative>::SharedPtr go_to_pose_relative_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_server_;
    rclcpp_action::Server<GoToHome>::SharedPtr go_to_home_server_;
    rclcpp_action::Server<GoToPoseQuaternion>::SharedPtr go_to_pose_quat_server_;
    rclcpp_action::Server<SetJointPosition>::SharedPtr set_joint_position_server_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    auto node = std::make_shared<rclcpp::Node>("bimanual_commander", node_options);
    auto commander = std::make_shared<Commander>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
