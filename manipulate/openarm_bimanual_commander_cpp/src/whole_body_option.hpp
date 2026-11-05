// Whole-body option of the bimanual commander.
//
// A caller selects this path per request by sending group_name whole_body_left
// or whole_body_right on GoToPose or GoToPoseQuaternion. Every other group name
// keeps the existing path and cannot command the base, because those groups have
// no base variables. See docs/guides/manipulation.md.
//
// A whole-body goal is frozen in map when it arrives. The current base pose is
// tried first through the matching *_arm_lift group, so a reachable target does
// not move the base. Otherwise base poses are sampled so that the arm's shoulder
// stands behind the hand, and the first ones with a usable IK solution are planned
// to with the whole-body group.
//
// Every refusal is an accepted goal followed by an abort with a reason in the
// status string, because a rejected goal gives the client no reason.

#pragma once

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

class WholeBodyOption
{
public:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using BasePose = std::array<double, 3>;  // x, y, yaw in map
    using CancelCheck = std::function<bool()>;
    // Holds a motion slot until it is destroyed. Null when the motion is refused. A ticket
    // can outlive this object in an execution thread, so it keeps the slot state alive.
    using MotionTicket = std::shared_ptr<void>;

    struct Outcome
    {
        bool success{false};
        bool canceled{false};
        std::string status;
    };

    static bool isWholeBodyName(const std::string & group_name)
    {
        return group_name == "whole_body_left" || group_name == "whole_body_right";
    }

    // GetEEPose and GetJointStates on a whole-body name answer for the arm and
    // lift, because clients ask right after a whole-body goal.
    static std::string queryGroupFor(const std::string & group_name)
    {
        if (group_name == "whole_body_left") return "left_arm_lift";
        if (group_name == "whole_body_right") return "right_arm_lift";
        return group_name;
    }

    WholeBodyOption(
        std::shared_ptr<rclcpp::Node> node,
        rclcpp::CallbackGroup::SharedPtr callback_group,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene,
        moveit::core::RobotModelConstPtr model,
        std::function<void()> on_motion_success)
    : node_(std::move(node)),
      callback_group_(std::move(callback_group)),
      tf_buffer_(std::move(tf_buffer)),
      get_planning_scene_(std::move(get_planning_scene)),
      model_(model),
      on_motion_success_(std::move(on_motion_success)),
      rng_(std::random_device{}())
    {
        base_mode_ = node_->declare_parameter<std::string>("base_mode", "fixed");
        planner_id_ = node_->declare_parameter<std::string>("whole_body_planner_id", "RRTConnect");
        planning_time_ = node_->declare_parameter<double>("whole_body_planning_time", 5.0);
        velocity_scaling_ = node_->declare_parameter<double>("whole_body_velocity_scaling", 1.0);
        acceleration_scaling_ = node_->declare_parameter<double>("whole_body_acceleration_scaling", 1.0);
        goal_joint_tolerance_ = node_->declare_parameter<double>("whole_body_goal_joint_tolerance", 0.001);
        execution_margin_ = node_->declare_parameter<double>("whole_body_execution_margin", 15.0);

        if (base_mode_ != "fixed") {
            state_validity_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
                "/check_state_validity", rclcpp::ServicesQoS(), callback_group_);
            clear_octomap_ = node_->create_client<std_srvs::srv::Empty>(
                "/clear_octomap", rclcpp::ServicesQoS(), callback_group_);
            rclcpp::SubscriptionOptions state_options;
            state_options.callback_group = callback_group_;
            joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", rclcpp::SensorDataQoS().keep_last(5),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                        joint_positions_[msg->name[i]] = msg->position[i];
                    }
                },
                state_options);
        }

        // Group interfaces exist only on a whole_body bring-up whose model has
        // the base variables: an interface on a missing group aborts the process,
        // and fixed and map bring-ups should not pay for unused clients.
        if (base_mode_ != "whole_body") return;

        compute_ik_ = node_->create_client<moveit_msgs::srv::GetPositionIK>(
            "/compute_ik", rclcpp::ServicesQoS(), callback_group_);
        base_server_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/base_controller/follow_joint_trajectory", callback_group_);

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;
        for (const std::string action : {"navigate_to_pose", "navigate_through_poses", "follow_path"}) {
            nav2_status_subs_.push_back(node_->create_subscription<action_msgs::msg::GoalStatusArray>(
                "/" + action + "/_action/status", rclcpp::QoS(10).reliable().transient_local(),
                [this, action](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
                    bool active = false;
                    for (const auto & status : msg->status_list) {
                        active = active || status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                                 status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                                 status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
                    }
                    std::lock_guard<std::mutex> lock(mutex_);
                    nav2_active_[action] = active;
                },
                options));
        }

        // The execution manager waits for its controllers one after another and looks at an
        // aborted goal only when that controller's turn comes. An arm that aborts 1 s into a
        // 20 s plan would leave the base driving behind the wait for base_controller, so
        // execute() hears about an abort directly.
        for (const auto & [controller, joint] : controllerJoints()) {
            (void)joint;
            controller_status_subs_.push_back(node_->create_subscription<action_msgs::msg::GoalStatusArray>(
                "/" + controller + "/follow_joint_trajectory/_action/status",
                rclcpp::QoS(10).reliable().transient_local(),
                [this, controller = controller](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    auto & goals = controller_goals_[controller];
                    goals.clear();
                    for (const auto & status : msg->status_list) {
                        const auto & uuid = status.goal_info.goal_id.uuid;
                        goals.insert(uuid);
                        // A status array keeps finished goals for a while, and another client
                        // can use a controller this plan does not. Only a goal that appeared
                        // after the execution began, on a controller the plan uses, is one
                        // this execution sent.
                        if (status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED &&
                            watched_controllers_.count(controller) > 0 &&
                            goals_before_execution_.count(uuid) == 0) {
                            aborted_controller_ = controller;
                        }
                    }
                },
                options));
        }

        // The base follower refuses goals without this and stops a running one when it ends,
        // so a dead commander cannot leave the base driving. While a whole-body motion runs
        // the message is true, and the follower stops move_group if it then ends, because a
        // motion without a base goal would otherwise run on unwatched. A wall timer, because
        // this reports on the process and not on the simulation.
        alive_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/whole_body/commander_alive", 10);
        alive_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                std_msgs::msg::Bool beat;
                {
                    std::lock_guard<std::mutex> lock(motion_->mutex);
                    beat.data = motion_->executing > 0;
                }
                alive_pub_->publish(beat);
            },
            callback_group_);

        const std::vector<std::array<std::string, 4>> specs = {
            {"whole_body_left", "left_arm_lift", "openarm_left_hand", "openarm_left_link1"},
            {"whole_body_right", "right_arm_lift", "openarm_right_hand", "openarm_right_link1"}};
        for (const auto & spec : specs) {
            if (!model->hasJointModelGroup(spec[0])) continue;
            const auto & variables = model->getJointModelGroup(spec[0])->getVariableNames();
            if (std::find(variables.begin(), variables.end(), "virtual_joint/x") == variables.end()) continue;
            Entry entry;
            entry.body = std::make_shared<MoveGroupInterface>(node_, spec[0]);
            // A separate interface from the commander's, so that the tolerance set
            // here cannot leak into later arm goals.
            entry.arm = std::make_shared<MoveGroupInterface>(node_, spec[1]);
            entry.tip = spec[2];
            entry.shoulder = spec[3];
            // The lift stays out of the seed: it is the slowest joint, so a goal near
            // its current height arrives soonest.
            entry.seed = entry.arm->getNamedTargetValues("home");
            entry.seed.erase("lift_joint");
            entries_[spec[0]] = entry;
        }
        applyPlanningParameters();
        RCLCPP_INFO(node_->get_logger(), "Whole-body option ready for %zu group(s)", entries_.size());
    }

    // Called from the commander's parameter callback. Returns false with a reason
    // on an invalid value, and ignores parameters this class does not own.
    bool validateParameter(const rclcpp::Parameter & p, std::string & reason) const
    {
        const auto & name = p.get_name();
        if ((name == "whole_body_velocity_scaling" || name == "whole_body_acceleration_scaling") &&
            !(p.as_double() > 0.0 && p.as_double() <= 1.0)) {
            reason = name + " must be in (0, 1]";
            return false;
        }
        if ((name == "whole_body_planning_time" || name == "whole_body_goal_joint_tolerance" ||
             name == "whole_body_execution_margin") && !(p.as_double() > 0.0)) {
            reason = name + " must be positive";
            return false;
        }
        if (name == "base_mode") {
            reason = "base_mode is fixed at startup, because the robot model depends on it";
            return false;
        }
        return true;
    }

    void applyParameter(const rclcpp::Parameter & p)
    {
        const auto & name = p.get_name();
        std::lock_guard<std::mutex> lock(mutex_);
        if (name == "whole_body_planner_id") planner_id_ = p.as_string();
        else if (name == "whole_body_planning_time") planning_time_ = p.as_double();
        else if (name == "whole_body_velocity_scaling") velocity_scaling_ = p.as_double();
        else if (name == "whole_body_acceleration_scaling") acceleration_scaling_ = p.as_double();
        else if (name == "whole_body_goal_joint_tolerance") goal_joint_tolerance_ = p.as_double();
        else if (name == "whole_body_execution_margin") execution_margin_ = p.as_double();
    }

    // Single-flight guard. An arm goal is refused only during whole-body motion, so
    // arm-on-arm behaviour is unchanged and this never refuses without a whole-body goal.
    MotionTicket beginArmMotion()
    {
        std::lock_guard<std::mutex> lock(motion_->mutex);
        if (motion_->whole_body_active) return nullptr;
        ++motion_->arm_active;
        return MotionTicket(motion_.get(), [motion = motion_](void *) {
            std::lock_guard<std::mutex> release(motion->mutex);
            --motion->arm_active;
        });
    }

    static std::string armRefusedStatus()
    {
        return "Refused: a whole-body motion is in progress";
    }

    // The start state of any goal. Under a fixed base this is the plain current state.
    void setStartToCurrent(const std::shared_ptr<MoveGroupInterface> & group)
    {
        group->setStartStateToCurrentState();
        if (base_mode_ == "fixed") return;
        clearVoxelsInsideRobot();
        const auto start = repairedStart(group->getName());
        if (!start.joint_state.name.empty()) group->setStartState(start);
    }

    // Plans and, unless plan_only, executes a whole-body goal.
    Outcome moveTo(
        const std::string & group_name, const std::string & frame_id, const geometry_msgs::msg::Pose & target,
        bool cartesian, bool plan_only, const CancelCheck & canceled)
    {
        const MotionTicket ticket = beginWholeBodyMotion();
        if (!ticket) return fail("Refused: another motion is in progress");

        std::string why;
        if (!passesGate(group_name, why)) return fail(why);
        const Entry & entry = entries_.at(group_name);

        if (cartesian) {
            // MoveIt returns code 1 with fraction 0 here, which reads as success.
            return fail("Cartesian paths are not supported on " + group_name + ": use " + entry.arm->getName());
        }
        if (!isFinite(target)) return fail("Target pose has a non-finite value");

        BasePose base;
        if (!currentBase(base, why)) return fail(why);

        geometry_msgs::msg::Pose goal;
        const std::string frame = frame_id.empty() ? "base_footprint" : frame_id;
        try {
            tf2::doTransform(target, goal, tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero));
        } catch (const tf2::TransformException & ex) {
            return fail(std::string("Cannot freeze the goal in map: ") + ex.what());
        }

        clearVoxelsInsideRobot();
        applyPlanningParameters();
        MoveGroupInterface::Plan plan;
        std::shared_ptr<MoveGroupInterface> runner;
        double travel = 0.0;

        std::map<std::string, double> joints;
        if (solveForArmAlone(entry, goal, base, joints)) {
            setStartToCurrent(entry.arm);
            if (entry.arm->setJointValueTarget(joints) &&
                entry.arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                runner = entry.arm;
            } else {
                RCLCPP_INFO(
                    node_->get_logger(), "[%s] No arm-only path to the IK solution: sampling base poses",
                    group_name.c_str());
            }
        }
        if (canceled()) return cancelResult();

        if (!runner) {
            Outcome planned = planWithSampledBase(entry, group_name, goal, base, canceled, plan, travel);
            if (!planned.success) return planned;
            runner = entry.body;
        }

        const std::string note =
            runner == entry.arm ? " (base fixed)" : " (base travel " + std::to_string(travel) + " m)";
        if (plan_only) {
            std::lock_guard<std::mutex> lock(mutex_);
            stored_[group_name] = StoredPlan{plan, base, runner == entry.arm};
            return Outcome{true, false, "Plan Only" + note};
        }

        // Planning can take many seconds, and Nav2 or a joystick can take the base meanwhile.
        if (!passesGate(group_name, why)) return fail(why);
        if (baseMovedSince(base, why)) return fail(why);
        if (canceled()) return cancelResult();

        Outcome executed = execute(runner, plan, canceled, ticket);
        if (executed.success) {
            if (on_motion_success_) on_motion_success_();
            // Planned travel, so it only describes a motion that finished.
            executed.status += note;
        }
        return executed;
    }

    // Executes the plan a plan-only goal stored, if the base has not moved since.
    // MoveIt's own start check is one-sided for the planar joint, so this checks both sides.
    Outcome executeStored(const std::string & group_name)
    {
        StoredPlan stored;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = stored_.find(group_name);
            if (it == stored_.end()) {
                return fail("No stored plan for group '" + group_name + "'");
            }
            stored = it->second;
        }
        const MotionTicket ticket = beginWholeBodyMotion();
        if (!ticket) return fail("Refused: another motion is in progress (plan still stored)");
        std::string why;
        if (!passesGate(group_name, why)) return fail(why + " (plan still stored)");
        if (baseMovedSince(stored.base, why)) return fail(why + " (plan still stored)");
        const Entry & entry = entries_.at(group_name);
        Outcome executed =
            execute(stored.arm_only ? entry.arm : entry.body, stored.plan, [] { return false; }, ticket);
        if (executed.success) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stored_.erase(group_name);
            }
            if (on_motion_success_) on_motion_success_();
        }
        return executed;
    }

private:
    struct Entry
    {
        std::shared_ptr<MoveGroupInterface> body;  // whole_body_*: base, lift, arm
        std::shared_ptr<MoveGroupInterface> arm;   // *_arm_lift: base stays still
        std::string tip;
        std::string shoulder;
        std::map<std::string, double> seed;  // IK seed for goals with a sampled base
    };

    struct StoredPlan
    {
        MoveGroupInterface::Plan plan;
        BasePose base;
        bool arm_only;
    };

    // Shared with tickets and execution threads, which can outlive this object at shutdown.
    struct MotionState
    {
        std::mutex mutex;
        bool whole_body_active{false};
        int arm_active{0};
        int executing{0};  // whole-body executions move_group has not finished
    };

    MotionTicket beginWholeBodyMotion()
    {
        std::lock_guard<std::mutex> lock(motion_->mutex);
        if (motion_->whole_body_active || motion_->arm_active > 0) return nullptr;
        motion_->whole_body_active = true;
        return MotionTicket(motion_.get(), [motion = motion_](void *) {
            std::lock_guard<std::mutex> release(motion->mutex);
            motion->whole_body_active = false;
        });
    }

    // The controllers a whole-body plan can use, as named in moveit_controllers.yaml, each
    // with a joint that only it moves.
    static const std::vector<std::pair<std::string, std::string>> & controllerJoints()
    {
        static const std::vector<std::pair<std::string, std::string>> joints = {
            {"base_controller", "virtual_joint"},
            {"left_joint_trajectory_controller", "openarm_left_joint1"},
            {"right_joint_trajectory_controller", "openarm_right_joint1"},
            {"lift_controller", "lift_joint"}};
        return joints;
    }

    static std::set<std::string> controllersOf(const MoveGroupInterface::Plan & plan)
    {
        std::set<std::string> joints(
            plan.trajectory.joint_trajectory.joint_names.begin(), plan.trajectory.joint_trajectory.joint_names.end());
        joints.insert(
            plan.trajectory.multi_dof_joint_trajectory.joint_names.begin(),
            plan.trajectory.multi_dof_joint_trajectory.joint_names.end());
        std::set<std::string> controllers;
        for (const auto & [controller, joint] : controllerJoints()) {
            if (joints.count(joint) > 0) controllers.insert(controller);
        }
        return controllers;
    }

    static Outcome fail(const std::string & status)
    {
        return Outcome{false, false, status};
    }

    static Outcome cancelResult()
    {
        return Outcome{false, true, "Canceled"};
    }

    static bool isFinite(const geometry_msgs::msg::Pose & p)
    {
        return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
               std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
               std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
    }

    void applyPlanningParameters()
    {
        // Read before taking the lock: the parameter callback holds the node's
        // parameter lock while it waits for mutex_ in applyParameter().
        const double arm_velocity = node_->get_parameter("max_velocity_scaling_factor").as_double();
        const double arm_acceleration = node_->get_parameter("max_acceleration_scaling_factor").as_double();
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto & [name, entry] : entries_) {
            (void)name;
            entry.body->setPlannerId(planner_id_);
            entry.body->setPlanningTime(planning_time_);
            entry.body->setMaxVelocityScalingFactor(velocity_scaling_);
            entry.body->setMaxAccelerationScalingFactor(acceleration_scaling_);
            entry.body->setGoalJointTolerance(goal_joint_tolerance_);
            entry.arm->setPlannerId(planner_id_);
            entry.arm->setPlanningTime(planning_time_);
            // A goal the arm can reach alone moves at the speed of any other arm goal.
            entry.arm->setMaxVelocityScalingFactor(arm_velocity);
            entry.arm->setMaxAccelerationScalingFactor(arm_acceleration);
            entry.arm->setGoalJointTolerance(goal_joint_tolerance_);
        }
    }

    // Runs on every whole-body request. Each check names what a caller has to fix.
    bool passesGate(const std::string & group_name, std::string & why)
    {
        if (base_mode_ != "whole_body") {
            why = "Whole-body mode unavailable: this bring-up runs base_mode:=" + base_mode_;
            return false;
        }
        if (entries_.find(group_name) == entries_.end()) {
            why = "Whole-body mode unavailable: this commander's robot model has no base variables in " + group_name;
            return false;
        }
        if (!sceneAllowsWholeBody(why)) return false;
        BasePose first;
        if (!currentBase(first, why)) return false;
        if (!base_server_->wait_for_action_server(std::chrono::seconds(1))) {
            why = "No base server on /base_controller/follow_joint_trajectory: is the base follower running?";
            return false;
        }
        const std::string nav2_action = nav2ActiveAction();
        if (!nav2_action.empty()) {
            why = "Nav2 has an active goal on " + nav2_action + ": the base is in use";
            return false;
        }
        // Two TF samples: the commander has no other view of base speed that also
        // covers the map to odom correction. The window is on the node's clock, because
        // a loaded machine runs the simulation at a fraction of real time and /clock can
        // stall for longer than a fixed sleep. A tiny interval would also inflate the speed.
        const rclcpp::Time first_time = node_->now();
        const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        double interval = 0.0;
        while (interval < 0.25 && std::chrono::steady_clock::now() < give_up) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            interval = (node_->now() - first_time).seconds();
        }
        if (interval <= 0.0) {
            why = "The node's clock is not advancing: is the simulation running?";
            return false;
        }
        BasePose second;
        if (!currentBase(second, why)) return false;
        const double speed = std::hypot(second[0] - first[0], second[1] - first[1]) / interval;
        const double turn_rate = std::abs(std::remainder(second[2] - first[2], 2.0 * M_PI)) / interval;
        if (speed > 0.03 || turn_rate > 0.05) {
            why = "The base is moving (" + std::to_string(speed) + " m/s, " + std::to_string(turn_rate) +
                  " rad/s): wait until it is at rest";
            return false;
        }
        return true;
    }

    // The Nav2 action with an active goal, or an empty string. A server that dies during a
    // goal never reports its end, and its last status stays latched here, so an action
    // whose status topic has no publisher left counts as idle.
    std::string nav2ActiveAction()
    {
        std::vector<std::string> active;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto & [action, on] : nav2_active_) {
                if (on) active.push_back(action);
            }
        }
        for (const auto & action : active) {
            if (node_->count_publishers("/" + action + "/_action/status") > 0) return action;
        }
        return "";
    }

    // A plan starts from the base pose it was planned at. The base follower refuses a
    // far start, but an arm-only plan never asks it, and would end in the wrong place in map.
    bool baseMovedSince(const BasePose & planned, std::string & why)
    {
        BasePose base;
        if (!currentBase(base, why)) return true;
        const double moved = std::hypot(base[0] - planned[0], base[1] - planned[1]);
        const double turned = std::abs(std::remainder(base[2] - planned[2], 2.0 * M_PI));
        if (moved <= 0.05 && turned <= 0.05) return false;
        why = "Base moved " + std::to_string(moved) + " m, " + std::to_string(turned) +
              " rad since the plan: plan again";
        return true;
    }

    // Asks move_group, because it can run another SRDF than this commander. This
    // must never request the OCTOMAP component: reading it races the updater and
    // has crashed move_group.
    bool sceneAllowsWholeBody(std::string & why)
    {
        if (!get_planning_scene_->service_is_ready()) {
            why = "Planning scene service not available: is move_group up?";
            return false;
        }
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                         moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto future = get_planning_scene_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            why = "Timed out asking move_group for its planning scene";
            return false;
        }
        const auto response = future.get();
        const auto & names = response->scene.robot_state.multi_dof_joint_state.joint_names;
        if (std::find(names.begin(), names.end(), "virtual_joint") == names.end()) {
            why = "move_group's robot model has no base joint: start it with base_mode:=whole_body";
            return false;
        }
        // Turning all checking off also allows <octomap>, so base_link is the one to report first.
        const auto & acm = response->scene.allowed_collision_matrix;
        const auto allowed = [&acm](const std::string & name) {
            for (size_t i = 0; i < acm.default_entry_names.size() && i < acm.default_entry_values.size(); ++i) {
                if (acm.default_entry_names[i] == name && acm.default_entry_values[i]) return true;
            }
            return false;
        };
        if (allowed("base_link")) {
            why = "Collision checking is off: call toggle_all_collision_checking first";
            return false;
        }
        if (allowed("<octomap>")) {
            why = "Octomap collision checking is off: call toggle_octomap_collision first";
            return false;
        }
        return true;
    }

    bool currentBase(BasePose & base, std::string & why)
    {
        // The TF buffer can still be empty just after a start.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero) &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        try {
            const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            tf2::Quaternion q;
            tf2::fromMsg(t.transform.rotation, q);
            base = {t.transform.translation.x, t.transform.translation.y, tf2::getYaw(q)};
            return true;
        } catch (const tf2::TransformException & ex) {
            why = std::string("No base pose in map: ") + ex.what();
            return false;
        }
    }

    static moveit_msgs::msg::RobotState baseState(const BasePose & base)
    {
        moveit_msgs::msg::RobotState state;
        state.is_diff = true;
        state.multi_dof_joint_state.header.frame_id = "map";
        state.multi_dof_joint_state.joint_names = {"virtual_joint"};
        geometry_msgs::msg::Transform t;
        t.translation.x = base[0];
        t.translation.y = base[1];
        t.rotation.z = std::sin(base[2] / 2.0);
        t.rotation.w = std::cos(base[2] / 2.0);
        state.multi_dof_joint_state.transforms = {t};
        return state;
    }

    // IK on the arm and lift group with the base held at the given pose. Joints that are
    // not in the seed start from the current state.
    bool solveAtBase(
        const Entry & entry, const geometry_msgs::msg::Pose & target_in_map, const BasePose & base,
        std::map<std::string, double> & joints, const std::map<std::string, double> & seed = {})
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        request->ik_request.group_name = entry.arm->getName();
        request->ik_request.robot_state = baseState(base);
        for (const auto & [name, value] : seed) {
            request->ik_request.robot_state.joint_state.name.push_back(name);
            request->ik_request.robot_state.joint_state.position.push_back(value);
        }
        request->ik_request.avoid_collisions = true;
        request->ik_request.ik_link_name = entry.tip;
        request->ik_request.pose_stamped.header.frame_id = "map";
        request->ik_request.pose_stamped.pose = target_in_map;
        auto future = compute_ik_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
        const auto response = future.get();
        if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) return false;
        const auto & active = entry.arm->getActiveJoints();
        joints.clear();
        const auto & state = response->solution.joint_state;
        for (size_t i = 0; i < state.name.size(); ++i) {
            if (std::find(active.begin(), active.end(), state.name[i]) != active.end()) {
                joints[state.name[i]] = state.position[i];
            }
        }
        return joints.size() == active.size();
    }

    // A joint solution for the arm and lift with the base where it is. Moving the base for a
    // goal the arm can reach is the one thing this first attempt exists to prevent, and one
    // IK answer says little: each call gets about 50 ms, and its solution can miss the
    // clearance check from one seed and pass it from another. So this tries a few times.
    bool solveForArmAlone(
        const Entry & entry, const geometry_msgs::msg::Pose & goal, const BasePose & base,
        std::map<std::string, double> & joints)
    {
        const std::map<std::string, double> current_state;
        const std::array<const std::map<std::string, double> *, 4> seeds = {
            &current_state, &entry.seed, &current_state, &entry.seed};
        int solved = 0;
        for (const auto * seed : seeds) {
            if (!solveAtBase(entry, goal, base, joints, *seed)) continue;
            ++solved;
            if (goalHasClearance(base, joints)) return true;
        }
        RCLCPP_INFO(
            node_->get_logger(), "[%s] No usable arm-only solution (%d of %zu IK calls solved, none with clearance): "
            "sampling base poses", entry.arm->getName().c_str(), solved, seeds.size());
        return false;
    }

    // The robot cannot stand inside an obstacle, so octomap contact at the current state
    // proves those voxels are wrong. A stale voxel behind the robot's own surface gets no
    // ray through it, so it stays, and every plan fails its start check until it goes.
    // MoveIt has no way to edit part of the octree, so this clears all of it. What is out
    // of the camera's view at this moment is forgotten and plans as free space.
    void clearVoxelsInsideRobot()
    {
        auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->robot_state.is_diff = true;
        auto future = state_validity_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return;
        const auto response = future.get();
        std::string links;
        for (const auto & contact : response->contacts) {
            if (contact.contact_body_1 == "<octomap>") links += " " + contact.contact_body_2;
            else if (contact.contact_body_2 == "<octomap>") links += " " + contact.contact_body_1;
        }
        if (links.empty()) return;
        RCLCPP_WARN(node_->get_logger(), "Clearing the octomap: the current state touches voxels at%s", links.c_str());
        auto cleared = clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        if (cleared.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return;
        // The camera publishes at about 5 Hz of the node's clock, so a second on that clock
        // lets the updater put back what the camera sees. A simulator below real time
        // stretches that second; the wall bound catches a clock that has stopped.
        const rclcpp::Time cleared_at = node_->now();
        const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while ((node_->now() - cleared_at).seconds() < 1.0 && std::chrono::steady_clock::now() < give_up) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // A position controller can stop just past a joint limit, and from there every plan
    // fails with "Start state out of bounds" until something moves that joint. MoveIt's
    // fix_start_state does not help: it only normalizes continuous and planar joints. So
    // a joint slightly past its limit starts from the limit. A joint farther out than
    // this is left alone, and MoveIt reports it, because that is a real fault. MoveIt
    // checks the bounds of the planned group only, so other joints keep their real value.
    moveit_msgs::msg::RobotState startAtJointLimits(const std::vector<std::string> & variables)
    {
        const double tolerance = 0.02;
        moveit_msgs::msg::RobotState start;
        start.is_diff = true;
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & name : variables) {
            const auto known = joint_positions_.find(name);
            if (known == joint_positions_.end()) continue;
            const double value = known->second;
            const auto & bounds = model_->getVariableBounds(name);
            if (!bounds.position_bounded_) continue;
            const double limit = std::clamp(value, bounds.min_position_, bounds.max_position_);
            if (limit != value && std::abs(limit - value) <= tolerance) {
                start.joint_state.name.push_back(name);
                start.joint_state.position.push_back(limit);
            }
        }
        return start;
    }

    // A goal with little clearance can settle into contact with the robot itself, and
    // from there every plan fails CheckStartStateCollision. MoveIt no longer has an
    // adapter that repairs a start state in collision, so a start a little higher stands
    // in for the current state, and the controller closes the difference when the
    // trajectory begins. allowed_start_tolerance in move_group.yaml accepts these raises.
    // Contact that they do not clear is left for MoveIt to report, because that is a
    // real collision.
    void raiseLiftOutOfSelfContact(moveit_msgs::msg::RobotState & start)
    {
        if (stateValid(start)) return;
        auto & names = start.joint_state.name;
        auto & positions = start.joint_state.position;
        const auto listed = std::find(names.begin(), names.end(), "lift_joint");
        double lift;
        if (listed != names.end()) {
            lift = positions[static_cast<size_t>(listed - names.begin())];
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto known = joint_positions_.find("lift_joint");
            if (known == joint_positions_.end()) return;
            lift = known->second;
        }
        const double highest = model_->getVariableBounds("lift_joint").max_position_;
        for (const double raise : {0.002, 0.005, 0.01, 0.02}) {
            if (lift + raise > highest) return;
            auto candidate = start;
            const auto at = std::find(candidate.joint_state.name.begin(), candidate.joint_state.name.end(), "lift_joint");
            if (at != candidate.joint_state.name.end()) {
                candidate.joint_state.position[static_cast<size_t>(at - candidate.joint_state.name.begin())] = lift + raise;
            } else {
                candidate.joint_state.name.push_back("lift_joint");
                candidate.joint_state.position.push_back(lift + raise);
            }
            if (stateValid(candidate)) {
                RCLCPP_WARN(
                    node_->get_logger(), "The current state is in contact: planning from the lift %.0f mm higher",
                    raise * 1000.0);
                start = candidate;
                return;
            }
        }
    }

    // The current state with the small repairs a plan needs to be able to start from it.
    // Each repair changes only joints of the planned group: the trajectory commands those
    // joints alone, so a change to any other joint would plan for a robot that is not there.
    // A group without the lift therefore gets no raise, and MoveIt reports the contact.
    moveit_msgs::msg::RobotState repairedStart(const std::string & group_name)
    {
        if (!model_->hasJointModelGroup(group_name)) return moveit_msgs::msg::RobotState();
        const auto & variables = model_->getJointModelGroup(group_name)->getVariableNames();
        auto start = startAtJointLimits(variables);
        if (std::find(variables.begin(), variables.end(), "lift_joint") != variables.end()) {
            raiseLiftOutOfSelfContact(start);
        }
        return start;
    }

    // The lift is exempt: its top stop is a normal pose, where the home state stands, and
    // goals near shoulder height often need it.
    bool nearJointLimit(const std::map<std::string, double> & joints) const
    {
        const double margin = 0.01;
        for (const auto & [name, value] : joints) {
            if (name == "lift_joint") continue;
            const auto & bounds = model_->getVariableBounds(name);
            if (bounds.position_bounded_ &&
                (value < bounds.min_position_ + margin || value > bounds.max_position_ - margin)) {
                return true;
            }
        }
        return false;
    }

    // No group name, so this checks the whole robot: compute_ik only checks the links of
    // its group, and the idle arm can reach the base when the lift is low.
    bool stateValid(const moveit_msgs::msg::RobotState & state)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->robot_state = state;
        auto future = state_validity_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
        return future.get()->valid;
    }

    bool stateValidAtBase(const BasePose & base, const std::map<std::string, double> & joints)
    {
        auto state = baseState(base);
        for (const auto & [name, value] : joints) {
            state.joint_state.name.push_back(name);
            state.joint_state.position.push_back(value);
        }
        return stateValid(state);
    }

    // The lift settles below its command (about 3 mm in MuJoCo), so a goal also has to be
    // valid somewhat below where it asks the lift to stop.
    bool goalHasClearance(const BasePose & base, std::map<std::string, double> joints)
    {
        if (!stateValidAtBase(base, joints)) return false;
        const auto lift = joints.find("lift_joint");
        if (lift == joints.end()) return true;
        const double margin = 0.01;
        lift->second = std::max(lift->second - margin, model_->getVariableBounds("lift_joint").min_position_);
        return stateValidAtBase(base, joints);
    }

    // Base poses that put the arm's shoulder behind the hand, nearest to the current base
    // first. Reach and heading have to vary together, because the shoulder is off the base
    // centre. The first tier keeps to where IK finds usable goals, measured for a hand
    // pointing forward at about 1 m: the shoulder 0.3 to 0.6 m behind the hand, the hand
    // mostly on the arm's own side of the shoulder, and the body turned up to 1 rad away
    // from that side, which keeps the target near the head camera's view. The second tier
    // is wider, for other hand orientations.
    std::vector<BasePose> sampleBasePoses(
        const Entry & entry, const geometry_msgs::msg::Pose & goal, const BasePose & base)
    {
        // Base behind the hand when the hand points sideways enough, otherwise on
        // the line to the target.
        tf2::Quaternion q;
        tf2::fromMsg(goal.orientation, q);
        const tf2::Vector3 hand_z = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
        const double approach = std::hypot(hand_z.x(), hand_z.y()) > 0.3
                                    ? std::atan2(hand_z.y(), hand_z.x())
                                    : std::atan2(goal.position.y - base[1], goal.position.x - base[0]);
        double shoulder_x = 0.0;
        double shoulder_y = 0.0;
        try {
            const auto t = tf_buffer_->lookupTransform("base_footprint", entry.shoulder, tf2::TimePointZero);
            shoulder_x = t.transform.translation.x;
            shoulder_y = t.transform.translation.y;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "Sampling around the base centre: %s", ex.what());
        }
        // Mirrors the first tier for the arm on the other side.
        const double side = shoulder_y >= 0.0 ? 1.0 : -1.0;

        struct Tier
        {
            double reach_min, reach_max, around_min, around_max, turn_min, turn_max;
        };
        const std::array<Tier, 2> tiers = {{{0.30, 0.60, -0.4, 0.8, -1.0, 0.0}, {0.20, 0.70, -0.8, 0.8, -1.5, 1.5}}};
        std::vector<BasePose> samples;
        for (const auto & range : tiers) {
            std::uniform_real_distribution<double> reach(range.reach_min, range.reach_max);
            std::uniform_real_distribution<double> around(range.around_min, range.around_max);
            std::uniform_real_distribution<double> turn(range.turn_min, range.turn_max);
            const auto tier = static_cast<std::ptrdiff_t>(samples.size());
            for (int i = 0; i < 40; ++i) {
                const double r = reach(rng_);
                const double a = approach + side * around(rng_);
                const double yaw = approach + side * turn(rng_);
                samples.push_back(
                    {goal.position.x - r * std::cos(a) - (std::cos(yaw) * shoulder_x - std::sin(yaw) * shoulder_y),
                     goal.position.y - r * std::sin(a) - (std::sin(yaw) * shoulder_x + std::cos(yaw) * shoulder_y),
                     std::remainder(yaw, 2.0 * M_PI)});
            }
            std::sort(samples.begin() + tier, samples.end(), [&](const BasePose & a, const BasePose & b) {
                return std::hypot(a[0] - base[0], a[1] - base[1]) < std::hypot(b[0] - base[0], b[1] - base[1]);
            });
        }
        return samples;
    }

    Outcome planWithSampledBase(
        const Entry & entry, const std::string & group_name, const geometry_msgs::msg::Pose & goal,
        const BasePose & base, const CancelCheck & canceled, MoveGroupInterface::Plan & plan, double & travel)
    {
        const auto samples = sampleBasePoses(entry, goal, base);
        double planning_time;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            planning_time = planning_time_;
        }
        int with_ik = 0;
        int on_limit = 0;
        int solved = 0;
        int last_code = 0;
        std::string last_spin;
        // A failed plan or a spinning plan moves on to the next sampled base, because
        // one base pose can sit behind an obstacle while the next one does not.
        const int max_plan_attempts = 5;
        for (const auto & sample : samples) {
            if (canceled()) return cancelResult();
            std::map<std::string, double> joints;
            if (!solveAtBase(entry, goal, sample, joints, entry.seed)) continue;
            ++with_ik;
            // Another sample costs nothing, and a goal on a joint limit can end just
            // past it, where the next plan starts out of bounds.
            if (nearJointLimit(joints)) {
                ++on_limit;
                continue;
            }
            if (!goalHasClearance(sample, joints)) continue;
            ++solved;

            joints["virtual_joint/x"] = sample[0];
            joints["virtual_joint/y"] = sample[1];
            joints["virtual_joint/theta"] = sample[2];
            // By-name start: the current-state message fails to plan in part of the heading range.
            moveit_msgs::msg::RobotState start = repairedStart(entry.body->getName());
            start.joint_state.name.insert(
                start.joint_state.name.end(), {"virtual_joint/x", "virtual_joint/y", "virtual_joint/theta"});
            start.joint_state.position.insert(start.joint_state.position.end(), {base[0], base[1], base[2]});
            entry.body->setStartState(start);
            if (!entry.body->setJointValueTarget(joints)) {
                return fail("Base goal outside the joint bounds on " + group_name);
            }
            // RRTConnect samples the base uniformly in this box, and path simplification
            // does not always remove a waypoint far off the straight line. A 5 m box gave
            // 2.18 m of travel for a 1.02 m displacement, through space the camera may not
            // have seen. So the small box around the straight line comes first, and the
            // large one only finds a way around an obstacle.
            const auto plan_in_box = [&](double margin, double seconds) {
                entry.body->setPlanningTime(seconds);
                entry.body->setWorkspace(
                    std::min(base[0], sample[0]) - margin, std::min(base[1], sample[1]) - margin, -1.0,
                    std::max(base[0], sample[0]) + margin, std::max(base[1], sample[1]) + margin, 3.0);
                return entry.body->plan(plan);
            };
            auto code = plan_in_box(0.3, std::min(2.0, planning_time));
            if (code != moveit::core::MoveItErrorCode::SUCCESS && !canceled()) {
                code = plan_in_box(5.0, planning_time);
            }
            if (code != moveit::core::MoveItErrorCode::SUCCESS) {
                last_code = code.val;
            } else {
                const double turned = totalTurn(plan);
                const double heading_change = std::abs(std::remainder(sample[2] - base[2], 2.0 * M_PI));
                // The UnwrapPlanarHeading adapter in move_group keeps a plan from taking
                // the long way round across +-pi. This catches a pipeline that runs
                // without it, or a plan that really does spin. Smaller wiggles around
                // obstacles are normal, hence the margin of pi.
                if (turned <= heading_change + M_PI) {
                    travel = basePathLength(plan);
                    return Outcome{true, false, ""};
                }
                last_spin = " Last plan spun " + std::to_string(turned) + " rad for a heading change of " +
                            std::to_string(heading_change) + " rad.";
            }
            if (solved >= max_plan_attempts) break;
        }
        if (solved == 0) {
            return fail("No base pose found to reach the target (" + std::to_string(samples.size()) + " samples: " +
                        std::to_string(with_ik) + " with an IK solution, " + std::to_string(on_limit) +
                        " of those on a joint limit, the rest in collision or too close to it)");
        }
        return fail("Planning Failed for " + std::to_string(solved) + " sampled base pose(s), last MoveIt code " +
                    std::to_string(last_code) + "." + last_spin);
    }

    static double totalTurn(const MoveGroupInterface::Plan & plan)
    {
        double turned = 0.0;
        const auto & points = plan.trajectory.multi_dof_joint_trajectory.points;
        for (size_t i = 1; i < points.size(); ++i) {
            const auto & a = points[i - 1].transforms.at(0).rotation;
            const auto & b = points[i].transforms.at(0).rotation;
            turned += std::abs(std::remainder(2.0 * (std::atan2(b.z, b.w) - std::atan2(a.z, a.w)), 2.0 * M_PI));
        }
        return turned;
    }

    static double basePathLength(const MoveGroupInterface::Plan & plan)
    {
        double length = 0.0;
        const auto & points = plan.trajectory.multi_dof_joint_trajectory.points;
        for (size_t i = 1; i < points.size(); ++i) {
            const auto & a = points[i - 1].transforms.at(0).translation;
            const auto & b = points[i].transforms.at(0).translation;
            length += std::hypot(b.x - a.x, b.y - a.y);
        }
        return length;
    }

    static double trajectoryDuration(const MoveGroupInterface::Plan & plan)
    {
        double duration = 0.0;
        const auto & joints = plan.trajectory.joint_trajectory.points;
        if (!joints.empty()) duration = rclcpp::Duration(joints.back().time_from_start).seconds();
        const auto & base = plan.trajectory.multi_dof_joint_trajectory.points;
        if (!base.empty()) duration = std::max(duration, rclcpp::Duration(base.back().time_from_start).seconds());
        return duration;
    }

    // A frozen or dead controller can hold execute() far longer than a client waits, and
    // the execution manager does not act on one controller's abort until it reaches that
    // controller. So execute() runs on its own thread while this loop watches for every
    // reason to stop. The thread owns copies of what it uses, so it stays safe if it
    // outlives this call or this object, and its copy of the ticket keeps the motion slot
    // taken until move_group has really finished.
    Outcome execute(
        const std::shared_ptr<MoveGroupInterface> & group, const MoveGroupInterface::Plan & plan,
        const CancelCheck & canceled, const MotionTicket & ticket)
    {
        double margin;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            margin = execution_margin_;
            aborted_controller_.clear();
            watched_controllers_ = controllersOf(plan);
            goals_before_execution_.clear();
            for (const auto & [controller, goals] : controller_goals_) {
                (void)controller;
                goals_before_execution_.insert(goals.begin(), goals.end());
            }
        }
        auto promise = std::make_shared<std::promise<int>>();
        const std::shared_future<int> future = promise->get_future().share();
        auto plan_copy = std::make_shared<MoveGroupInterface::Plan>(plan);
        {
            std::lock_guard<std::mutex> lock(motion_->mutex);
            ++motion_->executing;
        }
        std::thread([group, plan_copy, promise, ticket, motion = motion_]() {
            int code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            try {
                code = group->execute(*plan_copy).val;
            } catch (const std::exception &) {
                // The context shut down during the motion.
            }
            {
                std::lock_guard<std::mutex> lock(motion->mutex);
                --motion->executing;
            }
            promise->set_value(code);
        }).detach();

        // Trajectories run on the node's clock, and a simulator can run far below real
        // time, so the deadline is on that clock too. The wall clock bound catches a
        // clock that has stopped, where the first one never fires.
        const double allowed = 1.5 * trajectoryDuration(plan) + margin;
        const rclcpp::Time started = node_->now();
        const auto wall_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(10.0 * allowed);
        while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            if (canceled()) return halt(group, future, cancelResult());
            std::string aborted;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                aborted = aborted_controller_;
            }
            if (!aborted.empty()) {
                return halt(group, future, fail("Stopped: " + aborted + " aborted its goal"));
            }
            const std::string nav2_action = nav2ActiveAction();
            if (!nav2_action.empty()) {
                return halt(group, future, fail("Stopped: Nav2 took a goal on " + nav2_action));
            }
            if ((node_->now() - started).seconds() > allowed || std::chrono::steady_clock::now() > wall_deadline) {
                return halt(
                    group, future, fail("Execution deadline of " + std::to_string(allowed) + " s exceeded: stop sent"));
            }
        }
        const int code = future.get();
        if (canceled()) return cancelResult();
        if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            return fail("Execution Failed, MoveIt code " + std::to_string(code));
        }
        return Outcome{true, false, "Success"};
    }

    // stop() only reaches what the execution manager is already running. A request still on
    // its way to move_group would start after it, so the stop repeats until execute() ends.
    // The caller gets its answer after 10 s at most, and a thread carries on the repeats.
    static Outcome halt(
        const std::shared_ptr<MoveGroupInterface> & group, const std::shared_future<int> & future, Outcome outcome)
    {
        const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        if (!stopUntilFinished(group, future, give_up)) {
            std::thread([group, future]() {
                stopUntilFinished(group, future, std::chrono::steady_clock::time_point::max());
            }).detach();
            outcome.status +=
                " (move_group has not finished: the stop repeats, and new whole-body and arm goals stay refused, "
                "until it does)";
        }
        return outcome;
    }

    // Returns whether execute() finished before the given time.
    static bool stopUntilFinished(
        const std::shared_ptr<MoveGroupInterface> & group, const std::shared_future<int> & future,
        std::chrono::steady_clock::time_point give_up)
    {
        do {
            if (!rclcpp::ok()) return true;
            try {
                group->stop();
            } catch (const std::exception &) {
                return true;  // The context shut down.
            }
            if (std::chrono::steady_clock::now() > give_up) return false;
        } while (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready);
        return true;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_;
    moveit::core::RobotModelConstPtr model_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr base_server_;
    std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> nav2_status_subs_;
    std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> controller_status_subs_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::TimerBase::SharedPtr alive_timer_;
    std::function<void()> on_motion_success_;

    std::map<std::string, Entry> entries_;
    std::string base_mode_;
    std::mt19937 rng_;

    // Guards everything below.
    std::mutex mutex_;
    std::string planner_id_;
    double planning_time_{5.0};
    double velocity_scaling_{1.0};
    double acceleration_scaling_{1.0};
    double goal_joint_tolerance_{0.001};
    double execution_margin_{15.0};
    std::map<std::string, bool> nav2_active_;
    // The goals each controller's status topic lists, and which of them existed before
    // the current execution: only a new goal on a controller of the plan is its abort.
    std::map<std::string, std::set<std::array<uint8_t, 16>>> controller_goals_;
    std::set<std::array<uint8_t, 16>> goals_before_execution_;
    std::set<std::string> watched_controllers_;
    std::string aborted_controller_;
    std::map<std::string, double> joint_positions_;
    std::map<std::string, StoredPlan> stored_;

    // Shared with the tickets and the execute threads, which can outlive this object.
    std::shared_ptr<MotionState> motion_{std::make_shared<MotionState>()};
};
