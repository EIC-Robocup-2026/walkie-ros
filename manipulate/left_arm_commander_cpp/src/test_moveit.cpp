#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <thread>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/robot_state.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;

class DualArmCommander
{
public:
    DualArmCommander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;

        // 1. Initialize Groups for BOTH arms
        // Make sure these names match your SRDF group names!
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "right_arm");
        left_gripper_ = std::make_shared<MoveGroupInterface>(node_, "left_gripper");
        right_gripper_ = std::make_shared<MoveGroupInterface>(node_, "right_gripper");

        // Set scaling
        left_arm_->setMaxVelocityScalingFactor(1.0);
        right_arm_->setMaxVelocityScalingFactor(1.0);
        left_arm_->setMaxAccelerationScalingFactor(1.0);
        right_arm_->setMaxAccelerationScalingFactor(1.0);

        // Print Info
        RCLCPP_INFO(node_->get_logger(), "Left Reference: %s", left_arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(node_->get_logger(), "Right Reference: %s", right_arm_->getPlanningFrame().c_str());

        // Initialize Gripper Action Clients
        left_gripper_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, "/left_hand_controller/follow_joint_trajectory");
        right_gripper_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_, "/right_hand_controller/follow_joint_trajectory");

        // Setup Scene
        // setupScene();
    }

    void setupScene()
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        // --- Table 1 (Right Side) ---
        moveit_msgs::msg::CollisionObject table1;
        table1.id = "table_right";
        table1.header.frame_id = "base_link";
        table1.primitives.resize(1);
        table1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        table1.primitives[0].dimensions = {0.4, 0.4, 0.4}; 
        table1.primitive_poses.resize(1);
        table1.primitive_poses[0].position.x = 0.65;
        table1.primitive_poses[0].position.y = -0.3; // Right side
        table1.primitive_poses[0].position.z = 0.2; 
        table1.operation = table1.ADD;
        collision_objects.push_back(table1);

        // --- Table 2 (Left Side) ---
        moveit_msgs::msg::CollisionObject table2;
        table2.id = "table_left";
        table2.header.frame_id = "base_link";
        table2.primitives.resize(1);
        table2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        table2.primitives[0].dimensions = {0.4, 0.4, 0.4};
        table2.primitive_poses.resize(1);
        table2.primitive_poses[0].position.x = 0.65;
        table2.primitive_poses[0].position.y = 0.3; // Left side
        table2.primitive_poses[0].position.z = 0.2;
        table2.operation = table2.ADD;
        collision_objects.push_back(table2);

        // --- Object Right ---
        moveit_msgs::msg::CollisionObject box_right;
        box_right.id = "box_right";
        box_right.header.frame_id = "base_link";
        box_right.primitives.resize(1);
        box_right.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        box_right.primitives[0].dimensions = {0.04, 0.04, 0.2};
        box_right.primitive_poses.resize(1);
        box_right.primitive_poses[0].position.x = 0.65;
        box_right.primitive_poses[0].position.y = -0.3;
        box_right.primitive_poses[0].position.z = 0.505;
        box_right.operation = box_right.ADD;
        collision_objects.push_back(box_right);

        // --- Object Left ---
        moveit_msgs::msg::CollisionObject box_left;
        box_left.id = "box_left";
        box_left.header.frame_id = "base_link";
        box_left.primitives.resize(1);
        box_left.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        box_left.primitives[0].dimensions = {0.04, 0.04, 0.2};
        box_left.primitive_poses.resize(1);
        box_left.primitive_poses[0].position.x = 0.65;
        box_left.primitive_poses[0].position.y = 0.3;
        box_left.primitive_poses[0].position.z = 0.505;
        box_left.operation = box_left.ADD;
        collision_objects.push_back(box_left);

        // planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "Dual Arm Scene Setup Complete!");
    }

    // --- LEFT ARM ROUTINE ---
    void runLeftArmRoutine()
    {
        RCLCPP_INFO(node_->get_logger(), "[LEFT] Starting Routine...");
        
        controlGripper(left_gripper_, "left_close");

        controlGripper(left_gripper_, "left_open");

        controlGripper(left_gripper_, "left_open");

        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 2. Move to Pre-Grasp (Left Table)
        // x=0.55, y=0.3, z=0.55
        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, 0.0, 1.57,true);

        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, 0.7, 1.57,true);

        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, -0.7, 1.57,true);

        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, 0.7, 1.57,true);

        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, -0.7, 1.57,true);

        goToPoseTarget(left_arm_, 0.268149, 0.722525, 0.308642, 3.14, 0.0, 1.57,true);


            // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, 1.0, 0.0,true);

            // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, -2.0, 0.0,true);

            // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, 2.0, 0.0,true);

            // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, -2.0, 0.0,true);


        // 3. Move Down
        // goToPoseRelative(left_arm_, -0.4, 0.0, 0.95, 3.14, 0.0, 0.0,true);
        
        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, 1.1, 0.0);

        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, -2.2, 0.0);

        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, 2.2, 0.0);

        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, -2.2, 0.0);

        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, 2.2, 0.0);

        // goToPoseRelative(left_arm_,  0.0, 0.0, 0.0, 0.0, -2.2, 0.0);
        
        controlGripper(left_gripper_, "left_close");

        controlGripper(left_gripper_, "left_open");

        RCLCPP_INFO(node_->get_logger(), "[LEFT] Routine Finished!");
    }

    // --- RIGHT ARM ROUTINE ---
    void runRightArmRoutine()
    {
        RCLCPP_INFO(node_->get_logger(), "[RIGHT] Starting Routine...");

        controlGripper(right_gripper_, "right_close");

        controlGripper(right_gripper_, "right_open");

        controlGripper(right_gripper_, "right_close");

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, 0.0, 1.57,true);

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, 0.7, 1.57,true);

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, -0.7, 1.57,true);

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, 0.7, 1.57,true);

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, -0.7, 1.57,true);

        goToPoseTarget(right_arm_, 0.268149, -0.722525, 0.308642, 3.14, 0.7, 1.57,true);

        controlGripper(right_gripper_, "right_open");

        controlGripper(right_gripper_, "right_close");

        RCLCPP_INFO(node_->get_logger(), "[RIGHT] Routine Finished!");
    }

    void runDual()
    {
        // Execute both arms in parallel threads
        std::thread left_thread(&DualArmCommander::runLeftArmRoutine, this);
        std::thread right_thread(&DualArmCommander::runRightArmRoutine, this);

        // Wait for both to finish
        left_thread.join();
        right_thread.join();
        
        RCLCPP_INFO(node_->get_logger(), "DUAL ARM SEQUENCE COMPLETE");
    }

    // --- HELPER FUNCTIONS (Now accepting Group pointers) ---

    void goToPoseRelative(std::shared_ptr<MoveGroupInterface> group, 
                          double dx, double dy, double dz, 
                          double droll, double dpitch, double dyaw,
                          bool use_cartesian = false)
    {
        geometry_msgs::msg::Pose current_pose = group->getCurrentPose().pose;

        tf2::Quaternion q_current;
        tf2::fromMsg(current_pose.orientation, q_current);
        
        tf2::Quaternion q_increment;
        q_increment.setRPY(droll, dpitch, dyaw);
        tf2::Quaternion q_final = q_current * q_increment;
        q_final.normalize();

        tf2::Matrix3x3 mat_rot(q_current);
        tf2::Vector3 v_offset(dx, dy, dz);
        tf2::Vector3 v_offset_rotated = mat_rot * v_offset;

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + v_offset_rotated.x();
        target_pose.position.y = current_pose.position.y + v_offset_rotated.y();
        target_pose.position.z = current_pose.position.z + v_offset_rotated.z();
        target_pose.orientation = tf2::toMsg(q_final);

        if (use_cartesian) {
            executeCartesianTarget(group, target_pose);
        } else {
            executeTarget(group, target_pose);
        }
    }

    /**
     * @brief Moves the arm to an absolute target pose.
     */
    void goToPoseTarget(std::shared_ptr<MoveGroupInterface> group,
                        double x, double y, double z, 
                        double roll, double pitch, double yaw,
                        bool use_cartesian = false)
    {
        tf2::Quaternion q; 
        q.setRPY(roll, pitch, yaw); 
        q.normalize();
        
        geometry_msgs::msg::Pose target_pose; 
        target_pose.position.x = x; 
        target_pose.position.y = y; 
        target_pose.position.z = z; 
        target_pose.orientation = tf2::toMsg(q);
        
        if (use_cartesian) {
            executeCartesianTarget(group, target_pose);
        } else {
            executeTarget(group, target_pose);
        }
    }

    // --- Standard OMPL/BiTRRT Planner ---
    void executeTarget(std::shared_ptr<MoveGroupInterface> group, const geometry_msgs::msg::Pose& target)
    {
        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_link");
        group->setPoseTarget(target);

        MoveGroupInterface::Plan plan;
        if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            group->execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning Failed for %s", group->getName().c_str());
        }
    }

    // --- Cartesian Linear Planner ---
    void executeCartesianTarget(std::shared_ptr<MoveGroupInterface> group, const geometry_msgs::msg::Pose& target)
    {
        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_link");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;
        
        // UPDATED LINE: Removed the 0.0 jump_threshold argument
        double fraction = group->computeCartesianPath(waypoints, 0.01, trajectory);

        if (fraction > 0.90) { 
            // Time Parameterization for SPEED (Using TOTG)
            robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), group->getName());
            rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
            
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            if (totg.computeTimeStamps(rt, 1.0, 1.0)) { // 1.0 scaling = max speed
                rt.getRobotTrajectoryMsg(trajectory);
            }

            auto err = group->execute(trajectory);
            if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Cartesian Move Succeeded (%.2f%%)", fraction * 100.0);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Cartesian Move Failed during execution: %d", err.val);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Cartesian path failed (Only %.2f%% computed)", fraction * 100.0);
        }
    }
    
    void controlGripper(std::shared_ptr<MoveGroupInterface> gripper, const std::string& state) 
    { 
        // Identify side and parameters
        std::string side = (gripper->getName() == "left_gripper") ? "left" : "right";
        std::string controller_name = side + "_hand_controller";
        std::string joint_name = side + "_gripper_controller";
        std::string action_topic = "/" + controller_name + "/follow_joint_trajectory";

        // Determine target position (Ignoring SRDF values which appear out of bounds)
        double target_pos = 0.0;
        if (state.find("open") != std::string::npos) {
            target_pos = -10.0;
        } else if (state.find("half") != std::string::npos) {
            target_pos = -5.0;
        } else if (state.find("close") != std::string::npos) {
            target_pos = 0.0;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown gripper state '%s', defaulting to 0.0", state.c_str());
        }

        RCLCPP_INFO(node_->get_logger(), "Setting gripper '%s' (%s) to %.2f...", side.c_str(), state.c_str(), target_pos);

        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

        // Select the recurring client
        auto action_client = (gripper->getName() == "left_gripper") ? left_gripper_client_ : right_gripper_client_;

        if (!action_client->wait_for_action_server(std::chrono::seconds(2))) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper action server %s not available", action_topic.c_str());
             return;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.header.frame_id = "base_link";
        goal.trajectory.joint_names = {joint_name};
        
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = {target_pos};
        p.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal.trajectory.points.push_back(p);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        // Synchronous-like wait using a future (simplified for this script)
        auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);
        
        // Wait for goal acceptance
        if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal send timed out");
            return;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
             RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
             return;
        }

        // Wait for result
        auto result_future = action_client->async_get_result(goal_handle);
        // We can wait a bit longer for execution
        if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
             auto result = result_future.get();
             if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                 RCLCPP_INFO(node_->get_logger(), "Gripper action succeeded");
             } else {
                 RCLCPP_ERROR(node_->get_logger(), "Gripper action failed/aborted");
             }
        } else {
             RCLCPP_WARN(node_->get_logger(), "Gripper action timed out (but might still finish)");
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    
    // Define all 4 Groups
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;
    std::shared_ptr<MoveGroupInterface> left_gripper_;
    std::shared_ptr<MoveGroupInterface> right_gripper_;
    
    PlanningSceneInterface planning_scene_interface_;
    
    // Persistent Action Clients for Grippers
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr left_gripper_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr right_gripper_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // node_options.parameter_overrides({{"use_sim_time", true}});
    
    auto node = std::make_shared<rclcpp::Node>("dual_arm_commander", node_options);
    auto commander = std::make_shared<DualArmCommander>(node);

    // Background thread for ROS Callbacks
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    // Run the Logic
    commander->runDual();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;

}