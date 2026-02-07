#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <thread> 
#include <chrono> // Added for time delays

// --- 1. INCLUDE SERVICE HEADER ---
#include "my_robot_interfaces/srv/get_position.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using namespace std::chrono_literals; // Allows using 1s, 500ms

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        
        // --- 2. CREATE CLIENT ---
        client_ = node_->create_client<my_robot_interfaces::srv::GetPosition>("get_target_position");

        RCLCPP_INFO(node_->get_logger(), "Planning Frame: %s", arm_->getPlanningFrame().c_str());

        // Setup only the tables first. We don't know where the object is yet!
        setupStaticScene();
    }

    void setupStaticScene()
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.resize(2);

        // --- Table 1 (Pick) ---
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "base_link";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions = {0.4, 0.4, 0.4}; 
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.65;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = 0.2; 
        collision_objects[0].operation = collision_objects[0].ADD;

        // --- Table 2 (Place) ---
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = "base_link";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions = {0.4, 0.4, 0.4};
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0.0;
        collision_objects[1].primitive_poses[0].position.y = 0.65;
        collision_objects[1].primitive_poses[0].position.z = 0.2;
        collision_objects[1].operation = collision_objects[1].ADD;

        planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "Tables added to scene.");
    }

    // --- 3. DYNAMIC SPAWN FUNCTION ---
    void spawnObject(double x, double y)
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object_to_pick";
        object.header.frame_id = "base_link";
        object.primitives.resize(1);
        object.primitives[0].type = object.primitives[0].BOX;
        object.primitives[0].dimensions = {0.04, 0.04, 0.2};
        object.primitive_poses.resize(1);
        
        // Use the coordinates we got from Vision!
        object.primitive_poses[0].position.x = x;
        object.primitive_poses[0].position.y = y;
        object.primitive_poses[0].position.z = 0.505; 
        object.operation = object.ADD;

        std::vector<moveit_msgs::msg::CollisionObject> objects = {object};
        planning_scene_interface_.applyCollisionObjects(objects);
        RCLCPP_INFO(node_->get_logger(), "Spawned Object at X:%.3f Y:%.3f", x, y);
    }

    // --- MAIN SEQUENCE LOGIC ---
    void run()
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for Vision Service...");

        // A. Wait for Service
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting...");
        }

        // B. Send Request
        auto request = std::make_shared<my_robot_interfaces::srv::GetPosition::Request>();
        request->get_position = true;
        auto result_future = client_->async_send_request(request);

        // C. Wait for Response
        if (result_future.wait_for(10s) == std::future_status::ready)
        {
            auto response = result_future.get();
            if (response->success)
            {
                double target_x = response->x;
                double target_y = response->y;
                RCLCPP_INFO(node_->get_logger(), "TARGET FOUND -> X: %.3f, Y: %.3f", target_x, target_y);
                
                // --- START SEQUENCE WITH DYNAMIC COORDINATES ---
                
                // 1. Add Object to MoveIt Scene at REAL location
                spawnObject(target_x, target_y);
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Give MoveIt time to update

                openGripper();
                closeGripper();
                openGripper();

                // 2. Move to Pre-Grasp (Dynamic X/Y, Fixed Z=0.65)
                goToPoseTarget(target_x, target_y, 0.55, 3.14, 0.0, 0.0, false);

                // 3. Move Down (Approach)
                goToPoseRelativeTarget(0.1, 0.0, 0.1, 0.0, 0.0, 0.0, true);

                // 4. Grasp
                halfGripper();
                attachObject("object_to_pick");
                std::this_thread::sleep_for(std::chrono::seconds(1));

                std::vector<std::string> table1_id = {"table1"};
                planning_scene_interface_.removeCollisionObjects(table1_id);

                // 5. Lift Up
                goToPoseRelativeTarget(0.0, 0.0, -0.4, 0.0, 0.0, 0.0, true);

                // 6. Move to Place Location (Table 2 - Hardcoded is fine for placing)
                goToPoseTarget(0.0, 0.55, 0.55, 3.14, 0.0, 1.57, true);

                // 7. Move Down
                goToPoseRelativeTarget(0.1, 0.0, 0.1, 0.0, 0.0, 0.0, true); 

                std::vector<std::string> table2_id = {"table2"};
                planning_scene_interface_.removeCollisionObjects(table2_id);

                // 8. Release
                detachObject("object_to_pick");
                std::vector<std::string> object_id = {"object_to_pick"};
                planning_scene_interface_.removeCollisionObjects(object_id);
                openGripper(); 
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // 9. Retreat
                goToPoseRelativeTarget(-0.2, 0.0, -0.2, 0.0, 0.0, 0.0, true);
            }
            else {
                RCLCPP_WARN(node_->get_logger(), "Vision saw nothing! Cannot move.");
            }
        }
        else {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
        }
    }

    void attachObject(const std::string &object_id) {
        arm_->attachObject(object_id);
        RCLCPP_INFO(node_->get_logger(), "Attached: %s", object_id.c_str());
    }

    void detachObject(const std::string &object_id) {
        arm_->detachObject(object_id);
        RCLCPP_INFO(node_->get_logger(), "Detached: %s", object_id.c_str());
    }

    void goToPoseRelativeTarget(double dx, double dy, double dz, 
                                double droll, double dpitch, double dyaw, bool cartesian_path)
    {
        geometry_msgs::msg::Pose current_pose = arm_->getCurrentPose().pose;

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

        executeTarget(target_pose, cartesian_path);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path)
    {
        tf2::Quaternion q; 
        q.setRPY(roll, pitch, yaw); 
        q.normalize();
        
        geometry_msgs::msg::Pose target_pose; 
        target_pose.position.x = x; 
        target_pose.position.y = y; 
        target_pose.position.z = z; 
        target_pose.orientation = tf2::toMsg(q);
        
        executeTarget(target_pose, cartesian_path);
    }

    void executeTarget(const geometry_msgs::msg::Pose& target, bool cartesian)
    {
        arm_->setStartStateToCurrentState();
        arm_->setPoseReferenceFrame("base_link");

        if (cartesian) {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target);
            moveit_msgs::msg::RobotTrajectory trajectory;
            
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            
            if (fraction > 0.9) {
                arm_->execute(trajectory);
                RCLCPP_INFO(node_->get_logger(), "Cartesian Move Success");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Cartesian path failed (%.2f%%)", fraction * 100.0);
            }
        } 
        else {
            arm_->setPoseTarget(target);
            MoveGroupInterface::Plan plan;
            if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                arm_->execute(plan);
                RCLCPP_INFO(node_->get_logger(), "Standard Move Success");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
            }
        }
    }
    
    void openGripper() { gripper_->setNamedTarget("open"); gripper_->move(); }
    void closeGripper() { gripper_->setNamedTarget("close"); gripper_->move(); }
    void halfGripper() { gripper_->setNamedTarget("half_close"); gripper_->move(); }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    PlanningSceneInterface planning_scene_interface_;
    
    // --- 4. ADD CLIENT MEMBER ---
    rclcpp::Client<my_robot_interfaces::srv::GetPosition>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({{"use_sim_time", true}});
    
    auto node = std::make_shared<rclcpp::Node>("commander", node_options);
    auto commander = std::make_shared<Commander>(node);

    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    commander->run();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}