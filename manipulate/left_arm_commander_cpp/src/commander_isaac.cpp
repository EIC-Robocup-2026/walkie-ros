#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <thread>
#include <chrono>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

class CommanderIsaac : public rclcpp::Node
{
public:
    CommanderIsaac(const rclcpp::NodeOptions & options) 
    : Node("commander_isaac", options)
    {
        // 1. Initialize TF Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 2. Marker Publisher for Debugging
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        
        RCLCPP_INFO(this->get_logger(), "Commander Isaac initialized.");
    }

    void publishMarker(double x, double y, double z, std::string frame_id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = "target_debug";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
        marker.color.r = 1.0f; marker.color.a = 1.0; // RED
        
        marker_pub_->publish(marker);
    }

    void run()
    {
        // CONFIGURATION
        std::string isaac_frame = "base_footprint"; // Isaac's Frame Name
        std::string robot_frame = "base_footprint";     // MoveIt's Frame Name
        std::string object_name = "Cube";          // Object Name

        // 1. Find Object
        geometry_msgs::msg::Transform object_pose;
        
        RCLCPP_INFO(this->get_logger(), "Looking for transform: %s -> %s", object_name.c_str(), isaac_frame.c_str());

        while (rclcpp::ok())
        {
            try {
                // Get transform relative to robot base frame names
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    isaac_frame, object_name, tf2::TimePointZero);
                object_pose = t.transform;
                
                RCLCPP_INFO(this->get_logger(), "Found Object: [%.3f, %.3f, %.3f]", 
                    object_pose.translation.x, object_pose.translation.y, object_pose.translation.z);
                break; 
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Waiting for TF... (%s)", ex.what());
                std::this_thread::sleep_for(1000ms);
            }
        }

        if (!rclcpp::ok()) return;

        // 2. Initialize MoveIt
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");
        auto left_arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "left_arm");
        
        left_arm->setPoseReferenceFrame(robot_frame); 
        left_arm->setMaxVelocityScalingFactor(0.5);
        left_arm->setMaxAccelerationScalingFactor(0.5);
        left_arm->setGoalPositionTolerance(0.05);  
        left_arm->setPlanningTime(10.0); // Allow more time to calculate

        // 3. Prepare Target (NO SAFETY OFFSET)
        double target_x = object_pose.translation.x;
        double target_y = object_pose.translation.y;
        double target_z = object_pose.translation.z; // <--- RAW Z VALUE

        // Visualize Target
        publishMarker(target_x, target_y, target_z, robot_frame);
        
        // 4. Execute Move
        RCLCPP_INFO(this->get_logger(), "Moving to Home...");
        left_arm->setNamedTarget("home");
        RCLCPP_INFO(this->get_logger(), "Sending command (Async)...");
        left_arm->asyncMove(); 
        
        // Manual Wait - We assume it takes 3 seconds
        RCLCPP_INFO(this->get_logger(), "Command sent. Waiting 3s manually...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        RCLCPP_INFO(this->get_logger(), "Sequence 'Complete' (Ignored Feedback)");
        std::this_thread::sleep_for(1000ms);

        RCLCPP_INFO(this->get_logger(), "Moving to EXACT Target: [%.3f, %.3f, %.3f]", target_x, target_y, target_z);
        
        tf2::Quaternion q; 
        q.setRPY(-1.57, 0.0, 1.57); // Roll=0, Pitch=90deg, Yaw=0
        q.normalize();

        geometry_msgs::msg::Pose target_pose; 
        target_pose.position.x = target_x; 
        target_pose.position.y = target_y; 
        target_pose.position.z = target_z; 
        target_pose.orientation = tf2::toMsg(q);

        left_arm->setPoseTarget(target_pose);
        
        // Plan first to check validity
        MoveGroupInterface::Plan plan;
        if (left_arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            left_arm->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Motion Complete.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning Failed! Target is likely colliding with the floor.");
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;     
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommanderIsaac>(rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    node->run();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}