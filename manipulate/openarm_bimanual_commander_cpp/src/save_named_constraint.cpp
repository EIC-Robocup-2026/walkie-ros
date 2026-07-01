// RViz's MotionPlanning panel can only *select* a named path constraint from
// the warehouse (Planning tab combo box) -- it has no UI to create one. This
// is the missing other half: a CLI tool that builds a single-link
// OrientationConstraint and saves it under a name, so it shows up in that
// dropdown (and can be loaded by commander_template.cpp via constraint_name).
//
// Usage:
//   ros2 run openarm_bimanual_commander_cpp save_named_constraint
//     <name> <group_name(s), comma-separated> <link_name> <roll> <pitch> <yaw>
//     [tolerance_rad] [robot_name]
//     --ros-args -p warehouse_plugin:=warehouse_ros_sqlite::DatabaseConnection
//                -p warehouse_host:=/home/ryu/.ros/openarm_constraints.sqlite
//
// group_name must match whatever planning group is selected in RViz's
// Planning tab when you look for it: MoveGroupInterface::getKnownConstraints()
// (which backs the Path Constraints dropdown) filters by both robot name and
// the currently-active group, so a constraint saved under "left_arm" will not
// show up while "both_arms" is selected. Pass a comma-separated list (e.g.
// "left_arm,left_arm_lift,both_arms,both_arms_lift") to save it under several
// groups at once.
#include <rclcpp/rclcpp.hpp>
#include <moveit/warehouse/constraints_storage.hpp>
#include <warehouse_ros/database_loader.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <sstream>
#include <vector>

namespace
{
std::vector<std::string> splitComma(const std::string & s)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}
}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.size() < 7) {
        std::cerr << "Usage: save_named_constraint <name> <group_name(s), comma-separated> "
                     "<link_name> <roll> <pitch> <yaw> [tolerance_rad=0.2] [robot_name=walkie_bot] "
                     "--ros-args "
                     "-p warehouse_plugin:=warehouse_ros_sqlite::DatabaseConnection "
                     "-p warehouse_host:=<path-to-sqlite-file>\n";
        rclcpp::shutdown();
        return 1;
    }

    const std::string name = args[1];
    const std::vector<std::string> group_names = splitComma(args[2]);
    const std::string link_name = args[3];
    const double roll = std::stod(args[4]);
    const double pitch = std::stod(args[5]);
    const double yaw = std::stod(args[6]);
    const double tolerance = args.size() > 7 ? std::stod(args[7]) : 0.2;
    const std::string robot_name = args.size() > 8 ? args[8] : "walkie_bot";

    auto node = rclcpp::Node::make_shared("save_named_constraint");
    node->declare_parameter<std::string>("warehouse_plugin", "");
    node->declare_parameter<std::string>("warehouse_host", "");
    node->declare_parameter<int>("warehouse_port", 0);

    warehouse_ros::DatabaseLoader loader(node);
    warehouse_ros::DatabaseConnection::Ptr conn = loader.loadDatabase();
    if (!conn->connect()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to warehouse (check warehouse_plugin/warehouse_host)");
        rclcpp::shutdown();
        return 1;
    }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    moveit_msgs::msg::Constraints constraints;
    constraints.name = name;

    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "base_footprint";
    oc.link_name = link_name;
    oc.orientation = tf2::toMsg(q);
    oc.absolute_x_axis_tolerance = tolerance;
    oc.absolute_y_axis_tolerance = tolerance;
    oc.absolute_z_axis_tolerance = tolerance;
    oc.weight = 1.0;
    constraints.orientation_constraints.push_back(oc);

    moveit_warehouse::ConstraintsStorage storage(conn);
    for (const auto & group_name : group_names) {
        storage.addConstraints(constraints, robot_name, group_name);
        RCLCPP_INFO(node->get_logger(),
                    "Saved constraint '%s' for link '%s' under robot '%s' group '%s' "
                    "(roll=%.3f pitch=%.3f yaw=%.3f, tol=%.3f rad)",
                    name.c_str(), link_name.c_str(), robot_name.c_str(), group_name.c_str(),
                    roll, pitch, yaw, tolerance);
    }

    rclcpp::shutdown();
    return 0;
}
