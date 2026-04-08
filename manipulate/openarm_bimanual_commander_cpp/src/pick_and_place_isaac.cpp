// pick_and_place_isaac.cpp
// Uses the bimanual_commander action servers to perform a pick-and-place task.
// Runs in parallel with commander_template.cpp (the action server).
//
// Actions used (served by bimanual_commander):
//   go_to_pose      – move arm to pose (RPY)
//   control_gripper – open / close gripper (position in metres)
//   go_to_home      – move arm to home named target

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <my_robot_interfaces/action/go_to_pose.hpp>
#include <my_robot_interfaces/action/set_joint_position.hpp>
#include <my_robot_interfaces/action/go_to_home.hpp>
#include <thread>
#include <chrono>

using GoToPose         = my_robot_interfaces::action::GoToPose;
using SetJointPosition = my_robot_interfaces::action::SetJointPosition;
using GoToHome         = my_robot_interfaces::action::GoToHome;
using namespace std::chrono_literals;

// ── tunables ──────────────────────────────────────────────────────────────
static constexpr double GRIPPER_OPEN   = 0.044;   // metres finger gap (control_gripper)
static constexpr double GRIPPER_CLOSE  = 0.014;
static constexpr double APPROACH_ABOVE = 0.25;   // m above target
static constexpr double GRASP_OFFSET_Z = 0.25;   // m above object centre
static constexpr double PLACE_ABOVE    = 0.15;
static constexpr double RETREAT_ABOVE  = 0.15;

// ── table collision dimensions (metres) ───────────────────────────────────
// Cube  = pick table (SM_BottleA sits on top)
static constexpr double CUBE_SIZE_X  = 0.40;
static constexpr double CUBE_SIZE_Y  = 0.50;
static constexpr double CUBE_SIZE_Z  = 0.70;   // table height

// Cube_02 = place table
static constexpr double CUBE02_SIZE_X = 0.50;
static constexpr double CUBE02_SIZE_Y = 0.50;
static constexpr double CUBE02_SIZE_Z = 0.70;
// ──────────────────────────────────────────────────────────────────────────

class PickAndPlaceIsaac : public rclcpp::Node
{
public:
    PickAndPlaceIsaac(const rclcpp::NodeOptions & options)
    : Node("pick_and_place_isaac", options)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Standard /tf listener (robot joints, base_footprint, etc.)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Isaac publishes scene objects on /isaac_tf — feed them into the same buffer
        isaac_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/isaac_tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto & tf : msg->transforms) {
                    tf_buffer_->setTransform(tf, "isaac_tf", false);
                }
            });

        pose_client_    = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");
        gripper_client_ = rclcpp_action::create_client<SetJointPosition>(this, "set_joint_position");
        home_client_    = rclcpp_action::create_client<GoToHome>(this, "go_to_home");

        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    }

    // ── collision helpers ─────────────────────────────────────────────────

    // Add a box collision object centred at (cx, cy, cz) — the raw TF position.
    moveit_msgs::msg::CollisionObject makeTableBox(
        const std::string & id,
        const std::string & frame,
        double cx, double cy, double cz,
        double sx, double sy, double sz)
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id              = id;
        obj.header.frame_id = frame;
        obj.operation       = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive box;
        box.type       = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {sx, sy, sz};

        geometry_msgs::msg::Pose pose;
        pose.position.x    = cx;
        pose.position.y    = cy;
        pose.position.z    = cz;   // Isaac TF origin is at geometric centre
        pose.orientation.w = 1.0;

        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(pose);
        return obj;
    }

    void addTableCollisions(const std::string & base_frame)
    {
        // Look up the table frames directly — their Z is the geometric centre.
        auto cube_tf   = waitForTF(base_frame, "Cube");
        auto cube02_tf = waitForTF(base_frame, "Cube_02");

        double cx  = cube_tf.transform.translation.x;
        double cy  = cube_tf.transform.translation.y;
        double cz  = cube_tf.transform.translation.z;
        double p2x = cube02_tf.transform.translation.x;
        double p2y = cube02_tf.transform.translation.y;
        double p2z = cube02_tf.transform.translation.z;

        auto pick_table  = makeTableBox("table_pick",  base_frame,
                                        cx,  cy,  cz,
                                        CUBE_SIZE_X, CUBE_SIZE_Y, CUBE_SIZE_Z);
        auto place_table = makeTableBox("table_place", base_frame,
                                        p2x, p2y, p2z,
                                        CUBE02_SIZE_X, CUBE02_SIZE_Y, CUBE02_SIZE_Z);

        planning_scene_->applyCollisionObjects({pick_table, place_table});
        RCLCPP_INFO(this->get_logger(),
                    "Added collision objects: table_pick @ (%.2f,%.2f,%.2f)  "
                    "table_place @ (%.2f,%.2f,%.2f)",
                    cx, cy, cz, p2x, p2y, p2z);
    }

    // ── utilities ────────────────────────────────────────────────────────

    geometry_msgs::msg::TransformStamped waitForTF(
        const std::string & target, const std::string & source)
    {
        while (rclcpp::ok()) {
            try {
                return tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for TF '%s': %s", source.c_str(), ex.what());
                std::this_thread::sleep_for(500ms);
            }
        }
        throw std::runtime_error("ROS shutdown while waiting for TF");
    }

    // Blocking call to go_to_pose (RPY). Works for both arm and gripper groups.
    bool moveTo(const std::string & group,
                double x, double y, double z,
                double roll, double pitch, double yaw,
                bool cartesian = false, const std::string & label = "")
    {
        if (!pose_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "go_to_pose server not available");
            return false;
        }
        GoToPose::Goal goal;
        goal.group_name    = group;
        goal.x = x; goal.y = y; goal.z = z;
        goal.roll = roll; goal.pitch = pitch; goal.yaw = yaw;
        goal.cartesian_path = cartesian;

        auto future = pose_client_->async_send_goal(goal);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Goal send timed out", label.c_str());
            return false;
        }
        auto gh = future.get();
        if (!gh) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", label.c_str());
            return false;
        }
        auto result_future = pose_client_->async_get_result(gh);
        if (result_future.wait_for(30s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Result timed out", label.c_str());
            return false;
        }
        auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Failed: %s",
                         label.c_str(), result.result->status.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[%s] Done.", label.c_str());
        return true;
    }

    // Blocking gripper open/close via set_joint_position (MoveIt joint target).
    // open=0.044, close=0.0  (openarm_left/right_finger_joint1)
    bool sendGripper(const std::string & group, double position)
    {
        if (!gripper_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "set_joint_position server not available");
            return false;
        }
        SetJointPosition::Goal goal;
        goal.group_name      = group;
        goal.joint_positions = {position};

        auto future = gripper_client_->async_send_goal(goal);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Gripper goal send timed out");
            return false;
        }
        auto gh = future.get();
        if (!gh) { RCLCPP_ERROR(this->get_logger(), "Gripper goal rejected"); return false; }

        auto result_future = gripper_client_->async_get_result(gh);
        if (result_future.wait_for(10s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Gripper result timed out");
            return false;
        }
        auto result = result_future.get();
        return (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success);
    }

    // Blocking call to go_to_home action
    bool goHome(const std::string & group)
    {
        if (!home_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "go_to_home server not available");
            return false;
        }
        GoToHome::Goal goal;
        goal.group_name = group;

        auto future = home_client_->async_send_goal(goal);
        if (future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Home goal send timed out");
            return false;
        }
        auto gh = future.get();
        if (!gh) { RCLCPP_ERROR(this->get_logger(), "Home goal rejected"); return false; }

        auto result_future = home_client_->async_get_result(gh);
        if (result_future.wait_for(30s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Home result timed out");
            return false;
        }
        return (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    // ── main sequence ─────────────────────────────────────────────────────

    void run()
    {
        const std::string arm          = "left_arm";
        const std::string gripper      = "left_gripper";
        const std::string base_frame   = "base_footprint";   // Isaac TF root frame
        const std::string bottle_frame = "SM_BottleA";
        const std::string place_frame  = "Cube_02";

        // Orientation: tool pointing straight down — RPY = (π, 0, 0)
        const double R = M_PI, P = 0.0, Y = 0.0;

        // ── Step 0: wait for TF frames ───────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Waiting for TF frames...");
        auto bottle_tf = waitForTF(base_frame, bottle_frame);
        auto place_tf  = waitForTF(base_frame, place_frame);

        double bx = bottle_tf.transform.translation.x;
        double by = bottle_tf.transform.translation.y;
        double bz = bottle_tf.transform.translation.z;
        double px = place_tf.transform.translation.x;
        double py = place_tf.transform.translation.y;
        double pz = place_tf.transform.translation.z;

        RCLCPP_INFO(this->get_logger(), "SM_BottleA @ [%.3f, %.3f, %.3f]", bx, by, bz);
        RCLCPP_INFO(this->get_logger(), "Cube_02    @ [%.3f, %.3f, %.3f]", px, py, pz);

        // ── Step 0b: Register table collision objects ────────────────────
        addTableCollisions(base_frame);

        // ── Step 1: Open gripper ─────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        if (!sendGripper(gripper, GRIPPER_OPEN))
            RCLCPP_WARN(this->get_logger(), "Gripper open failed — continuing");
        std::this_thread::sleep_for(150ms);

        // ── Step 2: Home ─────────────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Moving to home...");
        if (!goHome(arm)) return;
        std::this_thread::sleep_for(150ms);

        // ── Step 3: Pre-grasp (above bottle) ────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Moving to pre-grasp...");
        if (!moveTo(arm, bx - 0.23, by, bz + 0.1, R, P - 1.57, Y, false, "pre-grasp")) return;
        std::this_thread::sleep_for(150ms);

        // ── Step 4: Lower to grasp pose ──────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Lowering to grasp...");
        if (!moveTo(arm, bx - 0.17, by, bz + 0.1, R, P - 1.57, Y, false, "grasp")) return;
        std::this_thread::sleep_for(150ms);

        // ── Step 5: Close gripper ────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Closing gripper...");
        if (!sendGripper(gripper, GRIPPER_CLOSE))
            RCLCPP_WARN(this->get_logger(), "Gripper close failed — continuing");
        std::this_thread::sleep_for(800ms);

        // ── Step 6: Lift ─────────────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Lifting...");
        if (!moveTo(arm, bx - 0.245, by, bz + 0.11, R, P - 1.57, Y, true, "lift")) return;
        std::this_thread::sleep_for(150ms);

        // ── Step 7: Pre-place (above Cube_02) ────────────────────────────

        RCLCPP_INFO(this->get_logger(), "Moving to pre-place...");
        if (!moveTo(arm, 0.079, 0.375, 0.855,-2.967, -1.570, 1.327, false, "pre-place")) return;
        std::this_thread::sleep_for(150ms);

        RCLCPP_INFO(this->get_logger(), "Moving to place...");
        if (!moveTo(arm, 0.079, 0.375, 0.75,-2.967, -1.570, 1.327, true, "place")) return;
        std::this_thread::sleep_for(1000ms);

        // // ── Step 8: Lower to place height ────────────────────────────────
        // RCLCPP_INFO(this->get_logger(), "Lowering to place...");
        // if (!moveTo(arm, 0.079, 0.375, 0.705,-2.967, -1.570, 1.327, true, "place")) return;
        // std::this_thread::sleep_for(150ms);

        // ── Step 9: Open gripper (release) ───────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        if (!sendGripper(gripper, GRIPPER_OPEN))
            RCLCPP_WARN(this->get_logger(), "Gripper open failed — continuing");
        std::this_thread::sleep_for(1000ms);

        RCLCPP_INFO(this->get_logger(), "Moving to pre-place...");
        if (!moveTo(arm, 0.079, 0.275, 0.705,-2.967, -1.570, 1.327, true, "post-place")) return;
        std::this_thread::sleep_for(150ms);

        // // ── Step 10: Retreat ──────────────────────────────────────────────
        // RCLCPP_INFO(this->get_logger(), "Retreating...");
        // moveTo(arm, px, py, pz + RETREAT_ABOVE, R, P, Y, true, "retreat");

        // ── Step 11: Return home ──────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(), "Moving to home...");
        if (!goHome(arm)) return;
        std::this_thread::sleep_for(150ms);

        


        RCLCPP_INFO(this->get_logger(), "Pick and place complete!");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr isaac_tf_sub_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    rclcpp_action::Client<GoToPose>::SharedPtr       pose_client_;
    rclcpp_action::Client<SetJointPosition>::SharedPtr gripper_client_;
    rclcpp_action::Client<GoToHome>::SharedPtr       home_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({{"use_sim_time", true}});
    auto node = std::make_shared<PickAndPlaceIsaac>(opts);

    // Spin in background so TF / action callbacks work
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    node->run();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
