// pick_and_place_isaac2.cpp
// Uses the bimanual_commander action servers to perform a pick-and-place task.
// Runs in parallel with commander_template.cpp (the action server).
// Mock variant: object positions are ROS parameters instead of Isaac TF frames.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <my_robot_interfaces/action/go_to_pose.hpp>
#include <my_robot_interfaces/action/set_joint_position.hpp>
#include <my_robot_interfaces/action/go_to_home.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <thread>
#include <chrono>

using GoToPose         = my_robot_interfaces::action::GoToPose;
using SetJointPosition = my_robot_interfaces::action::SetJointPosition;
using GoToHome         = my_robot_interfaces::action::GoToHome;
using namespace std::chrono_literals;

// ── tunables ───────────────────────────── ─────────────────────────────────
static constexpr double GRIPPER_OPEN   = 0.044;   // metres finger gap (control_gripper)
static constexpr double GRIPPER_CLOSE  = 0.01;

// ── table collision dimensions (metres) ───────────────────────────────────
// Cube  = pick table (SM_BottleA sits on top)
static constexpr double CUBE_SIZE_X  = 0.40;
static constexpr double CUBE_SIZE_Y  = 0.40;
static constexpr double CUBE_SIZE_Z  = 0.70;   

static constexpr double CUBE02_SIZE_X = 0.35;
static constexpr double CUBE02_SIZE_Y = 0.50;
static constexpr double CUBE02_SIZE_Z = 0.65;

static constexpr double CUBE03_SIZE_X = 0.35;
static constexpr double CUBE03_SIZE_Y = 0.50;
static constexpr double CUBE03_SIZE_Z = 0.65;
// ──────────────────────────────────────────────────────────────────────────

class PickAndPlaceMock : public rclcpp::Node
{
public:
    PickAndPlaceMock(const rclcpp::NodeOptions & options)
    : Node("pick_and_place_mock", options)
    {
        // Declare mock object positions (metres, in base_footprint frame).
        // Override at runtime: ros2 run ... --ros-args -p bottle_x:=0.6
        this->declare_parameter("bottle_x",  0.54);
        this->declare_parameter("bottle_y",  0.056);
        this->declare_parameter("bottle_z",  0.75);
        this->declare_parameter("cube_x",    0.617);
        this->declare_parameter("cube_y",    0.0);
        this->declare_parameter("cube_z",    0.325);
        this->declare_parameter("cube02_x",  0.083);
        this->declare_parameter("cube02_y",  0.7255);
        this->declare_parameter("cube02_z",  0.30);
        this->declare_parameter("cube03_x",  0.083);
        this->declare_parameter("cube03_y", -0.725);
        this->declare_parameter("cube03_z",  0.30);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_client_    = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");
        gripper_client_ = rclcpp_action::create_client<SetJointPosition>(this, "set_joint_position");
        home_client_    = rclcpp_action::create_client<GoToHome>(this, "go_to_home");

        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pick_and_place/markers", rclcpp::QoS(10));

        // Republish markers at 1 Hz so RViz always receives them regardless of startup order.
        marker_timer_ = this->create_wall_timer(1s, [this]() {
            if (!cached_markers_.markers.empty()) {
                marker_pub_->publish(cached_markers_);
            }
        });
    }

    // ── collision helpers ─────────────────────────────────────────────────

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
        pose.position.z    = cz;
        pose.orientation.w = 1.0;

        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(pose);
        return obj;
    }

    void addTableCollisions(const std::string & base_frame,
                            double cx,  double cy,  double cz,
                            double c2x, double c2y, double c2z,
                            double c3x, double c3y, double c3z)
    {
        auto start_table = makeTableBox("table_start", base_frame,
                                        cx,  cy,  cz,
                                        CUBE_SIZE_X, CUBE_SIZE_Y, CUBE_SIZE_Z);

        auto table_2 = makeTableBox("table_2", base_frame,
                                        c2x, c2y, c2z,
                                        CUBE02_SIZE_X, CUBE02_SIZE_Y, CUBE02_SIZE_Z);

        auto table_3 = makeTableBox("table_3", base_frame,
                                        c3x, c3y, c3z,
                                        CUBE03_SIZE_X, CUBE03_SIZE_Y, CUBE03_SIZE_Z);

        planning_scene_->applyCollisionObjects({start_table, table_2, table_3});
        RCLCPP_INFO(this->get_logger(), "Added collision objects from mock parameters.");
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

    // Single attempt — returns true on success, false on planning/execution failure.
    bool moveToOnce(const std::string & group,
                    double x, double y, double z,
                    double roll, double pitch, double yaw,
                    bool cartesian, const std::string & label)
    {
        if (!pose_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "go_to_pose server not available");
            return false;
        }
        GoToPose::Goal goal;
        goal.group_name     = group;
        goal.x = x; goal.y = y; goal.z = z;
        goal.roll = roll; goal.pitch = pitch; goal.yaw = yaw;
        goal.cartesian_path = cartesian;

        auto future = pose_client_->async_send_goal(goal);
        if (future.wait_for(5s) != std::future_status::ready) return false;
        auto gh = future.get();
        if (!gh) return false;
        auto result_future = pose_client_->async_get_result(gh);
        if (result_future.wait_for(30s) != std::future_status::ready) return false;
        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
    }

    // Retries up to MAX_RETRIES times — OMPL is probabilistic and can fail randomly.
    bool moveTo(const std::string & group,
                double x, double y, double z,
                double roll, double pitch, double yaw,
                bool cartesian = false, const std::string & label = "",
                int max_retries = 5)
    {
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            if (moveToOnce(group, x, y, z, roll, pitch, yaw, cartesian, label)) {
                RCLCPP_INFO(this->get_logger(), "[%s] Done (attempt %d).", label.c_str(), attempt);
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "[%s] Attempt %d/%d failed, retrying...",
                        label.c_str(), attempt, max_retries);
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_ERROR(this->get_logger(), "[%s] Failed after %d attempts.", label.c_str(), max_retries);
        return false;
    }

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

    // Move arm to the "hands_up" named state (joint4=2.0, rest=0).
    // Move arm to hands_up named state (joint4=2.0, rest=0). Retries on failure.
    bool goHandsUp(const std::string & arm, int max_retries = 5)
    {
        if (!gripper_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "[hands_up] server not available");
            return false;
        }
        SetJointPosition::Goal goal;
        goal.group_name      = arm;
        goal.joint_positions = {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};

        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            RCLCPP_INFO(this->get_logger(), "[hands_up] attempt %d/%d for %s",
                        attempt, max_retries, arm.c_str());
            auto future = gripper_client_->async_send_goal(goal);
            if (future.wait_for(5s) != std::future_status::ready) continue;
            auto gh = future.get();
            if (!gh) {
                RCLCPP_ERROR(this->get_logger(), "[hands_up] goal rejected (joint count mismatch?)");
                return false;  // no point retrying a rejection
            }
            auto rf = gripper_client_->async_get_result(gh);
            if (rf.wait_for(30s) != std::future_status::ready) continue;
            auto res = rf.get();
            if (res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result->success) {
                RCLCPP_INFO(this->get_logger(), "[hands_up] done (%s)", arm.c_str());
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "[hands_up] attempt %d failed: %s",
                        attempt, res.result->status.c_str());
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_ERROR(this->get_logger(), "[hands_up] failed after %d attempts", max_retries);
        return false;
    }

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

    // ── grasp approach helper ─────────────────────────────────────────────

    static constexpr double D_PREGRASP = 0.262;  // stand-off before contact
    static constexpr double D_GRASP    = 0.155;  // at-contact distance

    struct GraspApproach {
        double pre_x, pre_y;    // pre-grasp position (XY)
        double grasp_x, grasp_y;
        double roll;            // EEF roll for this approach direction
        double pitch;           // -π/2 (horizontal side grasp)
        bool   from_left;       // true = left-side, false = right-side
    };

    // Decide approach side by comparing bottle Y to openarm_left_link4 Y:
    //   bottle.y >= link4.y  →  bottle is on the outer/left side  → approach from LEFT  (yaw + π/2)
    //   bottle.y <  link4.y  →  bottle is on the inner/right side → approach from RIGHT (yaw - π/2)
    GraspApproach computeGraspApproach(
        double bx, double by, const std::string & base_frame)
    {
        // Look up link4 to find the arm's elbow Y position
        double link4_y = 0.0;
        try {
            auto tf = tf_buffer_->lookupTransform(
                base_frame, "openarm_left_link4", tf2::TimePointZero);
            link4_y = tf.transform.translation.y;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot lookup openarm_left_link4: %s — defaulting to left-side approach",
                        ex.what());
        }

        const double yaw_to_obj  = std::atan2(by, bx);
        const bool   from_left   = (by >= link4_y);
        const double approach_angle = from_left
            ? yaw_to_obj + M_PI / 2.0   // left side: 90° left of robot→object axis
            : yaw_to_obj - M_PI / 2.0;  // right side: 90° right of robot→object axis

        RCLCPP_INFO(this->get_logger(),
                    "Bottle Y=%.3f  link4 Y=%.3f  → %s approach  (angle=%.2f rad, roll=%.2f rad)",
                    by, link4_y,
                    from_left ? "LEFT-side" : "RIGHT-side",
                    approach_angle, approach_angle);

        GraspApproach g;
        g.pre_x    = bx + D_PREGRASP * std::cos(approach_angle);
        g.pre_y    = by + D_PREGRASP * std::sin(approach_angle);
        g.grasp_x  = bx + D_GRASP    * std::cos(approach_angle);
        g.grasp_y  = by + D_GRASP    * std::sin(approach_angle);
        g.roll     = approach_angle;
        g.pitch    = -M_PI / 2.0;
        g.from_left = from_left;
        return g;
    }

    // ── marker visualization ──────────────────────────────────────────────

    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Marker      = visualization_msgs::msg::Marker;

    // Build a sphere + text pair at (x,y,z) with a given color and label.
    // id must be unique within the array.
    void addTargetMarker(MarkerArray & arr,
                         int & id,
                         const std::string & frame,
                         double x, double y, double z,
                         float r, float g, float b,
                         const std::string & label)
    {
        auto stamp = this->get_clock()->now();

        // Sphere
        Marker sphere;
        sphere.header.frame_id = frame;
        sphere.header.stamp    = stamp;
        sphere.ns              = "pick_and_place_targets";
        sphere.id              = id++;
        sphere.type            = Marker::SPHERE;
        sphere.action          = Marker::ADD;
        sphere.pose.position.x = x;
        sphere.pose.position.y = y;
        sphere.pose.position.z = z;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04;
        sphere.color.r = r; sphere.color.g = g; sphere.color.b = b; sphere.color.a = 0.85f;
        sphere.lifetime = rclcpp::Duration::from_seconds(0);  // persistent
        arr.markers.push_back(sphere);

        // Text label floated above the sphere
        Marker text;
        text.header    = sphere.header;
        text.ns        = "pick_and_place_labels";
        text.id        = id++;
        text.type      = Marker::TEXT_VIEW_FACING;
        text.action    = Marker::ADD;
        text.pose      = sphere.pose;
        text.pose.position.z = z + 0.06;
        text.scale.z   = 0.04;
        text.color     = sphere.color;
        text.color.a   = 1.0f;
        text.text      = label;
        text.lifetime  = rclcpp::Duration::from_seconds(0);
        arr.markers.push_back(text);
    }

    // Add a thin arrow showing the approach direction (roll angle in XY plane).
    void addApproachArrow(MarkerArray & arr,
                          int & id,
                          const std::string & frame,
                          double tip_x, double tip_y, double tip_z,
                          double approach_roll,  // angle in XY plane
                          float r, float g, float b)
    {
        // Arrow from stand-off point → grasp point
        const double arrow_len = D_PREGRASP;

        Marker arrow;
        arrow.header.frame_id = frame;
        arrow.header.stamp    = this->get_clock()->now();
        arrow.ns              = "pick_and_place_arrows";
        arrow.id              = id++;
        arrow.type            = Marker::ARROW;
        arrow.action          = Marker::ADD;

        geometry_msgs::msg::Point tail, head;
        tail.x = tip_x + arrow_len * std::cos(approach_roll);
        tail.y = tip_y + arrow_len * std::sin(approach_roll);
        tail.z = tip_z;
        head.x = tip_x;
        head.y = tip_y;
        head.z = tip_z;
        arrow.points = {tail, head};

        arrow.scale.x = 0.012;  // shaft diameter
        arrow.scale.y = 0.022;  // head diameter
        arrow.scale.z = 0.04;   // head length
        arrow.color.r = r; arrow.color.g = g; arrow.color.b = b; arrow.color.a = 0.9f;
        arrow.lifetime = rclcpp::Duration::from_seconds(0);
        arr.markers.push_back(arrow);
    }

    void publishTargetMarkers(
        const std::string & frame,
        const GraspApproach & ga,
        double bx, double by, double bz,
        double c2x, double c2y, double c2z,
        double c3x, double c3y, double c3z)
    {
        MarkerArray arr;
        int id = 0;

        // ── Bottle (object to pick) ────────────────────────────────────────
        addTargetMarker(arr, id, frame, bx, by, bz,
                        1.0f, 1.0f, 1.0f, "Bottle");

        // ── Left arm pick ──────────────────────────────────────────────────
        addTargetMarker(arr, id, frame, ga.pre_x,   ga.pre_y,   bz + 0.075,
                        1.0f, 1.0f, 0.0f, "L Pre-grasp");
        addTargetMarker(arr, id, frame, ga.grasp_x, ga.grasp_y, bz + 0.075,
                        0.0f, 1.0f, 0.2f, "L Grasp");
        addTargetMarker(arr, id, frame, ga.grasp_x, ga.grasp_y, bz + 0.16,
                        0.0f, 0.8f, 1.0f, "L Lift");
        // Arrow showing approach direction
        addApproachArrow(arr, id, frame, ga.grasp_x, ga.grasp_y, bz + 0.075,
                         ga.roll, 1.0f, 1.0f, 0.0f);

        // ── Left arm place ─────────────────────────────────────────────────
        addTargetMarker(arr, id, frame, c2x, c2y - 0.285, c2z + 0.6,
                        0.2f, 0.4f, 1.0f, "L Place high");
        addTargetMarker(arr, id, frame, c2x, c2y - 0.285, c2z + 0.4,
                        0.0f, 0.2f, 1.0f, "L Place");

        // ── Right arm place ────────────────────────────────────────────────
        addTargetMarker(arr, id, frame, c3x, c3y + 0.285, c3z + 0.6,
                        0.8f, 0.0f, 0.8f, "R Place high");
        addTargetMarker(arr, id, frame, c3x, c3y + 0.285, c3z + 0.4,
                        0.6f, 0.0f, 0.6f, "R Place");

        cached_markers_ = arr;
        marker_pub_->publish(cached_markers_);
        RCLCPP_INFO(this->get_logger(), "Published %zu target markers to /pick_and_place/markers",
                    cached_markers_.markers.size());
    }

    // ── main sequence ─────────────────────────────────────────────────────

    void run()
    {
        const std::string l_arm      = "left_arm";
        const std::string l_gripper  = "left_gripper";
        const std::string r_arm      = "right_arm";
        const std::string r_gripper  = "right_gripper";
        const std::string base_frame = "base_footprint";

        // ── Step 0: Read mock object positions from parameters ───────────
        RCLCPP_INFO(this->get_logger(), "Loading mock object positions from parameters...");
        double bx  = this->get_parameter("bottle_x").as_double();
        double by  = this->get_parameter("bottle_y").as_double();
        double bz  = this->get_parameter("bottle_z").as_double();
        double cx  = this->get_parameter("cube_x").as_double();
        double cy  = this->get_parameter("cube_y").as_double();
        double cz  = this->get_parameter("cube_z").as_double();
        double c2x = this->get_parameter("cube02_x").as_double();
        double c2y = this->get_parameter("cube02_y").as_double();
        double c2z = this->get_parameter("cube02_z").as_double();
        double c3x = this->get_parameter("cube03_x").as_double();
        double c3y = this->get_parameter("cube03_y").as_double();
        double c3z = this->get_parameter("cube03_z").as_double();
        RCLCPP_INFO(this->get_logger(),
                    "Bottle=(%.2f,%.2f,%.2f)  Cube=(%.2f,%.2f,%.2f)",
                    bx, by, bz, cx, cy, cz);

        // Compute approach side based on bottle Y vs openarm_left_link4 Y
        auto ga = computeGraspApproach(bx, by, base_frame);

        publishTargetMarkers(base_frame, ga,
                             bx, by, bz,
                             c2x, c2y, c2z, c3x, c3y, c3z);

        addTableCollisions(base_frame, cx, cy, cz, c2x, c2y, c2z, c3x, c3y, c3z);

        // =================================================================
        // PHASE 1: LEFT ARM (Pick bottle, place in middle of Cube)
        // =================================================================
        RCLCPP_INFO(this->get_logger(), "--- STARTING LEFT ARM SEQUENCE ---");

        goHome(l_arm);
        sendGripper(l_gripper, GRIPPER_OPEN);

        goHandsUp(l_arm);
        std::this_thread::sleep_for(150ms);

        // 1. Pick from SM_BottleA — roll chosen based on bottle position vs link4
        moveTo(l_arm, ga.pre_x,   ga.pre_y,   bz + 0.075, ga.roll, ga.pitch, 0.0, false, "L Pre-grasp");
        std::this_thread::sleep_for(150ms);
        moveTo(l_arm, ga.grasp_x, ga.grasp_y, bz + 0.075, ga.roll, ga.pitch, 0.0, true,  "L Grasp");
        std::this_thread::sleep_for(150ms);
        sendGripper(l_gripper, GRIPPER_CLOSE);
        std::this_thread::sleep_for(500ms);
        moveTo(l_arm, ga.grasp_x, ga.grasp_y, bz + 0.16,  ga.roll, ga.pitch, 0.0, true,  "L Lift");
        std::this_thread::sleep_for(150ms);

        // Reach up with object in hand before moving to place
        goHandsUp(l_arm);
        std::this_thread::sleep_for(500ms);

        moveTo(l_arm, c2x , c2y - 0.285, c2z + 0.6,  3.14, -1.57, 1.57, false, "L Place");
        std::this_thread::sleep_for(1000ms);
        moveTo(l_arm, c2x , c2y - 0.285, c2z + 0.4,  3.14, -1.57, 1.57, false, "L Place");
        std::this_thread::sleep_for(1000ms);
        sendGripper(l_gripper, GRIPPER_OPEN);
        
        // goHome(l_arm);
        std::this_thread::sleep_for(150ms);
        goHome(l_arm);


        // =================================================================
        // PHASE 2: RIGHT ARM (same as left, but Y mirrored, place on Cube_03)
        // =================================================================
        // RCLCPP_INFO(this->get_logger(), "--- STARTING RIGHT ARM SEQUENCE ---");

        // goHome(r_arm);
        // sendGripper(r_gripper, GRIPPER_OPEN);

        // goHandsUp(r_arm);
        // std::this_thread::sleep_for(150ms);

        // // Pick from SM_BottleA (right side: Y offsets negated)
        // moveTo(r_arm, bx, by - 0.262, bz + 0.05, 3.14, -1.57, 1.57, false, "R Pre-grasp");
        // std::this_thread::sleep_for(150ms);
        // moveTo(r_arm, bx, by - 0.155, bz + 0.05, 3.14, -1.57, 1.57, true, "R Grasp");
        // std::this_thread::sleep_for(150ms);
        // sendGripper(r_gripper, GRIPPER_CLOSE);
        // std::this_thread::sleep_for(500ms);
        // moveTo(r_arm, bx, by - 0.155, bz + 0.16, 3.14, -1.57, 1.57, true, "R Lift");
        // std::this_thread::sleep_for(150ms);

        // // Reach up with object in hand
        // goHandsUp(r_arm);
        // std::this_thread::sleep_for(500ms);

        // // Place on Cube_03 (yaw negated to match right arm orientation)
        // moveTo(r_arm, c3x, c3y + 0.285, c3z + 0.6,  3.14, -1.57, -1.57, false, "R Pre-place");
        // std::this_thread::sleep_for(1000ms);
        // moveTo(r_arm, c3x, c3y + 0.285, c3z + 0.4, 3.14, -1.57, -1.57, false, "R Place");
        // std::this_thread::sleep_for(1000ms);
        // sendGripper(r_gripper, GRIPPER_OPEN);
        // std::this_thread::sleep_for(150ms);

        // goHome(r_arm);

        // RCLCPP_INFO(this->get_logger(), "Relay Pick and place complete!");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    rclcpp_action::Client<GoToPose>::SharedPtr       pose_client_;
    rclcpp_action::Client<SetJointPosition>::SharedPtr gripper_client_;
    rclcpp_action::Client<GoToHome>::SharedPtr       home_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    visualization_msgs::msg::MarkerArray cached_markers_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    auto node = std::make_shared<PickAndPlaceMock>(opts);

    std::thread spin_thread([node]() { rclcpp::spin(node); });

    node->run();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}