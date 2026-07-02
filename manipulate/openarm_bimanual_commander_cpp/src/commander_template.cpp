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
#include <my_robot_interfaces/srv/get_ee_pose.hpp>
#include <my_robot_interfaces/srv/get_joint_states.hpp>
#include <my_robot_interfaces/srv/toggle_gripper_collision.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/link_padding.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cmath>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <mutex>
#include <cstdint>
#include <map>
#include <set>
#include <vector>
#include <algorithm>

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
using GetEEPose = my_robot_interfaces::srv::GetEEPose;
using GetJointStates = my_robot_interfaces::srv::GetJointStates;
using ToggleGripperCollision = my_robot_interfaces::srv::ToggleGripperCollision;

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

        // OMPL planner selection. The arm groups list RRTConnect/RRT/RRTstar/
        // BKPIECE/BITstar/TRRT in ompl_planning.yaml. RRTConnect (the default
        // here) early-exits as soon
        // as it finds a path, so it plans fast. RRTstar is asymptotically optimal
        // (shorter, smoother paths) but never early-exits — it burns the FULL
        // planning-time budget every call, so it needs a larger arm_planning_time
        // to converge. Both are ROS params so you can switch/retune live.
        std::string planner_id = node_->declare_parameter<std::string>("planner_id", "RRTConnect");
        double arm_planning_time = node_->declare_parameter<double>("arm_planning_time", 5.0);
        plan_only_ = node_->declare_parameter<bool>("plan_only", false);
        node_->declare_parameter<std::string>("execute_plan_group", "");

        // Grippers keep the default planner (RRTstar isn't configured for the
        // gripper groups) and the short 1 s budget.
        for (auto &grp : {left_arm_, right_arm_, left_arm_lift_, right_arm_lift_,
                          both_arms_, both_arms_lift_})
        {
            grp->setPlannerId(planner_id);
        }

        left_arm_->setPlanningTime(arm_planning_time);
        right_arm_->setPlanningTime(arm_planning_time);
        left_gripper_->setPlanningTime(1.0);
        right_gripper_->setPlanningTime(1.0);
        left_arm_lift_->setPlanningTime(arm_planning_time);
        right_arm_lift_->setPlanningTime(arm_planning_time);
        both_arms_->setPlanningTime(arm_planning_time);
        both_arms_lift_->setPlanningTime(arm_planning_time);

        RCLCPP_INFO(node_->get_logger(),
                    "Arm groups using planner '%s' with %.1f s planning time",
                    planner_id.c_str(), arm_planning_time);

        left_arm_->setNumPlanningAttempts(10000);
        right_arm_->setNumPlanningAttempts(10000);
        left_gripper_->setNumPlanningAttempts(100);
        right_gripper_->setNumPlanningAttempts(100);
        left_arm_lift_->setNumPlanningAttempts(10000);
        right_arm_lift_->setNumPlanningAttempts(10000);
        both_arms_->setNumPlanningAttempts(10000);
        both_arms_lift_->setNumPlanningAttempts(10000);

        // Software gripper speed cap (command-position units per second).
        // GripperActionController has no velocity limit, so controlGripper()
        // ramps the setpoint at this rate. Lower = slower/gentler; <= 0
        // disables the ramp (commands jump straight to the target). Tune live
        // with the ROS param without recompiling.
        gripper_speed_ = node_->declare_parameter<double>("gripper_speed", 1.0);

        // Resting/open gripper command at startup, in GripperCommand position
        // units (NOT joint_states units — those use a different scale). Seeds
        // last_gripper_cmd_ so the *first* command after launch also ramps
        // instead of jumping. Set to wherever the grippers actually rest.
        double gripper_init = node_->declare_parameter<double>("gripper_init_pos", 0.0);
        last_gripper_cmd_["left_gripper"]  = gripper_init;
        last_gripper_cmd_["right_gripper"] = gripper_init;

        // 3. Gripper Action Clients (GripperActionController)
        left_gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            node_, "/left_gripper_controller/gripper_cmd");
        right_gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            node_, "/right_gripper_controller/gripper_cmd");

        // Cache the finger positions straight from /joint_states. This is the
        // ramp start used by controlGripper(); reading it here (instead of
        // MoveGroupInterface::getCurrentState) is always-fresh, thread-safe, and
        // — crucially — reliable, so an open/close always ramps from the real
        // position instead of occasionally falling back to a seed that triggers
        // the full-speed direct path. SensorDataQoS (best-effort) is compatible
        // with both reliable and best-effort joint_states publishers.
        joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(),
            std::bind(&Commander::jointStatesCb, this, _1), sub_opt);

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

        get_joint_states_service_ = node_->create_service<GetJointStates>(
            "get_joint_states",
            std::bind(&Commander::handle_get_joint_states, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        get_ee_pose_service_ = node_->create_service<GetEEPose>(
            "get_ee_pose",
            std::bind(&Commander::handle_get_ee_pose, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        // ToggleGripperCollision: runtime enable/disable of collision checking
        // for a gripper's links against the rest of the world (octomap, scene
        // objects, other robot links). Implemented by flipping the per-link
        // *default* entry in the AllowedCollisionMatrix, which leaves SRDF
        // adjacency and finger-vs-object touch_link entries intact (those are
        // specific entries and take precedence over the default). Reads the
        // current ACM via /get_planning_scene and writes it back via
        // /apply_planning_scene as a diff.
        toggle_gripper_collision_service_ = node_->create_service<ToggleGripperCollision>(
            "toggle_gripper_collision",
            std::bind(&Commander::handle_toggle_gripper_collision, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);
        get_planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene", rclcpp::ServicesQoS(), callback_group_);
        apply_planning_scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
            "/apply_planning_scene", rclcpp::ServicesQoS(), callback_group_);

        // ClearCollisionObjects: detach anything held by the grippers and remove
        // every world collision object from the planning scene. Trigger service
        // (no request fields); returns success + a count message.
        clear_collision_objects_service_ = node_->create_service<std_srvs::srv::Trigger>(
            "clear_collision_objects",
            std::bind(&Commander::handle_clear_collision_objects, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        execute_stored_plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
            "execute_stored_plan",
            std::bind(&Commander::handle_execute_stored_plan, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        // ToggleOctomapCollision: select whether planning USES the octomap.
        // data=true  -> use it (collision checked, normal);
        // data=false -> ignore it (planning plans through sensed voxels).
        // Works by flipping the "<octomap>" default entry in the ACM, like the
        // gripper toggle. The octomap keeps being built from the point cloud;
        // this only controls whether collision checking considers it.
        toggle_octomap_collision_service_ = node_->create_service<std_srvs::srv::SetBool>(
            "toggle_octomap_collision",
            std::bind(&Commander::handle_toggle_octomap_collision, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        // ToggleAllCollisionChecking: blanket enable/disable of ALL collision
        // checking during planning (self-collision between every robot link,
        // vs world objects, vs octomap). data=false -> plan straight through
        // everything; data=true -> restore normal checking. Off by default —
        // this is a blunt debugging/demo switch, not something to leave on.
        toggle_all_collision_service_ = node_->create_service<std_srvs::srv::SetBool>(
            "toggle_all_collision_checking",
            std::bind(&Commander::handle_toggle_all_collision, this, _1, _2),
            rclcpp::ServicesQoS(),
            callback_group_);

        // 5. Grasp attach/detach
        // When the gripper closes past grasp_close_threshold we attach a box to
        // the hand (touch_links = the fingers) so MoveIt: (a) ignores ONLY
        // finger-vs-object contact, keeping the outer faces strict against the
        // world, and (b) masks the box out of the octomap, so the sensed object
        // stops generating voxels that block planning. Opening detaches it.
        // All knobs are ROS params so they can be tuned without recompiling.
        grasp_attach_enable_   = node_->declare_parameter<bool>("grasp_attach_enable", true);
        grasp_close_threshold_ = node_->declare_parameter<double>("grasp_close_threshold", 0.03);

        // Grasp-success detection: after a close, the settled finger gap tells us
        // whether something is held. Empty close -> gap ~ 0 (fingers meet); a held
        // object stalls the fingers apart at its width; open -> gap ~ max. So an
        // object is "grasped" iff the settled gap lands in [min, max]. Read live;
        // CALIBRATE on the robot: close on nothing (record gap), close on the
        // bottle (record gap), set the band to bracket the bottle above empty.
        node_->declare_parameter<bool>("grasp_detect_enable", true);
        node_->declare_parameter<double>("grasp_detect_min", 0.005);   // m, above empty-closed
        node_->declare_parameter<double>("grasp_detect_max", 0.044);   // m, below fully-open (~0.044)
        // Tells the commander what an open/close MEANS for the planning-scene box.
        // Read fresh on every gripper command, so you can flip it live with
        // `ros2 param set <commander_node> grasp_scene_action place` right before
        // sending an open. Values:
        //   grasp - attach the box (picking up)
        //   place - detach + remove after the next motion (placing/releasing)
        //   none  - leave the scene untouched (just open/close, nothing held)
        // Default "none": the scene is only touched when you explicitly ask.
        node_->declare_parameter<std::string>("grasp_scene_action", "none");
        // Oversized box: covers the gripper's full aperture (~0.088 m) plus an
        // object body extending ~0.22 m past the fingertips, so it reliably
        // encloses (and octomap-masks) any object we grasp regardless of shape.
        // The trade-off is that it also masks the octomap in this region — keep
        // it just large enough for the biggest object you pick. Tune live.
        grasp_object_size_     = node_->declare_parameter<std::vector<double>>(
            "grasp_object_size", {0.09, 0.05, 0.16});      // box dims (m) in hand frame
        grasp_object_offset_   = node_->declare_parameter<std::vector<double>>(
            "grasp_object_offset", {0.0, 0.0, 0.065});      // box centre (m) in hand frame
        // Box orientation (roll,pitch,yaw in rad) in the hand frame. The box's
        // long axis is its local z; with identity it points along the gripper's
        // approach/drop direction (hand z). A 90° pitch swings that long axis
        // onto hand x so it runs ALONG the grasped object (e.g. an upright
        // bottle held in a side grasp) instead of along the approach. Tune live.
        grasp_object_rpy_      = node_->declare_parameter<std::vector<double>>(
            "grasp_object_rpy", {0.0, 1.5708, 0.0});       // box orient (rad) in hand frame

        // Object scene height, split about the grasp centre, measured ALONG the
        // box's long axis (its local z — after grasp_object_rpy this runs along
        // the grasped object body). These let you size the attached object by how
        // far it sticks out past the gripper centre instead of via the symmetric
        // grasp_object_size_[2]: total long-axis length = above + below, and the
        // box is shifted so the grasp centre sits `below` from the bottom face and
        // `above` from the top. Feed perception's height_above_grasp /
        // height_below_grasp straight in. Both 0 (default) = keep the old
        // grasp_object_size_/offset behaviour. Tunable live.
        grasp_object_height_above_ = node_->declare_parameter<double>(
            "grasp_object_height_above", 0.0);   // m, grasp centre → object top
        grasp_object_height_below_ = node_->declare_parameter<double>(
            "grasp_object_height_below", 0.0);   // m, grasp centre → object bottom

        // Extra clearance added on EACH side of the box ONLY while it is ATTACHED
        // to the gripper. So the carried box = real object + 2*margin per axis
        // (bigger, for clearance while moving), and on release it is deflated by
        // the same amount back to the real grasp size left in the world. 0 = off.
        node_->declare_parameter<double>("attach_object_margin", 0.02);   // m per side

        // 4b. Finger collision padding.
        // The table is only sensed as octomap voxels, and planned paths can graze
        // the surface (or overshoot it during execution), so the fingertips clip
        // the table. Inflating the finger links' collision geometry by a fixed
        // margin makes MoveIt keep that much clearance from any sensed voxel.
        // Applied to move_group's scene as a PlanningScene diff (link_padding),
        // so it affects all server-side planning. Tunable live via params.
        finger_padding_enable_ = node_->declare_parameter<bool>("finger_padding_enable", true);
        finger_padding_        = node_->declare_parameter<double>("finger_padding", 0.01);  // m
        finger_pad_links_      = node_->declare_parameter<std::vector<std::string>>(
            "finger_pad_links",
            {"openarm_left_left_finger",  "openarm_left_right_finger",
             "openarm_right_left_finger", "openarm_right_right_finger"});

        // 4c. Explicit table collision box (B). The table can't be trusted to the
        // octomap during a grasp (occluded + masked by the attached box), so add
        // it as an explicit CollisionObject the gripper/object plan around. The
        // box spans the floor (frame z=0) up to table_pose z (the live height).
        // Disabled by default — configure for your table:
        //   ros2 param set /bimanual_commander table_enable true
        //   ros2 param set /bimanual_commander table_pose "[0.6, 0.0, 0.75, 0.0]"
        node_->declare_parameter<bool>("table_enable", false);
        node_->declare_parameter<std::string>("table_frame", "base_footprint");
        node_->declare_parameter<std::vector<double>>("table_pose", {0.65, 0.0, 0.75, 0.0});  // x,y,top_z,yaw
        node_->declare_parameter<std::vector<double>>("table_size", {0.60, 1.20});            // footprint x,y

        // (A) Allow ONLY the gripper links vs the octomap, so grasping inside
        // sensed voxels no longer fails to plan — WITHOUT blinding the fingers to
        // explicit collision objects (table/shelf) and other links, which the
        // blanket ToggleGripperCollision does. Toggle live around a grasp:
        //   ros2 param set /bimanual_commander allow_gripper_vs_octomap true
        node_->declare_parameter<bool>("allow_gripper_vs_octomap", false);

        // --- Make planner_id / arm_planning_time retune LIVE ---
        // gripper_speed and grasp_scene_action are re-read per command, but
        // planner_id / arm_planning_time persist on each MoveGroupInterface, so
        // instead we re-apply them to the arm groups whenever they change via
        // `ros2 param set <commander_node> planner_id RRTstar` (or
        // arm_planning_time). The next plan uses the new value, no restart.
        param_cb_handle_ = node_->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params)
                -> rcl_interfaces::msg::SetParametersResult
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                bool finger_changed = false;
                bool table_changed = false;
                bool octomap_changed = false;
                for (const auto & p : params) {
                    if (p.get_name() == "planner_id") {
                        for (auto & grp : {left_arm_, right_arm_, left_arm_lift_,
                                           right_arm_lift_, both_arms_, both_arms_lift_})
                            grp->setPlannerId(p.as_string());
                        RCLCPP_INFO(node_->get_logger(),
                                    "planner_id -> '%s' (live)", p.as_string().c_str());
                    } else if (p.get_name() == "arm_planning_time") {
                        for (auto & grp : {left_arm_, right_arm_, left_arm_lift_,
                                           right_arm_lift_, both_arms_, both_arms_lift_})
                            grp->setPlanningTime(p.as_double());
                        RCLCPP_INFO(node_->get_logger(),
                                    "arm_planning_time -> %.2f s (live)", p.as_double());
                    } else if (p.get_name().rfind("finger_pad", 0) == 0) {
                        finger_changed = true;   // finger_padding / _enable / _links
                    } else if (p.get_name() == "allow_gripper_vs_octomap") {
                        octomap_changed = true;
                    } else if (p.get_name().rfind("table_", 0) == 0) {
                        table_changed = true;
                    }
                }
                // Re-apply off this (pre-commit) callback thread, so the blocking
                // planning-scene service calls run after the new values commit.
                if (finger_changed)  scheduleFingerPaddingReapply();
                if (table_changed)   scheduleTableReapply();
                if (octomap_changed) scheduleGripperOctomapAllowance();
                return result;
            });

        planning_scene_ = std::make_shared<PlanningSceneInterface>();

        // Defer the padding write: the executor is not spinning yet inside the
        // constructor, so a blocking service call here would deadlock. A one-shot
        // timer (on the reentrant group) fires once move_group is up, applies the
        // padding, then cancels itself.
        if (finger_padding_enable_) {
            padding_timer_ = node_->create_wall_timer(
                std::chrono::seconds(3),
                [this]() {
                    padding_timer_->cancel();
                    applyFingerPadding();
                },
                callback_group_);
        }

        // Defer the table write (needs move_group up); reads table_enable live so
        // it's a clean no-op when disabled.
        table_timer_ = node_->create_wall_timer(
            std::chrono::seconds(3),
            [this]() {
                table_timer_->cancel();
                buildAndApplyTable();
            },
            callback_group_);

        RCLCPP_INFO(node_->get_logger(), "Bimanual Commander Node Initialized");
    }

    // Map a gripper group to its hand link and the finger links that are
    // allowed to touch the attached object. Returns false for non-gripper groups.
    bool gripperAttachInfo(const std::string & gripper_group,
                           std::string & hand_link,
                           std::vector<std::string> & touch_links,
                           std::string & object_id)
    {
        if (gripper_group == "left_gripper") {
            hand_link   = "openarm_left_hand";
            touch_links = {"openarm_left_hand", "openarm_left_left_finger",
                           "openarm_left_right_finger", "openarm_left_link7",
                           "openarm_left_wristcam_bracket"};
            object_id   = "grasped_object_left";
            return true;
        }
        if (gripper_group == "right_gripper") {
            hand_link   = "openarm_right_hand";
            touch_links = {"openarm_right_hand", "openarm_right_left_finger",
                           "openarm_right_right_finger", "openarm_right_link7",
                           "openarm_right_wristcam_bracket"};
            object_id   = "grasped_object_right";
            return true;
        }
        return false;
    }

    // Attach a box (sized/placed by params) to the gripper's hand link.
    void attachGraspObject(const std::string & gripper_group)
    {
        std::string hand_link, object_id;
        std::vector<std::string> touch_links;
        if (!gripperAttachInfo(gripper_group, hand_link, touch_links, object_id)) return;

        // Re-grasping the same id cancels any pending world-removal from a prior
        // release, so flushPendingGraspRemovals() can't delete what we just attached.
        pending_grasp_removal_.erase(object_id);

        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name           = hand_link;
        aco.touch_links         = touch_links;
        aco.object.id           = object_id;
        aco.object.header.frame_id = hand_link;
        aco.object.operation    = moveit_msgs::msg::CollisionObject::ADD;

        // Read the geometry params fresh on every attach so `ros2 param set`
        // takes effect live without restarting (the constructor values are only
        // the defaults). Matches how planning_frame is handled.
        const auto size   = node_->get_parameter("grasp_object_size").as_double_array();
        const auto offset = node_->get_parameter("grasp_object_offset").as_double_array();
        const auto rpy    = node_->get_parameter("grasp_object_rpy").as_double_array();
        const double above = node_->get_parameter("grasp_object_height_above").as_double();
        const double below = node_->get_parameter("grasp_object_height_below").as_double();

        shape_msgs::msg::SolidPrimitive box;
        box.type       = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {size[0], size[1], size[2]};

        tf2::Quaternion q;
        q.setRPY(rpy[0], rpy[1], rpy[2]);

        geometry_msgs::msg::Pose pose;
        pose.position.x    = offset[0];
        pose.position.y    = offset[1];
        pose.position.z    = offset[2];
        pose.orientation   = tf2::toMsg(q);

        // If the above/below heights are set, they define the long-axis (local z)
        // extent instead of size[2]: length = above + below, with grasp_object_offset_
        // as the grasp centre. The object extends `above` toward the local +z face
        // and `below` toward the local -z face; the box centre is the midpoint, so
        // it shifts (above - below)/2 along the box's local +z (which q maps into
        // the hand frame), starting from the offset anchor.
        if (above + below > 0.0) {
            box.dimensions[2] = above + below;
            const double shift = (above - below) / 2.0;
            tf2::Vector3 local_z = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
            pose.position.x += shift * local_z.x();
            pose.position.y += shift * local_z.y();
            pose.position.z += shift * local_z.z();
        }

        // Inflate the ATTACHED box by attach_object_margin on each side (centre
        // unchanged). The released world copy is deflated back to this real size
        // in detachGraspObject, so the carried box is bigger than the placed one.
        const double attach_margin = node_->get_parameter("attach_object_margin").as_double();
        if (attach_margin > 0.0) {
            box.dimensions[0] += 2.0 * attach_margin;
            box.dimensions[1] += 2.0 * attach_margin;
            box.dimensions[2] += 2.0 * attach_margin;
        }

        aco.object.primitives.push_back(box);
        aco.object.primitive_poses.push_back(pose);

        if (planning_scene_->applyAttachedCollisionObject(aco)) {
            RCLCPP_INFO(node_->get_logger(),
                        "Attached '%s' to %s  dims=[%.3f %.3f %.3f] above=%.3f below=%.3f",
                        object_id.c_str(), hand_link.c_str(),
                        box.dimensions[0], box.dimensions[1], box.dimensions[2],
                        above, below);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to attach '%s' (is move_group up?)",
                        object_id.c_str());
        }
    }

    // Release/open: DETACH the box from the gripper but leave it standing in the
    // world. It keeps masking the (still physically present) object's octomap
    // voxels until the arm actually moves away. The world copy is deleted later,
    // at the start of the next arm motion -- see flushPendingGraspRemovals().
    void detachGraspObject(const std::string & gripper_group)
    {
        std::string hand_link, object_id;
        std::vector<std::string> touch_links;
        if (!gripperAttachInfo(gripper_group, hand_link, touch_links, object_id)) return;

        // Nothing attached? Skip quietly.
        auto attached = planning_scene_->getAttachedObjects({object_id});
        if (attached.find(object_id) == attached.end()) return;

        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name        = hand_link;
        aco.object.id        = object_id;
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_->applyAttachedCollisionObject(aco);   // detach -> stays in world

        // The carried box was inflated by attach_object_margin; shrink the world
        // copy back by the same amount so the PLACED object is the real grasp
        // size, smaller than what we carried.
        deflateReleasedObject(object_id);

        pending_grasp_removal_.insert(object_id);             // delete after next motion succeeds
        RCLCPP_INFO(node_->get_logger(),
                    "Detached '%s' (removes from world after next motion succeeds)", object_id.c_str());
    }

    // Shrink the just-released world object's box by 2*attach_object_margin per
    // axis, undoing the attach-time inflation so the placed object is the real
    // grasp size. No-op if margin is 0 or the object isn't a single box.
    // getObjects/applyCollisionObjects are synchronous services; safe here
    // (MultiThreadedExecutor + reentrant group).
    void deflateReleasedObject(const std::string & object_id)
    {
        const double margin = node_->get_parameter("attach_object_margin").as_double();
        if (margin <= 0.0) return;
        auto objs = planning_scene_->getObjects({object_id});
        auto it = objs.find(object_id);
        if (it == objs.end() || it->second.primitives.empty() ||
            it->second.primitives[0].dimensions.size() < 3)
            return;
        moveit_msgs::msg::CollisionObject co = it->second;
        auto & d = co.primitives[0].dimensions;
        co.operation = moveit_msgs::msg::CollisionObject::ADD;   // replace geometry in place
        d[0] = std::max(0.001, d[0] - 2.0 * margin);
        d[1] = std::max(0.001, d[1] - 2.0 * margin);
        d[2] = std::max(0.001, d[2] - 2.0 * margin);
        planning_scene_->applyCollisionObjects({co});
        RCLCPP_INFO(node_->get_logger(),
            "Released '%s' deflated to real size [%.3f %.3f %.3f]",
            object_id.c_str(), d[0], d[1], d[2]);
    }

    // Delete any released grasp boxes (detached but still in the world) from the
    // scene. Called only AFTER an arm motion EXECUTES SUCCESSFULLY, so the box
    // stays present through planning and execution -- the arm plans/moves around
    // the just-released object -- and is cleared only once the arm has safely
    // moved away. applyPlanningScene is synchronous (service).
    void flushPendingGraspRemovals()
    {
        if (pending_grasp_removal_.empty()) return;

        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;
        for (const auto & id : pending_grasp_removal_) {
            moveit_msgs::msg::CollisionObject co;
            co.id        = id;
            co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            ps.world.collision_objects.push_back(co);
            RCLCPP_INFO(node_->get_logger(), "Removed '%s' from world before motion", id.c_str());
        }
        planning_scene_->applyPlanningScene(ps);
        pending_grasp_removal_.clear();
    }

    // ========== ExecuteStoredPlan Service ==========
    // Execute the plan stored by the last plan_only action for a given group.
    // Set 'execute_plan_group' param to the target group before calling.
    // The stored plan is consumed (removed) after successful execution.
    void handle_execute_stored_plan(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        const std::string group_name =
            node_->get_parameter("execute_plan_group").as_string();
        if (group_name.empty()) {
            response->success = false;
            response->message = "Set 'execute_plan_group' param to the target group name first";
            return;
        }
        auto it = stored_plans_.find(group_name);
        if (it == stored_plans_.end()) {
            response->success = false;
            response->message = "No stored plan for group '" + group_name +
                                 "' — send a goal with plan_only=true first";
            return;
        }
        auto group = getGroup(group_name);
        if (!group) {
            response->success = false;
            response->message = "Invalid group: " + group_name;
            return;
        }
        auto err = group->execute(it->second);
        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
            flushPendingGraspRemovals();
            stored_plans_.erase(it);
            response->success = true;
            response->message = "Executed stored plan for " + group_name;
            RCLCPP_INFO(node_->get_logger(), "Executed stored plan for %s", group_name.c_str());
        } else {
            response->success = false;
            response->message = "Execution failed for " + group_name +
                                 " (plan still stored — call again to retry)";
            RCLCPP_ERROR(node_->get_logger(), "Execute stored plan failed for %s", group_name.c_str());
        }
    }

    // ========== ClearCollisionObjects Service ==========
    // Detach anything attached to a gripper, then remove every world collision
    // object from the planning scene. Leaves the octomap and robot state alone.
    void handle_clear_collision_objects(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        // 1. Detach every attached object so it becomes a world object (then it
        //    gets removed in step 2). Covers grasp boxes and anything else held.
        auto attached = planning_scene_->getAttachedObjects();
        for (const auto & kv : attached) {
            moveit_msgs::msg::AttachedCollisionObject aco;
            aco.link_name        = kv.second.link_name;
            aco.object.id        = kv.first;
            aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            planning_scene_->applyAttachedCollisionObject(aco);
        }

        // 2. Remove all world collision objects.
        auto names = planning_scene_->getKnownObjectNames();
        if (!names.empty()) {
            planning_scene_->removeCollisionObjects(names);
        }

        // 3. Nothing left to flush.
        pending_grasp_removal_.clear();

        response->success = true;
        response->message = "Cleared " + std::to_string(names.size()) +
                            " world object(s), detached " +
                            std::to_string(attached.size()) + " attached";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    // ========== ToggleOctomapCollision Service ==========
    // Select whether planning uses the octomap, by flipping the "<octomap>"
    // entry in the AllowedCollisionMatrix. data=true -> collision checked (use);
    // data=false -> allowed (ignored). The octomap keeps updating either way.
    void handle_toggle_octomap_collision(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        const bool use_octomap = request->data;
        const bool allowed = !use_octomap;   // allowed == ignored in planning

        if (!get_planning_scene_client_->service_is_ready() ||
            !apply_planning_scene_client_->service_is_ready()) {
            response->success = false;
            response->message = "Planning scene services not available (is move_group up?)";
            return;
        }

        // 1. Read the current ACM.
        auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_req->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto get_future = get_planning_scene_client_->async_send_request(get_req);
        if (get_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            response->success = false;
            response->message = "Timed out reading planning scene";
            return;
        }
        auto acm = get_future.get()->scene.allowed_collision_matrix;

        // 2. Flip the octomap's default ACM entry.
        const std::string oct = "<octomap>";
        auto it = std::find(acm.default_entry_names.begin(),
                            acm.default_entry_names.end(), oct);
        if (it != acm.default_entry_names.end()) {
            acm.default_entry_values[
                std::distance(acm.default_entry_names.begin(), it)] = allowed;
        } else {
            acm.default_entry_names.push_back(oct);
            acm.default_entry_values.push_back(allowed);
        }

        // 3. Write it back as a diff.
        auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        apply_req->scene.is_diff = true;
        apply_req->scene.allowed_collision_matrix = acm;
        auto apply_future = apply_planning_scene_client_->async_send_request(apply_req);
        if (apply_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
            !apply_future.get()->success) {
            response->success = false;
            response->message = "Failed to apply ACM update";
            return;
        }

        response->success = true;
        response->message = use_octomap
            ? "Octomap ENABLED (planning uses the octomap)"
            : "Octomap DISABLED (planning ignores the octomap)";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    // ========== ToggleAllCollisionChecking Service ==========
    // Blanket version of the gripper/octomap toggles above: flips the
    // *default* ACM entry for EVERY link in the robot model, plus the
    // octomap. A one-sided default entry is enough for MoveIt's ACM to
    // allow a pair (see AllowedCollisionMatrix::getAllowedCollision), so
    // marking every robot link "always allowed" disables self-collision AND
    // robot-vs-world checks together, without needing to know world object
    // ids ahead of time. data=true -> normal checking; data=false -> plan
    // through everything.
    void handle_toggle_all_collision(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        const bool check_collisions = request->data;
        const bool allowed = !check_collisions;

        if (!get_planning_scene_client_->service_is_ready() ||
            !apply_planning_scene_client_->service_is_ready()) {
            response->success = false;
            response->message = "Planning scene services not available (is move_group up?)";
            return;
        }

        // 1. Read the current ACM.
        auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_req->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto get_future = get_planning_scene_client_->async_send_request(get_req);
        if (get_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            response->success = false;
            response->message = "Timed out reading planning scene";
            return;
        }
        auto acm = get_future.get()->scene.allowed_collision_matrix;

        // 2. Flip the default entry for every robot link, plus the octomap.
        auto link_names = left_arm_->getRobotModel()->getLinkModelNames();
        link_names.push_back("<octomap>");
        for (const auto & name : link_names) {
            auto it = std::find(acm.default_entry_names.begin(),
                                acm.default_entry_names.end(), name);
            if (it != acm.default_entry_names.end()) {
                acm.default_entry_values[
                    std::distance(acm.default_entry_names.begin(), it)] = allowed;
            } else {
                acm.default_entry_names.push_back(name);
                acm.default_entry_values.push_back(allowed);
            }
        }

        // 3. Write it back as a diff.
        auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        apply_req->scene.is_diff = true;
        apply_req->scene.allowed_collision_matrix = acm;
        auto apply_future = apply_planning_scene_client_->async_send_request(apply_req);
        if (apply_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
            !apply_future.get()->success) {
            response->success = false;
            response->message = "Failed to apply ACM update";
            return;
        }

        response->success = true;
        response->message = check_collisions
            ? "Collision checking ENABLED (normal planning)"
            : "Collision checking DISABLED (planning ignores all collisions)";
        RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
    }

    // ========== ToggleGripperCollision Service ==========
    // Enable/disable collision checking for a gripper's links against the rest
    // of the world. Works by flipping each gripper link's *default* entry in the
    // AllowedCollisionMatrix: a true default means "collision with anything is
    // allowed unless a specific entry says otherwise". SRDF adjacency and
    // finger-vs-object touch_link pairs are *specific* entries and take
    // precedence over the default, so they stay intact either way. Only the
    // gripper links are touched here — any attached grasped_object_<side> keeps
    // being collision-checked.
    void handle_toggle_gripper_collision(
        const std::shared_ptr<ToggleGripperCollision::Request> request,
        std::shared_ptr<ToggleGripperCollision::Response> response)
    {
        // Reuse the touch_link list as the gripper's link set.
        std::string hand_link, object_id;
        std::vector<std::string> gripper_links;
        if (!gripperAttachInfo(request->group_name, hand_link, gripper_links, object_id)) {
            response->success = false;
            response->status  = "Invalid gripper group: " + request->group_name;
            return;
        }

        if (!get_planning_scene_client_->service_is_ready() ||
            !apply_planning_scene_client_->service_is_ready()) {
            response->success = false;
            response->status  = "Planning scene services not available (is move_group up?)";
            return;
        }

        // 1. Read the current ACM.
        auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_req->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto get_future = get_planning_scene_client_->async_send_request(get_req);
        if (get_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            response->success = false;
            response->status  = "Timed out reading planning scene";
            return;
        }
        auto acm = get_future.get()->scene.allowed_collision_matrix;

        // 2. enable=true  -> collision checked -> default entry = false (not allowed)
        //    enable=false -> ignore vs world   -> default entry = true  (allowed)
        const bool allowed = !request->enable;
        for (const auto & link : gripper_links) {
            auto it = std::find(acm.default_entry_names.begin(),
                                acm.default_entry_names.end(), link);
            if (it != acm.default_entry_names.end()) {
                acm.default_entry_values[
                    std::distance(acm.default_entry_names.begin(), it)] = allowed;
            } else {
                acm.default_entry_names.push_back(link);
                acm.default_entry_values.push_back(allowed);
            }
        }

        // 3. Write it back as a scene diff. is_diff=true keeps the rest of the
        //    scene (objects, robot state); the supplied full ACM replaces the old.
        auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        apply_req->scene.is_diff = true;
        apply_req->scene.allowed_collision_matrix = acm;
        auto apply_future = apply_planning_scene_client_->async_send_request(apply_req);
        if (apply_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            response->success = false;
            response->status  = "Timed out applying planning scene";
            return;
        }
        if (!apply_future.get()->success) {
            response->success = false;
            response->status  = "apply_planning_scene rejected the ACM update";
            return;
        }

        response->success = true;
        response->status  = request->enable
            ? "Gripper collision ENABLED for " + request->group_name
            : "Gripper collision DISABLED for " + request->group_name;
        RCLCPP_INFO(node_->get_logger(), "%s", response->status.c_str());
    }

    // ========== Finger collision padding ==========
    // Push a PlanningScene diff that sets a fixed collision padding (m) on the
    // finger links. MoveIt inflates each padded link's collision geometry by
    // this amount, so server-side planning keeps that clearance from octomap
    // voxels (and everything else). is_diff=true leaves the rest of the scene
    // untouched. Re-callable, so params can be changed and re-applied live.
    void applyFingerPadding()
    {
        if (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(),
                "apply_planning_scene unavailable; finger padding NOT applied");
            return;
        }

        // Read fresh so `ros2 param set finger_padding/_enable/_links` takes effect
        // live. Disabled -> push padding 0.0 to clear any previously-applied margin.
        const bool enable = node_->get_parameter("finger_padding_enable").as_bool();
        const double pad  = enable ? node_->get_parameter("finger_padding").as_double() : 0.0;
        const auto links  = node_->get_parameter("finger_pad_links").as_string_array();

        auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        req->scene.is_diff = true;
        for (const auto & link : links) {
            moveit_msgs::msg::LinkPadding lp;
            lp.link_name = link;
            lp.padding   = pad;
            req->scene.link_padding.push_back(lp);
        }

        auto fut = apply_planning_scene_client_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
            !fut.get()->success) {
            RCLCPP_WARN(node_->get_logger(), "Failed to apply finger collision padding");
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
            "Applied %.3f m collision padding to %zu finger links%s",
            pad, links.size(), enable ? "" : " (DISABLED -> cleared)");
    }

    // Debounced live re-apply for the finger_padding* params: one-shot timer so
    // the blocking applyFingerPadding() runs off the parameter-set callback
    // thread, after the new values commit.
    void scheduleFingerPaddingReapply()
    {
        finger_apply_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                finger_apply_timer_->cancel();
                applyFingerPadding();
            },
            callback_group_);
    }

    // ========== (B) Explicit table collision box ==========
    // (Re)build the table from the table_* params as an explicit CollisionObject.
    // The box spans the floor (frame z=0) up to the table top (table_pose z), so
    // changing that one value retunes the height. Read fresh; serves startup +
    // live changes; removes a stale table when disabled.
    void buildAndApplyTable()
    {
        const bool enable       = node_->get_parameter("table_enable").as_bool();
        const std::string frame = node_->get_parameter("table_frame").as_string();
        const auto pose = node_->get_parameter("table_pose").as_double_array();  // x,y,top_z,yaw
        const auto size = node_->get_parameter("table_size").as_double_array();  // footprint x,y

        std::vector<moveit_msgs::msg::CollisionObject> ops;
        bool added = false;
        if (enable) {
            if (pose.size() != 4 || size.size() != 2) {
                RCLCPP_WARN(node_->get_logger(),
                    "table_enable=true but table_pose needs 4 [x,y,top_z,yaw] and "
                    "table_size needs 2 [x,y]; skipping table");
            } else if (pose[2] <= 0.0) {
                RCLCPP_WARN(node_->get_logger(),
                    "table top height (table_pose z=%.3f) must be > 0; skipping table", pose[2]);
            } else {
                const double top = pose[2], yaw = pose[3];
                moveit_msgs::msg::CollisionObject obj;
                obj.id              = "table";
                obj.header.frame_id = frame;
                obj.operation       = moveit_msgs::msg::CollisionObject::ADD;
                shape_msgs::msg::SolidPrimitive box;
                box.type       = shape_msgs::msg::SolidPrimitive::BOX;
                box.dimensions = {size[0], size[1], top};   // full box, floor -> top
                geometry_msgs::msg::Pose p;
                p.position.x    = pose[0];
                p.position.y    = pose[1];
                p.position.z    = top * 0.5;
                p.orientation.z = std::sin(yaw * 0.5);
                p.orientation.w = std::cos(yaw * 0.5);
                obj.primitives.push_back(box);
                obj.primitive_poses.push_back(p);
                ops.push_back(obj);
                added = true;
            }
        }
        if (!added && table_added_) {
            moveit_msgs::msg::CollisionObject rm;
            rm.id = "table";
            rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            ops.push_back(rm);
        }
        if (ops.empty()) return;
        planning_scene_->applyCollisionObjects(ops);
        table_added_ = added;
        RCLCPP_INFO(node_->get_logger(), "Table collision box %s (frame=%s)",
                    added ? "applied" : "cleared", frame.c_str());
    }

    void scheduleTableReapply()
    {
        table_apply_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                table_apply_timer_->cancel();
                buildAndApplyTable();
            },
            callback_group_);
    }

    // ========== ACM helpers ==========
    // Ensure `n` is a row/col in the ACM entry matrix; return its index. New names
    // append all-false entries (collision checked by default).
    size_t acmEnsureName(moveit_msgs::msg::AllowedCollisionMatrix & acm,
                         const std::string & n)
    {
        auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), n);
        if (it != acm.entry_names.end())
            return static_cast<size_t>(std::distance(acm.entry_names.begin(), it));
        acm.entry_names.push_back(n);
        for (auto & row : acm.entry_values)
            row.enabled.push_back(false);
        moveit_msgs::msg::AllowedCollisionEntry row;
        row.enabled.assign(acm.entry_names.size(), false);
        acm.entry_values.push_back(row);
        return acm.entry_names.size() - 1;
    }

    void setAcmPair(moveit_msgs::msg::AllowedCollisionMatrix & acm,
                    const std::string & a, const std::string & b, bool allowed)
    {
        const size_t ia = acmEnsureName(acm, a);
        const size_t ib = acmEnsureName(acm, b);
        acm.entry_values[ia].enabled[ib] = allowed;
        acm.entry_values[ib].enabled[ia] = allowed;
    }

    // ========== (A) Allow gripper links vs the octomap only ==========
    // Sets the ACM pairs (each gripper link, "<octomap>") = allow_gripper_vs_octomap.
    // Lets grasping inside sensed voxels plan, WITHOUT the fingers ignoring
    // explicit collision objects/other links (unlike the blanket gripper disable).
    void applyGripperOctomapAllowance()
    {
        const bool allow = node_->get_parameter("allow_gripper_vs_octomap").as_bool();
        if (!get_planning_scene_client_->service_is_ready() ||
            !apply_planning_scene_client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(),
                "planning scene services not ready; gripper<->octomap allowance NOT applied");
            return;
        }
        auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_req->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto fut = get_planning_scene_client_->async_send_request(get_req);
        if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_WARN(node_->get_logger(), "timed out reading ACM for gripper<->octomap allowance");
            return;
        }
        auto acm = fut.get()->scene.allowed_collision_matrix;
        for (const auto & grp : {std::string("left_gripper"), std::string("right_gripper")}) {
            std::string hand, obj;
            std::vector<std::string> links;
            if (!gripperAttachInfo(grp, hand, links, obj)) continue;
            for (const auto & link : links)
                setAcmPair(acm, link, "<octomap>", allow);
        }
        auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        apply_req->scene.is_diff = true;
        apply_req->scene.allowed_collision_matrix = acm;
        auto af = apply_planning_scene_client_->async_send_request(apply_req);
        if (af.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
            !af.get()->success) {
            RCLCPP_WARN(node_->get_logger(), "failed to apply gripper<->octomap allowance");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Gripper <-> octomap collision %s",
            allow ? "ALLOWED (fingers ignore octomap, still avoid explicit objects)"
                  : "ENFORCED");
    }

    void scheduleGripperOctomapAllowance()
    {
        octomap_acm_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                octomap_acm_timer_->cancel();
                applyGripperOctomapAllowance();
            },
            callback_group_);
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
        group->setGoalTolerance(0.02);

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
    // Send one GripperCommand goal. With wait_result, block up to 5 s for the
    // controller's result and return whether it SUCCEEDED. Without it, return
    // true as soon as the goal is accepted — used for the intermediate steps
    // of the software ramp, which are paced by sleeps rather than results.
    // accepted (out, optional): set true once the controller ACCEPTS the goal,
    // independent of the eventual result. A grasp-close legitimately ends
    // STALLED/ABORTED (the finger jams on the object before the setpoint), so
    // callers track the commanded position by acceptance, not by SUCCEEDED.
    bool sendGripperGoal(rclcpp_action::Client<GripperCommand>::SharedPtr client,
                         double position, double max_effort, bool wait_result,
                         bool * accepted = nullptr)
    {
        if (accepted) *accepted = false;

        GripperCommand::Goal goal;
        goal.command.position   = position;
        goal.command.max_effort = max_effort;

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
        if (accepted) *accepted = true;
        if (!wait_result) return true;

        auto result_future = client->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto result = result_future.get();
            return (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
        return false;
    }

    // Fire-and-forget setpoint for the INTERMEDIATE steps of the ramp. Unlike
    // sendGripperGoal(), this does NOT block on the goal-acceptance handshake —
    // that round-trip (variable, and worse over zenoh) would be paid on top of
    // the 20 ms tick, so the setpoint would advance in irregular bursts and the
    // finger would judder/run at an inconsistent speed. Each goal still preempts
    // the previous one in GripperActionController; we only care that the latest
    // setpoint is the live command. The loop paces itself with a steady_clock
    // deadline, so dropping the handshake is what makes the ramp truly 50 Hz.
    void sendGripperSetpoint(rclcpp_action::Client<GripperCommand>::SharedPtr client,
                             double position, double max_effort)
    {
        GripperCommand::Goal goal;
        goal.command.position   = position;
        goal.command.max_effort = max_effort;
        client->async_send_goal(goal);   // fire and forget; do not wait for accept
    }

    // Cache each finger's position from /joint_states. Keyed by gripper group so
    // currentGripperPos() is a cheap, thread-safe, always-fresh lookup.
    void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(finger_pos_mtx_);
        const size_t n = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < n; ++i) {
            if (msg->name[i] == "openarm_left_finger_joint1")
                finger_pos_["left_gripper"] = msg->position[i];
            else if (msg->name[i] == "openarm_right_finger_joint1")
                finger_pos_["right_gripper"] = msg->position[i];
        }
    }

    // The gripper's actual current finger position (metres), cached from
    // /joint_states. The finger (openarm_*_finger_joint1) is prismatic in
    // [0, 0.044] m and GripperActionController commands it directly, so this is
    // the SAME scale as the GripperCommand goal — the correct ramp start.
    // Returns false only if no joint_states have arrived yet.
    bool currentGripperPos(const std::string & group_name, double & pos)
    {
        std::lock_guard<std::mutex> lk(finger_pos_mtx_);
        auto it = finger_pos_.find(group_name);
        if (it == finger_pos_.end()) return false;
        pos = it->second;
        return true;
    }

    // superseded (out, optional): set true when this call bails because a newer
    // gripper command for the same group arrived. Lets the caller skip the
    // grasp-scene update for a stale/spammed command.
    bool controlGripper(const std::string & group_name, double position,
                        bool * superseded = nullptr)
    {
        if (superseded) *superseded = false;
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

        constexpr double MAX_EFFORT = 50.0;

        // Read gripper_speed FRESH from the parameter on every command (like the
        // grasp_* params) so `ros2 param set <node> gripper_speed <v>` — or the
        // SDK/web param control — takes effect live. The gripper_speed_ member is
        // only the constructor default; using it directly would ignore runtime
        // changes (the param has no on-set callback to refresh the member).
        const double gripper_speed = node_->get_parameter("gripper_speed").as_double();

        // GripperActionController commands position with no speed control, so
        // we cap the gripper velocity here by ramping the setpoint at
        // gripper_speed (command-position units per second, a ROS param).
        // We ramp in command space from the last commanded position so units
        // always match — the joint_states reading uses a different scale than
        // the GripperCommand goal, so it cannot be used as the ramp start.
        //
        // A NEW gripper command can arrive mid-ramp (each goal runs in its own
        // detached thread and the server accepts goals concurrently). To stop
        // two ramps from interleaving setpoints on the same client — which yanks
        // the finger between the two targets and makes it judder — every ramp
        // claims a generation token under gripper_mtx_. A ramp aborts the instant
        // a newer one supersedes it, and the last commanded position is updated
        // on EVERY tick so the superseding ramp starts from where the finger
        // actually is and continues smoothly instead of jumping.
        uint64_t my_gen;
        bool have_start;
        double start;
        {
            std::lock_guard<std::mutex> lk(gripper_mtx_);
            my_gen = ++gripper_gen_[group_name];
            auto it = last_gripper_cmd_.find(group_name);
            have_start = (it != last_gripper_cmd_.end());
            start = have_start ? it->second : position;
        }

        // Serialize the actual gripper operation. Each ControlGripper goal runs
        // in its own detached thread, so spamming open/close would otherwise run
        // many controlGripper() bodies at once — and currentGripperPos() below
        // calls MoveGroupInterface::getCurrentState(), which is NOT thread-safe.
        // Concurrent access crashes the node. Holding this lock for the whole
        // operation guarantees one ramp at a time; a spammed command that has
        // already been superseded by a newer one (higher generation) grabs the
        // lock, sees it's stale, and returns immediately instead of doing work.
        std::lock_guard<std::mutex> op_lk(gripper_op_mtx_);
        if (gripperSuperseded(group_name, my_gen)) {
            if (superseded) *superseded = true;
            return false;   // a newer command arrived while we waited for the lock
        }

        // Prefer the ACTUAL current finger position as the ramp start. This is
        // what makes the FIRST command after launch ramp instead of jumping:
        // the seeded last_gripper_cmd_ is only a guess, and when the first
        // target happens to equal that guess (e.g. first close to 0.0 with the
        // 0.0 seed) the start==position shortcut below skips the ramp entirely.
        // Reading the real position also makes a command after a grasp stall
        // ramp from where the finger actually jammed. Guard with a sanity range
        // so hardware that (unexpectedly) reports a different scale falls back
        // to the tracked/seeded command rather than starting from a bad value.
        constexpr double GRIPPER_MIN_POS = 0.0;
        constexpr double GRIPPER_MAX_POS = 0.044;   // finger_joint1 upper limit
        constexpr double GRIPPER_POS_MARGIN = 0.01;
        double actual;
        if (currentGripperPos(group_name, actual) &&
            actual >= GRIPPER_MIN_POS - GRIPPER_POS_MARGIN &&
            actual <= GRIPPER_MAX_POS + GRIPPER_POS_MARGIN) {
            start = actual;
            have_start = true;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Setting %s to %.3f (from %.3f @ %.2f units/s)",
                    group_name.c_str(), position, start, gripper_speed);

        bool ok;
        bool accepted = false;   // did the controller accept the final goal?
        if (!have_start || gripper_speed <= 0.0 || start == position) {
            // First command for this gripper (no known start) or ramp disabled:
            // command directly. Subsequent calls ramp from here.
            ok = sendGripperGoal(client, position, MAX_EFFORT, true, &accepted);
        } else {
            const auto period = std::chrono::milliseconds(20);   // 50 Hz ramp
            const double period_s = 0.02;
            const double step = gripper_speed * period_s; // units per tick
            const double dist = std::abs(position - start);
            const int steps = std::max(1, static_cast<int>(std::ceil(dist / step)));

            // Pace against an ABSOLUTE deadline rather than sleep_for: send
            // latency then can't accumulate, so the ramp keeps a true 50 Hz and
            // the open/close speed is the configured gripper_speed_ regardless of
            // direction or transport jitter.
            auto next = std::chrono::steady_clock::now();
            ok = true;
            for (int i = 1; i <= steps && ok; ++i) {
                if (gripperSuperseded(group_name, my_gen)) {
                    if (superseded) *superseded = true;
                    return false;   // a newer command owns the gripper now
                }
                const double frac = static_cast<double>(i) / steps;
                const double cmd = start + (position - start) * frac;
                if (i == steps) {
                    // Final setpoint: block for the controller result so the
                    // action reports real success/failure (stall on grasp, etc.).
                    ok = sendGripperGoal(client, cmd, MAX_EFFORT, true, &accepted);
                } else {
                    sendGripperSetpoint(client, cmd, MAX_EFFORT);
                    if (!noteGripperCmd(group_name, my_gen, cmd)) {
                        if (superseded) *superseded = true;
                        return false;   // superseded mid-ramp
                    }
                    next += period;
                    std::this_thread::sleep_until(next);
                }
            }
        }

        // Record the commanded setpoint as long as the controller ACCEPTED it
        // AND we are still the latest command, even if the result was
        // STALLED/ABORTED (expected when the finger jams on a grasped object).
        // Gating this on `ok` (== SUCCEEDED) left last_gripper_cmd_ stale after
        // every grasp-close, so the next open saw start==position and jumped
        // open instantly; the generation guard keeps an interrupted ramp from
        // clobbering the value the superseding command is now tracking.
        {
            std::lock_guard<std::mutex> lk(gripper_mtx_);
            if (accepted && gripper_gen_[group_name] == my_gen)
                last_gripper_cmd_[group_name] = position;
        }
        return ok;
    }

    // True if a newer controlGripper() call for this group has claimed the
    // generation token, meaning the caller's ramp has been superseded.
    bool gripperSuperseded(const std::string & group_name, uint64_t gen)
    {
        std::lock_guard<std::mutex> lk(gripper_mtx_);
        return gripper_gen_[group_name] != gen;
    }

    // Record an intermediate ramp setpoint as the last commanded position, but
    // only while this ramp is still the latest. Returns false (telling the ramp
    // to abort) once superseded.
    bool noteGripperCmd(const std::string & group_name, uint64_t gen, double cmd)
    {
        std::lock_guard<std::mutex> lk(gripper_mtx_);
        if (gripper_gen_[group_name] != gen) return false;
        last_gripper_cmd_[group_name] = cmd;
        return true;
    }

    // ========== GetJointStates Service ==========

    void handle_get_joint_states(
        const std::shared_ptr<GetJointStates::Request> request,
        std::shared_ptr<GetJointStates::Response> response)
    {
        auto group = getGroup(request->group_name);
        if (!group) {
            response->success = false;
            response->status  = "Invalid group: " + request->group_name;
            return;
        }

        auto state = group->getCurrentState();
        if (!state) {
            response->success = false;
            response->status  = "Failed to get current robot state";
            return;
        }
        auto jmg = state->getJointModelGroup(request->group_name);
        if (!jmg) {
            response->success = false;
            response->status  = "Joint model group not found: " + request->group_name;
            return;
        }
        response->joint_names = jmg->getVariableNames();
        std::vector<double> positions;
        state->copyJointGroupPositions(jmg, positions);
        response->joint_positions = positions;
        response->success         = true;
        response->status          = "OK";
    }

    // ========== GetEEPose Service ==========

    void handle_get_ee_pose(
        const std::shared_ptr<GetEEPose::Request> request,
        std::shared_ptr<GetEEPose::Response> response)
    {
        auto group = getGroup(request->group_name);
        if (!group) {
            response->success = false;
            response->status = "Invalid group: " + request->group_name;
            return;
        }

        std::string frame = request->frame_id.empty() ? "base_footprint" : request->frame_id;
        std::string ee_link = group->getEndEffectorLink();

        if (ee_link.empty()) {
            response->success = false;
            response->status = "No end-effector link for group: " + request->group_name;
            return;
        }

        try {
            auto t = tf_buffer_->lookupTransform(frame, ee_link, tf2::TimePointZero);
            response->x        = t.transform.translation.x;
            response->y        = t.transform.translation.y;
            response->z        = t.transform.translation.z;
            response->qx       = t.transform.rotation.x;
            response->qy       = t.transform.rotation.y;
            response->qz       = t.transform.rotation.z;
            response->qw       = t.transform.rotation.w;
            response->frame_id = frame;
            response->success  = true;
            response->status   = "OK";
        } catch (const tf2::TransformException & ex) {
            response->success = false;
            response->status  = std::string("TF lookup failed: ") + ex.what();
        }
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
        std::string frame = goal->frame_id.empty() ? "base_footprint" : goal->frame_id;
        group->setPoseReferenceFrame(frame);
        group->setGoalTolerance(0.02);

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
            if (plan_only_) {
                stored_plans_[goal->group_name] = plan;
                result->success = true; result->status = "Plan Only";
                goal_handle->succeed(result);
            } else {
                auto err = group->execute(plan);
                if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                    flushPendingGraspRemovals();
                    result->success = true; result->status = "Success";
                    goal_handle->succeed(result);
                } else {
                    result->success = false; result->status = "Execution Failed";
                    goal_handle->abort(result);
                }
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

        std::string target_frame = goal->frame_id.empty() ? "base_footprint" : goal->frame_id;
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
        group->setGoalTolerance(0.02);

        tf2::Quaternion q_current;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::Quaternion q_increment;
        q_increment.setRPY(goal->roll, goal->pitch, goal->yaw);
        // ee_frame=false: rotate around frame_id (world-fixed) axes
        // ee_frame=true:  rotate around EEF's own (body-fixed) axes
        tf2::Quaternion q_final = goal->ee_frame
            ? (q_current * q_increment)
            : (q_increment * q_current);
        q_final.normalize();

        tf2::Vector3 v_offset(goal->x, goal->y, goal->z);
        if (goal->ee_frame) {
            // rotate offset from EEF-local into the reference frame
            v_offset = tf2::Matrix3x3(q_current) * v_offset;
        }

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + v_offset.x();
        target_pose.position.y = current_pose.position.y + v_offset.y();
        target_pose.position.z = current_pose.position.z + v_offset.z();
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

        if (plan_success && plan_only_) {
            if (using_cartesian_result) {
                MoveGroupInterface::Plan stored;
                stored.trajectory = cartesian_traj;
                stored_plans_[goal->group_name] = stored;
            } else {
                stored_plans_[goal->group_name] = plan;
            }
            result->success = true; result->status = "Plan Only";
            goal_handle->succeed(result);
            return;
        }

        moveit::core::MoveItErrorCode err = moveit::core::MoveItErrorCode::FAILURE;
        if (plan_success) {
            err = using_cartesian_result ? group->execute(cartesian_traj) : group->execute(plan);
        }

        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
            flushPendingGraspRemovals();   // motion succeeded -> delete released grasp box(es)
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
        std::string frame = goal->frame_id.empty() ? "base_footprint" : goal->frame_id;
        group->setPoseReferenceFrame(frame);
        group->setGoalTolerance(0.02);

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
            if (plan_only_) {
                stored_plans_[goal->group_name] = plan;
                result->success = true; result->status = "Plan Only";
                goal_handle->succeed(result);
            } else {
                auto err = group->execute(plan);
                if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                    flushPendingGraspRemovals();
                    result->success = true; result->status = "Success";
                    goal_handle->succeed(result);
                } else {
                    result->success = false; result->status = "Execution Failed";
                    goal_handle->abort(result);
                }
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

        bool superseded = false;
        bool success = controlGripper(goal->group_name, goal->position, &superseded);

        if (goal_handle->is_canceling()) {
            result->success = false; result->status = "Canceled";
            goal_handle->canceled(result); return;
        }

        // Update the planning-scene grasp box according to the grasp_scene_action
        // param, read fresh here so `ros2 param set ... grasp_scene_action place`
        // takes effect on the very next command. This is what lets an "open" mean
        // either "place the held object" (detach + remove after the next motion)
        // or "just open, nothing held" (leave the scene alone). You set the
        // intent explicitly before the command — there is no position-based auto.
        //
        // Driven by intent (the param), NOT by `success`: grasping a real object
        // makes the gripper stall before reaching the closed setpoint, so the
        // controller often reports ABORTED/STALLED instead of SUCCEEDED. We still
        // report success/failure in the action result below.
        // Skip the scene update for a superseded (spammed) command: it didn't
        // actually move the gripper, and letting many stale threads hit
        // PlanningSceneInterface concurrently is unsafe. Only the command that
        // ran updates the scene.
        if (grasp_attach_enable_ && !superseded) {
            std::string action = node_->get_parameter("grasp_scene_action").as_string();
            if      (action == "grasp") attachGraspObject(goal->group_name);
            else if (action == "place") detachGraspObject(goal->group_name);
            else if (action == "none")  { /* leave the planning scene untouched */ }
            else {
                RCLCPP_WARN(node_->get_logger(),
                            "Unknown grasp_scene_action '%s' (use grasp|place|none); "
                            "leaving scene untouched", action.c_str());
            }
        }

        // Grasp-success detection: read the settled finger gap and decide whether
        // an object is held. An object is grasped iff the gap lands in the detect
        // band (clearly above empty-closed, clearly below fully-open). Reported in
        // the result regardless of the controller's SUCCEEDED/STALLED code (a real
        // grasp usually stalls before the closed setpoint, so `success` is often
        // false even when the bottle IS held — `grasped` is the answer to ask).
        //
        // ONLY on a CLOSE/grasp command (goal->position <= grasp_close_threshold_).
        // Opening the gripper is never a grasp, so we leave grasped=false there
        // rather than reporting a meaningless (and possibly false) hold verdict.
        if (!superseded && node_->get_parameter("grasp_detect_enable").as_bool()
            && goal->position <= grasp_close_threshold_) {
            double gap = 0.0;
            if (currentGripperPos(goal->group_name, gap)) {
                const double lo = node_->get_parameter("grasp_detect_min").as_double();
                const double hi = node_->get_parameter("grasp_detect_max").as_double();
                result->gripper_gap = gap;
                result->grasped     = (gap >= lo && gap <= hi);
                RCLCPP_INFO(node_->get_logger(),
                    "Grasp detect [%s]: gap=%.4f band=[%.4f,%.4f] -> %s",
                    goal->group_name.c_str(), gap, lo, hi,
                    result->grasped ? "HELD" : "empty");
            }
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

        const std::string pose_name =
            goal->pose_name.empty() ? "home" : goal->pose_name;

        RCLCPP_INFO(node_->get_logger(), "Executing GoToHome (%s) for %s",
                    pose_name.c_str(), goal->group_name.c_str());

        group->setStartStateToCurrentState();
        group->setPoseReferenceFrame("base_footprint");
        group->setGoalTolerance(0.02);

        // Use the SRDF named state rather than a fabricated all-zeros target.
        // For lift-inclusive groups the states keep lift_joint = 0.7435 (the
        // homed top of travel); forcing it to 0.0 lands the lift on its lower
        // joint limit with the arm folded against the body, which OMPL cannot
        // sample as a valid goal ("Insufficient states in sampleable goal
        // region"). The all-zeros fallback applies only to the default "home"
        // request; an explicitly requested state that is missing from the SRDF
        // aborts instead of silently moving somewhere else.
        if (!group->setNamedTarget(pose_name)) {
            if (!goal->pose_name.empty()) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Unknown SRDF state '%s' for group %s",
                             pose_name.c_str(), goal->group_name.c_str());
                result->success = false;
                goal_handle->abort(result); return;
            }
            std::vector<double> home_joints(group->getVariableCount(), 0.0);
            group->setJointValueTarget(home_joints);
        }

        MoveGroupInterface::Plan plan;
        bool plan_success = (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result); return;
        }

        if (plan_success) {
            if (plan_only_) {
                stored_plans_[goal->group_name] = plan;
                result->success = true;
                goal_handle->succeed(result);
            } else {
                auto err = group->execute(plan);
                if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                    flushPendingGraspRemovals();
                    result->success = true;
                    goal_handle->succeed(result);
                } else {
                    result->success = false;
                    goal_handle->abort(result);
                }
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan path to '%s' for %s",
                         pose_name.c_str(), goal->group_name.c_str());
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
        group->setGoalTolerance(0.02);

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
            if (plan_only_) {
                stored_plans_[goal->group_name] = plan;
                result->success = true; result->status = "Plan Only";
                goal_handle->succeed(result);
            } else {
                auto err = group->execute(plan);
                if (err == moveit::core::MoveItErrorCode::SUCCESS) {
                    flushPendingGraspRemovals();
                    result->success = true; result->status = "Success";
                    goal_handle->succeed(result);
                    RCLCPP_INFO(node_->get_logger(), "SetJointPosition succeeded for %s",
                                goal->group_name.c_str());
                } else {
                    result->success = false; result->status = "Execution Failed";
                    goal_handle->abort(result);
                }
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
    // Keeps the live-retune callback for planner_id / arm_planning_time alive.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

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
    rclcpp::Service<GetJointStates>::SharedPtr get_joint_states_service_;
    rclcpp::Service<GetEEPose>::SharedPtr get_ee_pose_service_;

    // Runtime toggle of gripper-link collision checking via the ACM.
    rclcpp::Service<ToggleGripperCollision>::SharedPtr toggle_gripper_collision_service_;
    // Detach + remove all collision objects from the planning scene.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_collision_objects_service_;
    // Execute the plan stored by the last plan_only action.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_stored_plan_service_;
    // Select whether planning uses the octomap (ACM toggle).
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_octomap_collision_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_all_collision_service_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;

    // Software gripper speed cap and last commanded position per gripper group,
    // used by controlGripper() to ramp the setpoint (GripperActionController
    // has no built-in velocity limit).
    double gripper_speed_;
    std::map<std::string, double> last_gripper_cmd_;

    // Serializes gripper command state and provides preemption: each
    // controlGripper() call bumps gripper_gen_[group]; an older ramp aborts once
    // it sees a newer generation, so concurrent commands on the same gripper
    // can't interleave setpoints (which would make the finger judder).
    std::mutex gripper_mtx_;
    std::map<std::string, uint64_t> gripper_gen_;

    // Held for the whole duration of a single controlGripper() call so gripper
    // operations never run concurrently (e.g. concurrent action-client sends or
    // grasp-scene updates). Superseded/spammed commands take it, see they're
    // stale via gripper_gen_, and return without doing work.
    std::mutex gripper_op_mtx_;

    // Finger positions cached from /joint_states (gripper group -> metres), the
    // ramp start used by controlGripper(). Replaces MoveGroupInterface state
    // reads: always fresh and reliable, so open/close always ramps.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex finger_pos_mtx_;
    std::map<std::string, double> finger_pos_;

    // Grasp attach/detach: when the gripper closes past grasp_close_threshold_
    // a box (grasp_object_size_/offset_) is attached to the hand so only the
    // fingers may touch it and it is masked from the octomap. Opening detaches.
    bool grasp_attach_enable_;
    double grasp_close_threshold_;
    std::vector<double> grasp_object_size_;
    std::vector<double> grasp_object_offset_;
    std::vector<double> grasp_object_rpy_;
    double grasp_object_height_above_;   // m from grasp centre to object top
    double grasp_object_height_below_;   // m from grasp centre to object bottom
    std::shared_ptr<PlanningSceneInterface> planning_scene_;

    // Grasp boxes that were detached on gripper-open but are still standing in
    // the world. flushPendingGraspRemovals() deletes them at the start of the
    // next arm motion (before planning); attachGraspObject() un-queues on re-grasp.
    std::set<std::string> pending_grasp_removal_;

    // Finger collision padding: inflate the finger links' collision geometry by
    // finger_padding_ (m) so planning keeps clearance from the (octomap-sensed)
    // table. Applied once via a deferred one-shot timer after move_group is up.
    bool plan_only_;
    std::map<std::string, MoveGroupInterface::Plan> stored_plans_;

    bool finger_padding_enable_;
    double finger_padding_;
    std::vector<std::string> finger_pad_links_;
    rclcpp::TimerBase::SharedPtr padding_timer_;
    rclcpp::TimerBase::SharedPtr finger_apply_timer_;   // live finger_padding re-apply
    rclcpp::TimerBase::SharedPtr table_timer_;          // (B) deferred table startup apply
    rclcpp::TimerBase::SharedPtr table_apply_timer_;    // (B) live table re-apply
    bool table_added_ = false;
    rclcpp::TimerBase::SharedPtr octomap_acm_timer_;    // (A) live gripper<->octomap allowance
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
