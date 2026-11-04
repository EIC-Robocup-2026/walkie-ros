// Tracks the base part of a whole-body MoveIt plan.
//
// MoveIt's base_controller entry sends a FollowJointTrajectory goal whose base
// motion is in multi_dof_trajectory, as map-frame poses and velocities. The stock
// joint_trajectory_controller ignores that field, so this node is its own server.
//
// Commands go out on /cmd_vel_smoothed, the input of Nav2's collision monitor, as
// body-frame TwistStamped messages with a fresh stamp. Nothing downstream stamps
// or rotates them. The omni controller runs open loop, so the loop is closed here
// on the map -> base_footprint transform.
//
// An abort on one MoveIt controller does not cancel the others. Every abort here
// therefore also publishes "stop" on trajectory_execution_event, which halts the
// arm and the lift with the base.
//
// Base motion needs a live caller. The commander publishes a heartbeat, and without
// one this node refuses a goal and stops a running one, because move_group would
// otherwise carry on with the plan of a commander that has died. A "stop" on
// trajectory_execution_event also ends a goal directly, which still works when
// move_group itself is what has hung.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using CollisionMonitorState = nav2_msgs::msg::CollisionMonitorState;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TrajectorySample
{
  double time{0.0};
  Pose2D pose;
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
};

// Linear interpolation between waypoints, shortest way round on yaw. The planner's
// samples are 0.1 s apart at base speeds of 0.12 m/s, so the error is negligible
// next to the feedback term.
TrajectorySample sampleAt(const std::vector<TrajectorySample> & samples, double time)
{
  if (time <= samples.front().time) {
    return samples.front();
  }
  if (time >= samples.back().time) {
    TrajectorySample last = samples.back();
    last.vx = last.vy = last.vyaw = 0.0;
    return last;
  }
  const auto after = std::upper_bound(
    samples.begin(), samples.end(), time,
    [](double value, const TrajectorySample & sample) {return value < sample.time;});
  const auto before = std::prev(after);
  const double span = after->time - before->time;
  const double ratio = span > 1e-9 ? (time - before->time) / span : 1.0;
  TrajectorySample out;
  out.time = time;
  out.pose.x = before->pose.x + ratio * (after->pose.x - before->pose.x);
  out.pose.y = before->pose.y + ratio * (after->pose.y - before->pose.y);
  out.pose.yaw = normalizeAngle(
    before->pose.yaw + ratio * normalizeAngle(after->pose.yaw - before->pose.yaw));
  out.vx = before->vx + ratio * (after->vx - before->vx);
  out.vy = before->vy + ratio * (after->vy - before->vy);
  out.vyaw = before->vyaw + ratio * (after->vyaw - before->vyaw);
  return out;
}
}  // namespace

class WholeBodyBaseFollower : public rclcpp::Node
{
public:
  WholeBodyBaseFollower()
  : Node("whole_body_base_follower")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "virtual_joint");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    position_gain_ = declare_parameter<double>("position_gain", 1.5);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 1.5);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.25);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.6);
    start_position_tolerance_ = declare_parameter<double>("start_position_tolerance", 0.05);
    start_yaw_tolerance_ = declare_parameter<double>("start_yaw_tolerance", 0.1);
    // A collision monitor slowdown at ratio 0.4 lags 12 cm behind at 0.12 m/s, and
    // that is normal near a table, so the abort threshold sits well above it.
    abort_position_error_ = declare_parameter<double>("abort_position_error", 0.25);
    abort_yaw_error_ = declare_parameter<double>("abort_yaw_error", 0.35);
    goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", 0.02);
    goal_yaw_tolerance_ = declare_parameter<double>("goal_yaw_tolerance", 0.03);
    goal_settle_time_ = declare_parameter<double>("goal_settle_time", 3.0);
    max_transform_age_ = declare_parameter<double>("max_transform_age", 0.5);
    // Wall time. At the planned 0.12 m/s the base covers about 24 cm in this time, and
    // a machine under load can stall a healthy commander for a second.
    commander_timeout_ = declare_parameter<double>("commander_timeout", 2.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Their own group and a second executor thread, so that these still arrive while
    // handleGoal() waits for the collision monitor's answer.
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto event_topic =
      declare_parameter<std::string>("event_topic", "/trajectory_execution_event");
    commander_topic_ =
      declare_parameter<std::string>("commander_topic", "/whole_body/commander_alive");
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      declare_parameter<std::string>("cmd_topic", "/cmd_vel_smoothed"), 10);
    event_pub_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
    monitor_sub_ = create_subscription<CollisionMonitorState>(
      declare_parameter<std::string>("monitor_state_topic", "/collision_monitor_state"), 10,
      [this](CollisionMonitorState::ConstSharedPtr msg) {
        monitor_stop_ = msg->action_type == CollisionMonitorState::STOP;
        if (monitor_stop_) {
          std::lock_guard<std::mutex> lock(monitor_mutex_);
          monitor_polygon_ = msg->polygon_name;
        }
      },
      options);
    event_sub_ = create_subscription<std_msgs::msg::String>(
      event_topic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data == "stop") {
          stop_requested_ = true;
        }
      },
      options);
    commander_sub_ = create_subscription<std_msgs::msg::Empty>(
      commander_topic_, 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        last_heartbeat_ = std::chrono::steady_clock::now().time_since_epoch().count();
      },
      options);

    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, "/base_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID &, FollowJointTrajectory::Goal::ConstSharedPtr goal) {
        return handleGoal(*goal);
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        if (worker_.joinable()) {
          worker_.join();
        }
        worker_ = std::thread([this, goal_handle]() {
              execute(goal_handle);
              busy_ = false;
            });
      });
    RCLCPP_INFO(get_logger(), "Serving /base_controller/follow_joint_trajectory");
  }

  ~WholeBodyBaseFollower() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  // A rejected goal stops the MoveIt dispatch here, because base_controller sorts
  // first, so the arm and the lift never start. Bad goals are therefore refused now
  // and not aborted after acceptance.
  rclcpp_action::GoalResponse handleGoal(const FollowJointTrajectory::Goal & goal)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejected: a base trajectory is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::string problem;
    if (!parseTrajectory(goal, problem) || !startPoseMatches(problem) ||
      !commanderReady(problem) || !monitorAllowsMotion(problem))
    {
      RCLCPP_WARN(get_logger(), "Rejected: %s", problem.c_str());
      busy_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }
    stop_requested_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  bool commanderAlive() const
  {
    const auto age = std::chrono::steady_clock::now().time_since_epoch().count() -
      last_heartbeat_.load();
    return std::chrono::duration<double>(std::chrono::steady_clock::duration(age)).count() <=
           commander_timeout_;
  }

  bool commanderReady(std::string & problem) const
  {
    if (commanderAlive()) {
      return true;
    }
    problem = "no heartbeat from the commander on " + commander_topic_;
    return false;
  }

  // The monitor reports a change only, and looks at its sensors only when a command
  // arrives, so a STOP on record can be long over, and no state message would ever say so
  // while the base sits idle. A zero command makes it look again. Without this, one STOP
  // would refuse every later goal, because no goal means no command.
  bool monitorAllowsMotion(std::string & problem)
  {
    if (!monitor_stop_) {
      return true;
    }
    publishTwist(0.0, 0.0, 0.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (monitor_stop_ && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!monitor_stop_) {
      return true;
    }
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    problem = "collision monitor stop on polygon " + monitor_polygon_;
    return false;
  }

  bool parseTrajectory(const FollowJointTrajectory::Goal & goal, std::string & problem)
  {
    const auto & trajectory = goal.multi_dof_trajectory;
    if (trajectory.joint_names.size() != 1 || trajectory.joint_names[0] != joint_name_) {
      problem = "the goal must carry only " + joint_name_ + " in multi_dof_trajectory";
      return false;
    }
    if (trajectory.points.empty()) {
      problem = "the base trajectory has no points";
      return false;
    }
    std::vector<TrajectorySample> samples;
    samples.reserve(trajectory.points.size());
    for (const auto & point : trajectory.points) {
      if (point.transforms.size() != 1) {
        problem = "a base trajectory point does not have exactly one transform";
        return false;
      }
      TrajectorySample sample;
      sample.time = rclcpp::Duration(point.time_from_start).seconds();
      if (!samples.empty() && sample.time < samples.back().time) {
        problem = "the base trajectory times go backwards";
        return false;
      }
      sample.pose.x = point.transforms[0].translation.x;
      sample.pose.y = point.transforms[0].translation.y;
      tf2::Quaternion rotation;
      tf2::fromMsg(point.transforms[0].rotation, rotation);
      sample.pose.yaw = tf2::getYaw(rotation);
      if (!point.velocities.empty()) {
        // MoveIt writes planar velocities in the map frame.
        sample.vx = point.velocities[0].linear.x;
        sample.vy = point.velocities[0].linear.y;
        sample.vyaw = point.velocities[0].angular.z;
      }
      samples.push_back(sample);
    }
    samples_ = std::move(samples);
    return true;
  }

  bool startPoseMatches(std::string & problem)
  {
    const auto current = lookupBasePose();
    if (!current) {
      problem = "no fresh transform from " + map_frame_ + " to " + base_frame_;
      return false;
    }
    const Pose2D & start = samples_.front().pose;
    const double position_error = std::hypot(start.x - current->x, start.y - current->y);
    const double yaw_error = std::fabs(normalizeAngle(start.yaw - current->yaw));
    if (position_error > start_position_tolerance_ || yaw_error > start_yaw_tolerance_) {
      problem = "the base is " + std::to_string(position_error) + " m and " +
        std::to_string(yaw_error) + " rad away from the trajectory start";
      return false;
    }
    return true;
  }

  std::optional<Pose2D> lookupBasePose()
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      if (age > max_transform_age_) {
        return std::nullopt;
      }
      Pose2D pose;
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      tf2::Quaternion rotation;
      tf2::fromMsg(transform.transform.rotation, rotation);
      pose.yaw = tf2::getYaw(rotation);
      return pose;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void publishTwist(double vx, double vy, double vyaw)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = base_frame_;
    command.twist.linear.x = vx;
    command.twist.linear.y = vy;
    command.twist.angular.z = vyaw;
    cmd_pub_->publish(command);
  }

  // The collision monitor only acts on incoming commands, and a silent input keeps
  // the last command alive for cmd_vel_timeout, so a stop is sent as explicit zeros.
  void publishStop()
  {
    for (int i = 0; i < 5; ++i) {
      publishTwist(0.0, 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void abortAll(const std::shared_ptr<GoalHandle> & goal_handle, const std::string & reason)
  {
    publishStop();
    std_msgs::msg::String event;
    event.data = "stop";
    event_pub_->publish(event);
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
    result->error_string = reason;
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "Aborted: %s", reason.c_str());
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const rclcpp::Time start = now();
    const double duration = samples_.back().time;
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->multi_dof_joint_names = {joint_name_};
    rclcpp::WallRate rate(control_rate_);
    int tick = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        publishStop();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_string = "canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Canceled");
        return;
      }
      if (stop_requested_) {
        abortAll(goal_handle, "a stop was requested on trajectory_execution_event");
        return;
      }
      if (!commanderAlive()) {
        abortAll(goal_handle, "the commander stopped reporting on " + commander_topic_);
        return;
      }
      if (monitor_stop_) {
        std::string polygon;
        {
          std::lock_guard<std::mutex> lock(monitor_mutex_);
          polygon = monitor_polygon_;
        }
        abortAll(goal_handle, "collision monitor stop on polygon " + polygon);
        return;
      }
      const auto current = lookupBasePose();
      if (!current) {
        abortAll(
          goal_handle, "no fresh transform from " + map_frame_ + " to " + base_frame_);
        return;
      }

      const double elapsed = (now() - start).seconds();
      const TrajectorySample desired = sampleAt(samples_, elapsed);
      const double error_x = desired.pose.x - current->x;
      const double error_y = desired.pose.y - current->y;
      const double error_yaw = normalizeAngle(desired.pose.yaw - current->yaw);
      const double position_error = std::hypot(error_x, error_y);

      if (position_error > abort_position_error_ || std::fabs(error_yaw) > abort_yaw_error_) {
        abortAll(
          goal_handle, "tracking error " + std::to_string(position_error) + " m, " +
          std::to_string(error_yaw) + " rad");
        return;
      }
      if (elapsed >= duration) {
        if (position_error <= goal_position_tolerance_ &&
          std::fabs(error_yaw) <= goal_yaw_tolerance_)
        {
          publishStop();
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          goal_handle->succeed(result);
          RCLCPP_INFO(
            get_logger(), "Done with %.3f m and %.3f rad of error", position_error,
            std::fabs(error_yaw));
          return;
        }
        if (elapsed >= duration + goal_settle_time_) {
          abortAll(
            goal_handle, "did not settle: " + std::to_string(position_error) + " m, " +
            std::to_string(error_yaw) + " rad");
          return;
        }
      }

      // Map-frame velocity, then rotated into the body frame at the current yaw.
      double vx = desired.vx + position_gain_ * error_x;
      double vy = desired.vy + position_gain_ * error_y;
      const double speed = std::hypot(vx, vy);
      if (speed > max_linear_speed_) {
        vx *= max_linear_speed_ / speed;
        vy *= max_linear_speed_ / speed;
      }
      const double vyaw = std::clamp(
        desired.vyaw + yaw_gain_ * error_yaw, -max_angular_speed_, max_angular_speed_);
      const double cos_yaw = std::cos(current->yaw);
      const double sin_yaw = std::sin(current->yaw);
      publishTwist(cos_yaw * vx + sin_yaw * vy, -sin_yaw * vx + cos_yaw * vy, vyaw);

      if (++tick % 5 == 0) {
        feedback->header.stamp = now();
        goal_handle->publish_feedback(feedback);
      }
      rate.sleep();
    }
    publishStop();
  }

  std::string joint_name_;
  std::string map_frame_;
  std::string base_frame_;
  double control_rate_{50.0};
  double position_gain_{1.5};
  double yaw_gain_{1.5};
  double max_linear_speed_{0.25};
  double max_angular_speed_{0.6};
  double start_position_tolerance_{0.05};
  double start_yaw_tolerance_{0.1};
  double abort_position_error_{0.25};
  double abort_yaw_error_{0.35};
  double goal_position_tolerance_{0.02};
  double goal_yaw_tolerance_{0.03};
  double goal_settle_time_{3.0};
  double max_transform_age_{0.5};
  double commander_timeout_{2.0};
  std::string commander_topic_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<CollisionMonitorState>::SharedPtr monitor_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr commander_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;

  std::vector<TrajectorySample> samples_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> monitor_stop_{false};
  std::atomic<bool> stop_requested_{false};
  // steady_clock ticks. A start far in the past makes every check fail until the first beat.
  std::atomic<std::chrono::steady_clock::rep> last_heartbeat_{
    std::chrono::steady_clock::duration::min().count() / 2};
  std::mutex monitor_mutex_;
  std::string monitor_polygon_;
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WholeBodyBaseFollower>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
