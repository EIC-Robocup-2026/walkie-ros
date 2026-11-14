// IK-first arm pose goals for the commander.
//
// A pose goal makes OMPL sample goal states by IK inside its planning time, and with the
// commander's 10000 planning attempts every arm plan runs for the full arm_planning_time
// anyway. A joint goal needs no goal sampling. So for a single-arm group the commander
// first solves IK through move_group's /compute_ik, checks the whole robot at the solution
// through /check_state_validity, and plans to that joint goal with a shorter planning time.
// The pose goal stays as the fallback. See "Planning time and attempts" in
// docs/guides/manipulation.md.
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>

class IkFirstGoal
{
public:
  IkFirstGoal(
      const rclcpp::Node::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & callback_group,
      moveit::core::RobotModelConstPtr model)
  : model_(std::move(model))
  {
    compute_ik_ = node->create_client<moveit_msgs::srv::GetPositionIK>(
        "/compute_ik", rclcpp::ServicesQoS(), callback_group);
    state_validity_ = node->create_client<moveit_msgs::srv::GetStateValidity>(
        "/check_state_validity", rclcpp::ServicesQoS(), callback_group);
  }

  // Only a single chain has one tip to solve for. both_arms and both_arms_lift are not
  // chains and have no IK solver.
  bool applies(const moveit::planning_interface::MoveGroupInterface & group) const
  {
    const auto * jmg = model_->getJointModelGroup(group.getName());
    return jmg && jmg->isChain() && !group.getEndEffectorLink().empty();
  }

  // All service calls of one solve() share this much time. One IK call takes about 50 ms
  // (about 130 ms on a 0.02 m octomap) and a validity check less, so a goal reaches its
  // deadline only when move_group stops answering; then the pose goal is planned instead.
  static constexpr std::chrono::milliseconds kServiceTime{2000};

  // A joint solution of the group for its end-effector link at `pose` in `frame`, with the
  // whole robot collision free there and with the lift 1 cm lower. The same steps as
  // WholeBodyOption::solveForArmAlone: compute_ik only checks the links of its group, and
  // one IK answer can fail the clearance check from one seed and pass it from another, so
  // this tries the current state and the group's home state, twice each, and returns the
  // first solution that passes. A group without the lift skips the lowered-lift check.
  // Returns false within about 20 ms of `canceled` turning true, and on the first call
  // that gets no answer before kServiceTime has passed; no call is sent after either.
  bool solve(
      const moveit::planning_interface::MoveGroupInterface & group, const std::string & frame,
      const geometry_msgs::msg::Pose & pose, const std::function<bool()> & canceled,
      std::map<std::string, double> & joints, std::string & why)
  {
    if (!compute_ik_->service_is_ready() || !state_validity_->service_is_ready()) {
      why = "move_group's /compute_ik or /check_state_validity is not available";
      return false;
    }
    const auto deadline = Clock::now() + kServiceTime;
    // The lift stays out of the home seed: it is the slowest joint, so a solution near its
    // current height arrives soonest.
    auto home = group.getNamedTargetValues("home");
    home.erase("lift_joint");
    const std::map<std::string, double> current;
    const std::array<const std::map<std::string, double> *, 4> seeds = {&current, &home, &current, &home};
    int solved = 0;
    for (const auto * seed : seeds) {
      const Answer ik = callIk(group, frame, pose, *seed, deadline, canceled, joints);
      if (ik == Answer::kNone) {
        why = "no answer from /compute_ik in time";
        return false;
      }
      if (ik == Answer::kNo) continue;
      ++solved;
      const Answer clear = hasClearance(joints, deadline, canceled);
      if (clear == Answer::kYes) return true;
      if (clear == Answer::kNone) {
        why = "no answer from /check_state_validity in time";
        return false;
      }
    }
    why = std::to_string(solved) + " of " + std::to_string(seeds.size()) +
          " IK calls solved, none with clearance for the whole robot";
    return false;
  }

private:
  using Clock = std::chrono::steady_clock;

  // kNone: canceled, or no answer before the deadline.
  enum class Answer { kYes, kNo, kNone };

  // Sends `request` and waits for the answer in steps of at most 20 ms, so a cancel takes
  // effect within one step. Returns null when the goal was canceled or the deadline had
  // passed, before the request went out or by the time its answer is read, so an answer
  // read late does not count. A request still pending then is dropped from the client.
  template <typename ServiceT>
  static typename ServiceT::Response::SharedPtr call(
      rclcpp::Client<ServiceT> & client, const typename ServiceT::Request::SharedPtr & request,
      Clock::time_point deadline, const std::function<bool()> & canceled)
  {
    if (canceled() || Clock::now() >= deadline) return nullptr;
    auto future = client.async_send_request(request);
    for (;;) {
      const auto left = deadline - Clock::now();
      if (canceled() || left <= Clock::duration::zero()) break;
      const auto step = std::min<Clock::duration>(std::chrono::milliseconds(20), left);
      if (future.wait_for(step) == std::future_status::ready) {
        if (canceled() || Clock::now() >= deadline) return nullptr;
        return future.get();
      }
    }
    client.remove_pending_request(future.request_id);
    return nullptr;
  }

  // Joints that are not in the seed start from move_group's current state.
  Answer callIk(
      const moveit::planning_interface::MoveGroupInterface & group, const std::string & frame,
      const geometry_msgs::msg::Pose & pose, const std::map<std::string, double> & seed,
      Clock::time_point deadline, const std::function<bool()> & canceled,
      std::map<std::string, double> & joints)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = group.getName();
    request->ik_request.robot_state.is_diff = true;
    for (const auto & [name, value] : seed) {
      request->ik_request.robot_state.joint_state.name.push_back(name);
      request->ik_request.robot_state.joint_state.position.push_back(value);
    }
    request->ik_request.avoid_collisions = true;
    request->ik_request.ik_link_name = group.getEndEffectorLink();
    request->ik_request.pose_stamped.header.frame_id = frame;
    request->ik_request.pose_stamped.pose = pose;
    const auto response = call(*compute_ik_, request, deadline, canceled);
    if (!response) return Answer::kNone;
    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) return Answer::kNo;
    const auto & active = group.getActiveJoints();
    joints.clear();
    const auto & state = response->solution.joint_state;
    for (size_t i = 0; i < state.name.size() && i < state.position.size(); ++i) {
      if (std::find(active.begin(), active.end(), state.name[i]) != active.end()) {
        joints[state.name[i]] = state.position[i];
      }
    }
    return joints.size() == active.size() ? Answer::kYes : Answer::kNo;
  }

  // No group name, so move_group checks the whole robot, including the other arm.
  Answer stateValid(
      const std::map<std::string, double> & joints, Clock::time_point deadline,
      const std::function<bool()> & canceled)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->robot_state.is_diff = true;
    for (const auto & [name, value] : joints) {
      request->robot_state.joint_state.name.push_back(name);
      request->robot_state.joint_state.position.push_back(value);
    }
    const auto response = call(*state_validity_, request, deadline, canceled);
    if (!response) return Answer::kNone;
    return response->valid ? Answer::kYes : Answer::kNo;
  }

  // The lift settles below its command (about 3 mm in MuJoCo), so a goal that moves it
  // also has to be valid somewhat below where it asks the lift to stop.
  Answer hasClearance(
      std::map<std::string, double> joints, Clock::time_point deadline,
      const std::function<bool()> & canceled)
  {
    const Answer at_goal = stateValid(joints, deadline, canceled);
    if (at_goal != Answer::kYes) return at_goal;
    const auto lift = joints.find("lift_joint");
    if (lift == joints.end()) return Answer::kYes;
    lift->second = std::max(lift->second - 0.01, model_->getVariableBounds("lift_joint").min_position_);
    return stateValid(joints, deadline, canceled);
  }

  moveit::core::RobotModelConstPtr model_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_;
};
