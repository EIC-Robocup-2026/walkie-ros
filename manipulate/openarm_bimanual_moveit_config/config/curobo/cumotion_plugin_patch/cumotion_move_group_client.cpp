// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_cumotion_moveit/cumotion_move_group_client.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <set>
#include <string>

#include "moveit/robot_model/joint_model_group.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/planning_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace nvidia
{
namespace isaac
{
namespace manipulation
{

namespace
{

constexpr unsigned kGetGoalWaitIntervalInMs = 10;

}  // namespace

CumotionMoveGroupClient::CumotionMoveGroupClient(const rclcpp::Node::SharedPtr & node)
: result_ready(false),
  success(false),
  get_goal_handle_(false),
  get_result_handle_(false),
  node_(node),
  client_cb_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
  client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
    node_,
    "cumotion/move_group",
    client_cb_group_);

  send_goal_options_ = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

  send_goal_options_.goal_response_callback = std::bind(
    &CumotionMoveGroupClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options_.feedback_callback = std::bind(
    &CumotionMoveGroupClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  // NOTE: result_callback is intentionally NOT registered. getGoal() already
  // retrieves the result via async_get_result()'s future on the planning thread
  // (the solve() poll loop). Registering resultCallback too made the action
  // result drive TWO writers concurrently -- getGoal() on the planning thread and
  // resultCallback() on a multi-threaded-executor thread -- both reassigning
  // plan_response.trajectory (a std::vector free+realloc) with no lock. On
  // SUCCESSFUL plans (error_code.val == 1, the only branch that writes the
  // vector) that races into a heap double-free -> move_group aborts with
  // "malloc_consolidate(): invalid chunk size". async_get_result still satisfies
  // result_future_ without this callback, so getGoal() is unaffected. resultCallback
  // is left defined (unreferenced) only to avoid editing the container-only header.
}

void CumotionMoveGroupClient::updateGoal(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req)
{
  planning_request_ = req;

  // cuMotion (cuRobo) plans the ENTIRE XRDF cspace and expects the request's
  // start state to contain EXACTLY that group's cspace joints. Standard MoveIt
  // clients (RViz MotionPlanning, MoveGroupInterface / the bimanual commander)
  // send the FULL RobotState, so cuRobo fails with a dimension mismatch
  // ("cspace_position [N] must equal [cspace_dim]"). Prune the start state's
  // joint_state down to the planning group's active joints so any standard
  // client can drive cuMotion by only switching the planning pipeline -- no
  // per-request start-state surgery on the caller side.
  const moveit::core::JointModelGroup * jmg =
    planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
  if (jmg) {
    const auto & group_joints = jmg->getActiveJointModelNames();
    const std::set<std::string> keep(group_joints.begin(), group_joints.end());
    const auto & js = req.start_state.joint_state;
    sensor_msgs::msg::JointState pruned;
    pruned.header = js.header;
    for (size_t i = 0; i < js.name.size(); ++i) {
      if (keep.count(js.name[i]) == 0) {
        continue;
      }
      pruned.name.push_back(js.name[i]);
      if (i < js.position.size()) {pruned.position.push_back(js.position[i]);}
      if (i < js.velocity.size()) {pruned.velocity.push_back(js.velocity[i]);}
      if (i < js.effort.size()) {pruned.effort.push_back(js.effort[i]);}
    }
    planning_request_.start_state.joint_state = pruned;
    // is_diff stays false: the pruned joint_state is the complete cspace start.
  }
  planning_scene->getPlanningSceneMsg(planning_scene_);
}

bool CumotionMoveGroupClient::sendGoal()
{
  result_ready = false;
  success = false;

  moveit_msgs::msg::PlanningOptions plan_options;
  plan_options.planning_scene_diff = planning_scene_;

  if (!client_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = moveit_msgs::action::MoveGroup::Goal();

  goal_msg.planning_options = plan_options;
  goal_msg.request = planning_request_;
  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options_);
  goal_h_ = goal_handle_future;
  get_result_handle_ = true;
  get_goal_handle_ = true;
  return true;
}

void CumotionMoveGroupClient::getGoal()
{
  using namespace std::chrono_literals;

  if (get_goal_handle_) {
    if (goal_h_.wait_for(std::chrono::milliseconds(kGetGoalWaitIntervalInMs)) !=
      std::future_status::ready)
    {
      return;
    }

    GoalHandle::SharedPtr goal_handle = goal_h_.get();

    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      return;
    }
    auto result_future = client_->async_get_result(goal_handle);
    result_future_ = result_future;
    get_goal_handle_ = false;
  }

  if (get_result_handle_) {
    if (result_future_.wait_for(std::chrono::milliseconds(kGetGoalWaitIntervalInMs)) !=
      std::future_status::ready)
    {
      return;
    }

    auto res = result_future_.get();

    RCLCPP_INFO(node_->get_logger(), "Checking results");

    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Success");
      result_ready = true;
      success = false;
      plan_response.error_code = res.result->error_code;
      if (plan_response.error_code.val == 1) {
        success = true;
        plan_response.trajectory_start = res.result->trajectory_start;
        plan_response.group_name = planning_request_.group_name;
        plan_response.trajectory.resize(1);
        plan_response.trajectory[0] = res.result->planned_trajectory;
        plan_response.processing_time = {res.result->planning_time};
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "Failed");
    }
    get_result_handle_ = false;
  }
}

void CumotionMoveGroupClient::goalResponseCallback(const GoalHandle::SharedPtr & future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    result_ready = true;
    success = false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void CumotionMoveGroupClient::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback)
{
  std::string status = feedback->state;
  RCLCPP_INFO(node_->get_logger(), "Checking status");
  RCLCPP_INFO(node_->get_logger(), status.c_str());
}

void CumotionMoveGroupClient::resultCallback(const GoalHandle::WrappedResult & result)
{
  RCLCPP_INFO(node_->get_logger(), "Received result");

  result_ready = true;
  success = false;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  plan_response.error_code = result.result->error_code;
  if (plan_response.error_code.val == 1) {
    success = true;
    plan_response.trajectory_start = result.result->trajectory_start;
    plan_response.group_name = planning_request_.group_name;
    plan_response.trajectory = {result.result->planned_trajectory};
    plan_response.processing_time = {result.result->planning_time};
  }
}

}  // namespace manipulation
}  // namespace isaac
}  // namespace nvidia
