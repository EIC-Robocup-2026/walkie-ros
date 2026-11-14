// Planning benchmark for the whole-body MoveIt setup near a table.
//
// Runs in one process with the robot model, the IK plugins and the OMPL pipeline that
// move_group loads under base_mode:=whole_body. The obstacles are those of
// description/mujoco/scene_whole_body.xml, either as exact boxes or as an octomap
// built from their surfaces. The finger links carry the commander's collision padding
// (--finger-padding, 0.01 m by default). It measures:
//   - the cost of a collision check per scene,
//   - collision-aware IK the way /compute_ik runs it, call by call, with the reason
//     for each failure and the link pairs in contact,
//   - OMPL plans of left_arm_lift to joint goals, with 1, 4 and 10000 planning attempts,
//   - OMPL plans of left_arm_lift to pose goals, as the commander sends them, for several
//     planning times,
//   - the same pose goals solved IK first, as the commander's ik_first_enable does: IK,
//     then a plan to the joint solution, with the pose goal as the fallback,
//   - whole-body goals, following the steps of WholeBodyOption::moveTo.
// Launch it with run_planning_benchmark.launch.py in openarm_bimanual_moveit_config.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <octomap/OcTree.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
using BasePose = std::array<double, 3>;

double msSince(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct Stats
{
  size_t n{0};
  double mean{0.0}, median{0.0}, p90{0.0}, max{0.0};
};

Stats summarize(std::vector<double> values)
{
  Stats s;
  s.n = values.size();
  if (values.empty()) return s;
  std::sort(values.begin(), values.end());
  s.mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(s.n);
  const auto at = [&](double p) {
    const double idx = p * static_cast<double>(s.n - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    return values[lo] + (values[hi] - values[lo]) * (idx - static_cast<double>(lo));
  };
  s.median = at(0.5);
  s.p90 = at(0.9);
  s.max = values.back();
  return s;
}

std::string statsJson(const Stats & s)
{
  std::ostringstream o;
  o << "{\"n\":" << s.n << ",\"mean\":" << s.mean << ",\"median\":" << s.median << ",\"p90\":" << s.p90
    << ",\"max\":" << s.max << "}";
  return o.str();
}

std::string quote(const std::string & s)
{
  return "\"" + s + "\"";
}

// Obstacles of description/mujoco/scene_whole_body.xml: full sizes and centres in map.
// The free box is where the scene file places it at the start.
struct Box
{
  const char * name;
  double sx, sy, sz, x, y, z;
};
constexpr std::array<Box, 7> kSceneBoxes = {{
    {"table_top", 0.70, 1.20, 0.04, 1.90, 0.00, 0.72},
    {"table_leg_near_left", 0.06, 0.06, 0.70, 1.60, 0.55, 0.35},
    {"table_leg_near_right", 0.06, 0.06, 0.70, 1.60, -0.55, 0.35},
    {"table_leg_far_left", 0.06, 0.06, 0.70, 2.20, 0.55, 0.35},
    {"table_leg_far_right", 0.06, 0.06, 0.70, 2.20, -0.55, 0.35},
    {"box", 0.06, 0.06, 0.10, 1.75, 0.15, 0.79},
    {"wall", 0.10, 5.00, 2.00, 3.20, 0.00, 1.00},
}};
constexpr double kTableNearEdge = 1.55;
// Front face of the column collision box, ahead of base_footprint (base.urdf.xacro).
constexpr double kColumnFront = 0.02;

struct SceneSpec
{
  std::string name;  // empty, boxes, octomap:<resolution>
  std::string kind;
  double resolution{0.0};
};

SceneSpec parseScene(const std::string & text)
{
  SceneSpec spec{text, text, 0.0};
  const auto colon = text.find(':');
  if (colon != std::string::npos) {
    spec.kind = text.substr(0, colon);
    spec.resolution = std::stod(text.substr(colon + 1));
  }
  if (spec.kind != "empty" && spec.kind != "boxes" && spec.kind != "octomap") {
    throw std::invalid_argument("Unknown scene '" + text + "': use empty, boxes or octomap:<resolution>");
  }
  if (spec.kind == "octomap" && spec.resolution <= 0.0) {
    throw std::invalid_argument("Scene '" + text + "' needs a resolution, such as octomap:0.03");
  }
  return spec;
}

// Points on every face of every box, at half the voxel size, marked occupied. A camera
// sees fewer faces, but each face it sees becomes voxels the same way.
size_t applyScene(planning_scene::PlanningScene & scene, const SceneSpec & spec)
{
  auto world = scene.getWorldNonConst();
  world->clearObjects();
  if (spec.kind == "boxes") {
    for (const auto & b : kSceneBoxes) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(b.x, b.y, b.z);
      world->addToObject(b.name, std::make_shared<const shapes::Box>(b.sx, b.sy, b.sz), pose);
    }
    return kSceneBoxes.size();
  }
  if (spec.kind != "octomap") return 0;
  auto tree = std::make_shared<octomap::OcTree>(spec.resolution);
  const double step = spec.resolution / 2.0;
  for (const auto & b : kSceneBoxes) {
    const std::array<double, 3> c = {b.x, b.y, b.z};
    const std::array<double, 3> h = {b.sx / 2.0, b.sy / 2.0, b.sz / 2.0};
    for (int k = 0; k < 3; ++k) {
      const int u = (k + 1) % 3;
      const int v = (k + 2) % 3;
      const int nu = static_cast<int>(std::ceil(2.0 * h[u] / step));
      const int nv = static_cast<int>(std::ceil(2.0 * h[v] / step));
      for (const double sign : {-1.0, 1.0}) {
        for (int i = 0; i <= nu; ++i) {
          for (int j = 0; j <= nv; ++j) {
            std::array<double, 3> p{};
            p[k] = c[k] + sign * h[k];
            p[u] = c[u] - h[u] + std::min(2.0 * h[u], i * step);
            p[v] = c[v] - h[v] + std::min(2.0 * h[v], j * step);
            tree->updateNode(octomap::point3d(p[0], p[1], p[2]), true);
          }
        }
      }
    }
  }
  tree->updateInnerOccupancy();
  const size_t leaves = tree->getNumLeafNodes();
  world->addToObject(
      planning_scene::PlanningScene::OCTOMAP_NS,
      std::make_shared<const shapes::OcTree>(std::shared_ptr<const octomap::OcTree>(tree)),
      Eigen::Isometry3d::Identity());
  return leaves;
}

// Both arms and the lift at "home" (arms hanging, lift at the top), the right arm then
// set to the named state other_arm of right_arm.
moveit::core::RobotState homeState(
    const moveit::core::RobotModelConstPtr & model, const BasePose & base, const std::string & other_arm = "home")
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  if (const auto * both = model->getJointModelGroup("both_arms_lift")) state.setToDefaultValues(both, "home");
  if (const auto * right = model->getJointModelGroup("right_arm")) state.setToDefaultValues(right, other_arm);
  state.setVariablePosition("virtual_joint/x", base[0]);
  state.setVariablePosition("virtual_joint/y", base[1]);
  state.setVariablePosition("virtual_joint/theta", base[2]);
  state.update();
  return state;
}

void setBase(moveit::core::RobotState & state, const BasePose & base)
{
  state.setVariablePosition("virtual_joint/x", base[0]);
  state.setVariablePosition("virtual_joint/y", base[1]);
  state.setVariablePosition("virtual_joint/theta", base[2]);
  state.update();
}

struct Target
{
  std::string name;
  double x, y, z, roll, pitch, yaw;
};

Eigen::Isometry3d toPose(const Target & t)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  pose.linear() = (Eigen::AngleAxisd(t.yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(t.pitch, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(t.roll, Eigen::Vector3d::UnitX()))
                      .toRotationMatrix();
  return pose;
}

// Hand orientations: pointing forward as in the whole-body demo (roll 0 puts the wrist
// camera bracket under the hand), the same turned over (bracket on top), and pointing
// down. Roll alone cannot turn the hand over here: it turns about the hand's x axis,
// which points down when pitch is pi/2.
struct Orientation
{
  const char * name;
  double roll, pitch, yaw;
};
constexpr std::array<Orientation, 3> kOrientations = {{
    {"fwd", 0.0, M_PI / 2.0, 0.0},
    {"fwd_up", 0.0, -M_PI / 2.0, M_PI},  // the same direction, turned 180 deg about the approach axis
    {"down", 0.0, M_PI, 0.0},
}};

// How far the gripper's collision geometry reaches below the hand frame origin for a
// hand orientation, from the extents in the hand frame.
double reachBelow(const Orientation & o, const Eigen::Vector3d & lo, const Eigen::Vector3d & hi)
{
  const Eigen::Matrix3d r = (Eigen::AngleAxisd(o.yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(o.pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(o.roll, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();
  double below = 0.0;
  for (int c = 0; c < 8; ++c) {
    const Eigen::Vector3d corner(c & 1 ? hi.x() : lo.x(), c & 2 ? hi.y() : lo.y(), c & 4 ? hi.z() : lo.z());
    below = std::min(below, (r * corner).z());
  }
  return -below;
}

// Hand targets above the table, for the left arm. The height is set by the clearance
// between the table top and the lowest point of the gripper, so every target is
// collision free on its own and the result depends only on reaching it.
std::vector<Target> tableTargets(
    bool quick, const Eigen::Vector3d & lo, const Eigen::Vector3d & hi, const std::vector<double> & xs_override)
{
  constexpr double kTableTop = 0.74;
  std::vector<Target> out;
  const std::vector<double> xs = !xs_override.empty() ? xs_override
                                 : quick              ? std::vector<double>{1.62, 1.85}
                                                      : std::vector<double>{1.62, 1.75, 1.90};
  // y 0.30 keeps the gripper clear of the free box at y 0.15.
  const std::vector<double> ys = quick ? std::vector<double>{0.30} : std::vector<double>{0.0, 0.30};
  const std::vector<double> clearances = quick ? std::vector<double>{0.03} : std::vector<double>{0.03, 0.15};
  char name[64];
  for (const auto & o : kOrientations) {
    for (const double c : clearances) {
      const double z = kTableTop + reachBelow(o, lo, hi) + c;
      for (const double x : xs) {
        for (const double y : ys) {
          std::snprintf(name, sizeof(name), "%s_c%02.0f_%.2f_%.2f", o.name, c * 100.0, x, y);
          out.push_back({name, x, y, z, o.roll, o.pitch, o.yaw});
        }
      }
    }
  }
  return out;
}

// Base poses facing the table, named by the gap between the column's front face and the
// table's near edge.
struct BaseCase
{
  std::string name;
  BasePose pose;
};

std::vector<BaseCase> nearTableBases(bool quick)
{
  std::vector<BaseCase> out;
  const std::vector<double> gaps = quick ? std::vector<double>{0.20, 0.08} : std::vector<double>{0.40, 0.20, 0.08};
  char name[32];
  for (const double gap : gaps) {
    std::snprintf(name, sizeof(name), "gap_%.2f", gap);
    out.push_back({name, {kTableNearEdge - kColumnFront - gap, 0.0, 0.0}});
  }
  return out;
}

std::string contactPairs(
    const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state, const std::string & group)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 50;
  req.max_contacts_per_pair = 1;
  req.group_name = group;
  collision_detection::CollisionResult res;
  scene.checkCollision(req, res, state);
  std::set<std::string> pairs;
  for (const auto & [names, contacts] : res.contacts) {
    std::string a = names.first;
    std::string b = names.second;
    if (b < a) std::swap(a, b);
    pairs.insert(a + "|" + b);
  }
  std::string out;
  for (const auto & p : pairs) out += (out.empty() ? "" : ";") + p;
  return out;
}

// Mirrors WholeBodyOption::goalHasClearance: the whole robot is collision free at the
// goal and with the lift 1 cm lower.
bool goalClear(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState state, std::string * pairs = nullptr)
{
  state.update();
  if (scene.isStateColliding(state)) {
    if (pairs) *pairs = contactPairs(scene, state, "");
    return false;
  }
  const auto & bounds = state.getRobotModel()->getVariableBounds("lift_joint");
  const double lift = state.getVariablePosition("lift_joint");
  state.setVariablePosition("lift_joint", std::max(lift - 0.01, bounds.min_position_));
  state.update();
  if (scene.isStateColliding(state)) {
    if (pairs) *pairs = "lift-1cm:" + contactPairs(scene, state, "");
    return false;
  }
  return true;
}

enum class CallResult { kOk, kNoKinematicSolution, kGroupCollision, kClearance };

const char * toString(CallResult r)
{
  switch (r) {
    case CallResult::kOk: return "ok";
    case CallResult::kNoKinematicSolution: return "no_kinematic_solution";
    case CallResult::kGroupCollision: return "ik_solution_in_collision";
    case CallResult::kClearance: return "goal_clearance";
  }
  return "?";
}

struct IkCall
{
  CallResult result;
  double ms;
  std::string pairs;
};

// One /compute_ik call with avoid_collisions: one setFromIK, with the group-only
// collision check of move_group's kinematics service as the validity callback, then
// the commander's full-robot clearance check.
IkCall ikCall(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * jmg, const Eigen::Isometry3d & pose, const std::string & tip)
{
  bool callback_ran = false;
  std::string rejected_pairs;
  const moveit::core::GroupStateValidityCallbackFn validity =
      [&](moveit::core::RobotState * s, const moveit::core::JointModelGroup * g, const double * values) {
        callback_ran = true;
        s->setJointGroupPositions(g, values);
        s->update();
        const bool colliding = scene.isStateColliding(*s, g->getName());
        if (colliding) rejected_pairs = contactPairs(scene, *s, g->getName());
        return !colliding;
      };
  const auto start = Clock::now();
  const bool solved = state.setFromIK(jmg, pose, tip, 0.0, validity);
  IkCall call{CallResult::kOk, msSince(start), ""};
  if (!solved) {
    call.result = callback_ran ? CallResult::kGroupCollision : CallResult::kNoKinematicSolution;
    call.pairs = rejected_pairs;
    return call;
  }
  const auto check_start = Clock::now();
  std::string pairs;
  if (!goalClear(scene, state, &pairs)) {
    call.result = CallResult::kClearance;
    call.pairs = pairs;
  }
  call.ms += msSince(check_start);
  return call;
}

// The seeds the commander uses, then random ones. Call 0 is the current state, call 1
// the named home state without the lift (WholeBodyOption::solveForArmAlone).
void seedFor(
    int call, moveit::core::RobotState & state, const moveit::core::RobotState & current,
    const moveit::core::JointModelGroup * jmg, const std::map<std::string, double> & home_seed,
    random_numbers::RandomNumberGenerator & rng)
{
  std::vector<double> values;
  current.copyJointGroupPositions(jmg, values);
  state.setJointGroupPositions(jmg, values);
  if (call < 4) {
    if (call % 2 == 1) {
      for (const auto & [name, value] : home_seed) state.setVariablePosition(name, value);
    }
  } else {
    state.setToRandomPositions(jmg, rng);
  }
  state.update();
}

struct IkSequence
{
  std::vector<IkCall> calls;
  int first_ok{-1};
  std::vector<double> solution;  // left_arm_lift variables of the first usable solution
};

IkSequence ikSequence(
    const planning_scene::PlanningScene & scene, const moveit::core::RobotState & current,
    const moveit::core::JointModelGroup * jmg, const Eigen::Isometry3d & pose, const std::string & tip,
    const std::map<std::string, double> & home_seed, int max_calls, random_numbers::RandomNumberGenerator & rng)
{
  IkSequence seq;
  for (int i = 0; i < max_calls; ++i) {
    moveit::core::RobotState state(current);
    seedFor(i, state, current, jmg, home_seed, rng);
    seq.calls.push_back(ikCall(scene, state, jmg, pose, tip));
    if (seq.calls.back().result == CallResult::kOk) {
      seq.first_ok = i;
      state.copyJointGroupPositions(jmg, seq.solution);
      break;
    }
  }
  return seq;
}

struct PlanOutcome
{
  bool ok{false};
  int code{0};
  double wall_ms{0.0};
  double planning_time_ms{0.0};
  size_t waypoints{0};
  double base_travel{0.0};
  double joint_path{0.0};  // sum of the joint-space steps of the group's non-planar joints, rad and m
  double duration_s{0.0};  // after time parameterization at full speed
};

PlanOutcome plan(
    const planning_pipeline::PlanningPipeline & pipeline, const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & start, const moveit_msgs::msg::Constraints & goal, const std::string & group,
    double seconds, int attempts, const Eigen::Vector2d & ws_min, const Eigen::Vector2d & ws_max)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = group;
  req.pipeline_id = "ompl";
  req.planner_id = "RRTConnect";
  req.allowed_planning_time = seconds;
  req.num_planning_attempts = attempts;
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  moveit::core::robotStateToRobotStateMsg(start, req.start_state);
  req.goal_constraints.push_back(goal);
  req.workspace_parameters.header.frame_id = start.getRobotModel()->getModelFrame();
  req.workspace_parameters.min_corner.x = ws_min.x();
  req.workspace_parameters.min_corner.y = ws_min.y();
  req.workspace_parameters.min_corner.z = -1.0;
  req.workspace_parameters.max_corner.x = ws_max.x();
  req.workspace_parameters.max_corner.y = ws_max.y();
  req.workspace_parameters.max_corner.z = 3.0;

  planning_interface::MotionPlanResponse res;
  const auto t0 = Clock::now();
  const bool ok = pipeline.generatePlan(scene, req, res);
  PlanOutcome out;
  out.wall_ms = msSince(t0);
  out.code = res.error_code.val;
  out.ok = ok && res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  out.planning_time_ms = res.planning_time * 1000.0;
  if (res.trajectory) {
    out.waypoints = res.trajectory->getWayPointCount();
    out.duration_s = res.trajectory->getDuration();
    std::vector<std::string> joints;
    for (const auto & name : start.getRobotModel()->getJointModelGroup(group)->getVariableNames()) {
      if (name.rfind("virtual_joint", 0) != 0) joints.push_back(name);
    }
    for (size_t i = 1; i < out.waypoints; ++i) {
      const auto & a = res.trajectory->getWayPoint(i - 1);
      const auto & b = res.trajectory->getWayPoint(i);
      out.base_travel += std::hypot(
          b.getVariablePosition("virtual_joint/x") - a.getVariablePosition("virtual_joint/x"),
          b.getVariablePosition("virtual_joint/y") - a.getVariablePosition("virtual_joint/y"));
      double step = 0.0;
      for (const auto & j : joints) step += std::pow(b.getVariablePosition(j) - a.getVariablePosition(j), 2);
      out.joint_path += std::sqrt(step);
    }
  }
  return out;
}

// A joint goal: the planner starts with the goal state in hand.
PlanOutcome plan(
    const planning_pipeline::PlanningPipeline & pipeline, const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & start, const moveit::core::RobotState & goal, const std::string & group,
    double seconds, int attempts, const Eigen::Vector2d & ws_min, const Eigen::Vector2d & ws_max)
{
  return plan(
      pipeline, scene, start,
      kinematic_constraints::constructGoalConstraints(goal, start.getRobotModel()->getJointModelGroup(group), 0.001),
      group, seconds, attempts, ws_min, ws_max);
}

// A pose goal for `link` the way MoveGroupInterface::setPoseTarget builds it, with the
// commander's goal tolerance (setGoalTolerance(0.02)). The planner has to sample goal
// states by IK within the planning time.
moveit_msgs::msg::Constraints poseGoal(
    const moveit::core::RobotModelConstPtr & model, const std::string & link, const Eigen::Isometry3d & pose)
{
  constexpr double kGoalTolerance = 0.02;
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = model->getModelFrame();
  stamped.pose.position.x = pose.translation().x();
  stamped.pose.position.y = pose.translation().y();
  stamped.pose.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  stamped.pose.orientation.x = q.x();
  stamped.pose.orientation.y = q.y();
  stamped.pose.orientation.z = q.z();
  stamped.pose.orientation.w = q.w();
  return kinematic_constraints::constructGoalConstraints(link, stamped, kGoalTolerance, kGoalTolerance);
}

// Copy of WholeBodyOption::sampleBasePoses (two tiers of 40, nearest first per tier).
std::vector<BasePose> sampleBasePoses(
    const Eigen::Isometry3d & goal, const BasePose & base, double shoulder_x, double shoulder_y, std::mt19937 & rng)
{
  const Eigen::Vector3d hand_z = goal.linear().col(2);
  const double approach = std::hypot(hand_z.x(), hand_z.y()) > 0.3
                              ? std::atan2(hand_z.y(), hand_z.x())
                              : std::atan2(goal.translation().y() - base[1], goal.translation().x() - base[0]);
  const double side = shoulder_y >= 0.0 ? 1.0 : -1.0;
  struct Tier
  {
    double reach_min, reach_max, around_min, around_max, turn_min, turn_max;
  };
  const std::array<Tier, 2> tiers = {{{0.30, 0.60, -0.4, 0.8, -1.0, 0.0}, {0.20, 0.70, -0.8, 0.8, -1.5, 1.5}}};
  std::vector<BasePose> samples;
  for (const auto & range : tiers) {
    std::uniform_real_distribution<double> reach(range.reach_min, range.reach_max);
    std::uniform_real_distribution<double> around(range.around_min, range.around_max);
    std::uniform_real_distribution<double> turn(range.turn_min, range.turn_max);
    const auto tier = static_cast<std::ptrdiff_t>(samples.size());
    for (int i = 0; i < 40; ++i) {
      const double r = reach(rng);
      const double a = approach + side * around(rng);
      const double yaw = approach + side * turn(rng);
      samples.push_back(
          {goal.translation().x() - r * std::cos(a) - (std::cos(yaw) * shoulder_x - std::sin(yaw) * shoulder_y),
           goal.translation().y() - r * std::sin(a) - (std::sin(yaw) * shoulder_x + std::cos(yaw) * shoulder_y),
           std::remainder(yaw, 2.0 * M_PI)});
    }
    std::sort(samples.begin() + tier, samples.end(), [&](const BasePose & a, const BasePose & b) {
      return std::hypot(a[0] - base[0], a[1] - base[1]) < std::hypot(b[0] - base[0], b[1] - base[1]);
    });
  }
  return samples;
}

bool nearJointLimit(const moveit::core::RobotState & state, const moveit::core::JointModelGroup * jmg)
{
  const double margin = 0.01;
  for (const auto & name : jmg->getVariableNames()) {
    if (name == "lift_joint") continue;
    const auto & bounds = state.getRobotModel()->getVariableBounds(name);
    const double value = state.getVariablePosition(name);
    if (bounds.position_bounded_ &&
        (value < bounds.min_position_ + margin || value > bounds.max_position_ - margin)) {
      return true;
    }
  }
  return false;
}

struct WholeBodyOutcome
{
  std::string result;  // arm_only, base_moved, no_base_pose, plan_failed
  double total_ms{0.0}, ik_ms{0.0}, plan_ms{0.0};
  int ik_calls{0}, samples_tried{0}, samples_with_ik{0}, samples_on_limit{0}, samples_clear{0}, plans_tried{0};
  double base_travel{0.0};
  std::map<std::string, int> rejections;  // failure reason or contact pair -> count
};

// The steps of WholeBodyOption::moveTo for one goal, in process. reseeds > 0 retries a
// failed IK call that many times with random seeds, which /compute_ik with TRAC-IK
// 2.0.2 does not do.
WholeBodyOutcome wholeBodyGoal(
    const planning_pipeline::PlanningPipeline & pipeline, const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & current, const Eigen::Isometry3d & goal, const std::string & body_group,
    const std::string & arm_group, const std::string & tip, const std::string & shoulder,
    const std::map<std::string, double> & home_seed, double planning_time, int reseeds,
    random_numbers::RandomNumberGenerator & rng, std::mt19937 & sampler_rng)
{
  WholeBodyOutcome out;
  const auto t0 = Clock::now();
  const auto & model = current.getRobotModel();
  const auto * arm = model->getJointModelGroup(arm_group);
  const BasePose base = {
      current.getVariablePosition("virtual_joint/x"), current.getVariablePosition("virtual_joint/y"),
      current.getVariablePosition("virtual_joint/theta")};

  // Without `deferred`, a solution has to pass goalClear() too. With it, the first IK
  // solution ends the search even if it fails goalClear(), and *deferred holds that call,
  // so the base sampling can count in the commander's order: IK solution, joint limit,
  // clearance.
  const auto solve = [&](const moveit::core::RobotState & at, int first_seed, moveit::core::RobotState & result,
                         IkCall * deferred) {
    for (int k = 0; k <= reseeds; ++k) {
      moveit::core::RobotState state(at);
      seedFor(k == 0 ? first_seed : 4, state, at, arm, home_seed, rng);
      const auto call = ikCall(*scene, state, arm, goal, tip);
      ++out.ik_calls;
      out.ik_ms += call.ms;
      if (call.result == CallResult::kOk || (deferred && call.result == CallResult::kClearance)) {
        if (deferred) *deferred = call;
        result = state;
        return true;
      }
      out.rejections[std::string(toString(call.result)) + (call.pairs.empty() ? "" : " " + call.pairs)]++;
    }
    return false;
  };

  // Arm alone: four calls, current state and home seed alternating.
  for (int i = 0; i < 4; ++i) {
    moveit::core::RobotState solution(current);
    if (!solve(current, i, solution, nullptr)) continue;
    const auto p0 = Clock::now();
    const auto outcome = plan(
        pipeline, scene, current, solution, arm_group, planning_time, 1, Eigen::Vector2d(base[0] - 1.0, base[1] - 1.0),
        Eigen::Vector2d(base[0] + 1.0, base[1] + 1.0));
    out.plan_ms += msSince(p0);
    ++out.plans_tried;
    if (outcome.ok) {
      out.result = "arm_only";
      out.total_ms = msSince(t0);
      return out;
    }
    break;  // the commander moves on to base sampling after one failed arm plan
  }

  Eigen::Isometry3d shoulder_in_base = current.getGlobalLinkTransform("base_footprint").inverse() *
                                       current.getGlobalLinkTransform(shoulder);
  const auto samples = sampleBasePoses(
      goal, base, shoulder_in_base.translation().x(), shoulder_in_base.translation().y(), sampler_rng);
  int solved = 0;
  for (const auto & sample : samples) {
    ++out.samples_tried;
    moveit::core::RobotState at(current);
    setBase(at, sample);
    moveit::core::RobotState solution(at);
    // WholeBodyOption::planWithSampledBase seeds every sample with the home seed.
    IkCall ik{};
    if (!solve(at, 1, solution, &ik)) continue;
    ++out.samples_with_ik;
    if (nearJointLimit(solution, arm)) {
      ++out.samples_on_limit;
      continue;
    }
    if (ik.result == CallResult::kClearance) {
      out.rejections[std::string(toString(ik.result)) + (ik.pairs.empty() ? "" : " " + ik.pairs)]++;
      continue;
    }
    ++out.samples_clear;
    ++solved;
    const Eigen::Vector2d lo(std::min(base[0], sample[0]), std::min(base[1], sample[1]));
    const Eigen::Vector2d hi(std::max(base[0], sample[0]), std::max(base[1], sample[1]));
    const auto p0 = Clock::now();
    auto outcome = plan(
        pipeline, scene, current, solution, body_group, std::min(2.0, planning_time), 1,
        lo - Eigen::Vector2d(0.3, 0.3), hi + Eigen::Vector2d(0.3, 0.3));
    ++out.plans_tried;
    if (!outcome.ok) {
      outcome = plan(
          pipeline, scene, current, solution, body_group, planning_time, 1, lo - Eigen::Vector2d(5.0, 5.0),
          hi + Eigen::Vector2d(5.0, 5.0));
      ++out.plans_tried;
    }
    out.plan_ms += msSince(p0);
    if (outcome.ok) {
      out.result = "base_moved";
      out.base_travel = outcome.base_travel;
      out.total_ms = msSince(t0);
      return out;
    }
    out.rejections["plan_failed code " + std::to_string(outcome.code)]++;
    if (solved >= 5) break;
  }
  out.result = solved == 0 ? "no_base_pose" : "plan_failed";
  out.total_ms = msSince(t0);
  return out;
}

// Bounding box of the collision meshes of the given links, in the frame of `frame`.
std::pair<Eigen::Vector3d, Eigen::Vector3d> collisionExtents(
    const moveit::core::RobotState & state, const std::vector<std::string> & links, const std::string & frame)
{
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d hi = Eigen::Vector3d::Constant(-1e9);
  const Eigen::Isometry3d to_frame = state.getGlobalLinkTransform(frame).inverse();
  for (const auto & name : links) {
    const auto * link = state.getRobotModel()->getLinkModel(name);
    if (!link) continue;
    const auto & shapes = link->getShapes();
    const auto & origins = link->getCollisionOriginTransforms();
    for (size_t i = 0; i < shapes.size(); ++i) {
      const Eigen::Isometry3d t = to_frame * state.getGlobalLinkTransform(link) * origins[i];
      std::vector<Eigen::Vector3d> points;
      if (const auto * mesh = dynamic_cast<const shapes::Mesh *>(shapes[i].get())) {
        for (unsigned v = 0; v < mesh->vertex_count; ++v) {
          points.emplace_back(mesh->vertices[3 * v], mesh->vertices[3 * v + 1], mesh->vertices[3 * v + 2]);
        }
      } else if (const auto * box = dynamic_cast<const shapes::Box *>(shapes[i].get())) {
        for (int c = 0; c < 8; ++c) {
          points.emplace_back(
              (c & 1 ? 0.5 : -0.5) * box->size[0], (c & 2 ? 0.5 : -0.5) * box->size[1],
              (c & 4 ? 0.5 : -0.5) * box->size[2]);
        }
      }
      for (const auto & p : points) {
        const Eigen::Vector3d q = t * p;
        lo = lo.cwiseMin(q);
        hi = hi.cwiseMax(q);
      }
    }
  }
  return {lo, hi};
}

std::string mapJson(const std::map<std::string, int> & m)
{
  std::ostringstream o;
  o << "{";
  bool first = true;
  for (const auto & [k, v] : m) {
    o << (first ? "" : ",") << quote(k) << ":" << v;
    first = false;
  }
  o << "}";
  return o.str();
}

std::vector<std::string> split(const std::string & text, char sep)
{
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, sep)) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

// Everything that holds the node lives in here, so it is gone before rclcpp::shutdown().
// Zenoh aborts the process when a session is closed from a static destructor.
int run(int argc, char ** argv)
{
  // The same options as move_group, so the pipeline and the IK plugins read the
  // parameters the launch file passes in the same way.
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("planning_benchmark", options);

  std::string output = "planning_benchmark.json";
  std::string scenes_arg = "empty,boxes,octomap:0.03,octomap:0.02";
  std::string sections = "geometry,collision,ik,arm_plan,attempts,pose_plan,ik_first,whole_body";
  int ik_calls = 16;
  int plan_trials = 3;
  int collision_states = 2000;
  int wb_reseeds = 0;
  double planning_time = 5.0;
  bool quick = false;
  std::string other_arms_arg = "home,standby";
  int attempt_goals = 4;
  std::string attempt_counts_arg = "1,4,10000";
  std::string target_xs_arg;  // empty: the default target rows
  std::string pose_times_arg = "1.5,5";
  // Planning times for the joint goal of the ik_first section. Its fallback pose goal
  // plans for --planning-time, like the commander's arm_planning_time.
  std::string ik_first_times_arg = "0.5,1,1.5";
  int pose_goal_count = 6;
  int pose_repeats = 4;
  // The commander's finger_padding default (commander_template.cpp).
  double finger_padding = 0.01;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (arg == "--output") output = next();
    else if (arg == "--scenes") scenes_arg = next();
    else if (arg == "--sections") sections = next();
    else if (arg == "--ik-calls") ik_calls = std::stoi(next());
    else if (arg == "--plan-trials") plan_trials = std::stoi(next());
    else if (arg == "--collision-states") collision_states = std::stoi(next());
    else if (arg == "--wb-reseeds") wb_reseeds = std::stoi(next());
    else if (arg == "--planning-time") planning_time = std::stod(next());
    else if (arg == "--quick") quick = true;
    else if (arg == "--other-arms") other_arms_arg = next();
    else if (arg == "--attempt-goals") attempt_goals = std::stoi(next());
    else if (arg == "--attempt-counts") attempt_counts_arg = next();
    else if (arg == "--target-xs") target_xs_arg = next();
    else if (arg == "--pose-times") pose_times_arg = next();
    else if (arg == "--ik-first-times") ik_first_times_arg = next();
    else if (arg == "--pose-goals") pose_goal_count = std::stoi(next());
    else if (arg == "--pose-repeats") pose_repeats = std::stoi(next());
    else if (arg == "--finger-padding") finger_padding = std::stod(next());
  }
  const auto other_arms = split(other_arms_arg, ',');
  std::vector<int> attempt_counts;
  for (const auto & count : split(attempt_counts_arg, ',')) attempt_counts.push_back(std::stoi(count));
  std::vector<double> pose_times;
  for (const auto & t : split(pose_times_arg, ',')) pose_times.push_back(std::stod(t));
  std::vector<double> ik_first_times;
  for (const auto & t : split(ik_first_times_arg, ',')) ik_first_times.push_back(std::stod(t));
  const auto section_list = split(sections, ',');
  const auto has = [&](const std::string & s) {
    return std::find(section_list.begin(), section_list.end(), s) != section_list.end();
  };
  for (const auto & s : section_list) {
    static const std::vector<std::string> known = {
        "geometry", "collision", "ik", "arm_plan", "attempts", "pose_plan", "ik_first", "whole_body"};
    if (std::find(known.begin(), known.end(), s) == known.end()) {
      RCLCPP_FATAL(node->get_logger(), "Unknown section '%s'", s.c_str());
      return 1;
    }
  }
  // arm_plan and attempts plan to the IK solutions of the ik section, and pose_plan and
  // ik_first to their hand poses, so without it they would print nothing, or only the
  // launch-test goal.
  if ((has("arm_plan") || has("attempts")) && !has("ik")) {
    RCLCPP_FATAL(node->get_logger(), "The arm_plan and attempts sections need the ik section: add ik to --sections");
    return 1;
  }
  if ((has("pose_plan") || has("ik_first")) && !has("ik")) {
    RCLCPP_WARN(
        node->get_logger(), "pose_plan and ik_first without the ik section plan only the launch tests' near arm target");
  }

  robot_model_loader::RobotModelLoader loader(node, robot_model_loader::RobotModelLoader::Options("robot_description"));
  const auto model = loader.getModel();
  if (!model) {
    RCLCPP_FATAL(node->get_logger(), "No robot model");
    return 1;
  }
  if (!model->hasJointModelGroup("whole_body_left") ||
      !model->getJointModelGroup("whole_body_left")->hasJointModel("virtual_joint") ||
      model->getJointModel("virtual_joint")->getType() != moveit::core::JointModel::PLANAR) {
    RCLCPP_FATAL(node->get_logger(), "The SRDF needs base_mode:=whole_body (a planar virtual_joint)");
    return 1;
  }
  auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(model, node, "ompl");
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  // The commander pads the finger links (applyFingerPadding in commander_template.cpp),
  // so plans near a surface keep that clearance. Apply it the same way, as a scene diff.
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    for (const char * link : {"openarm_left_left_finger", "openarm_left_right_finger", "openarm_right_left_finger",
                              "openarm_right_right_finger"}) {
      moveit_msgs::msg::LinkPadding padding;
      padding.link_name = link;
      padding.padding = finger_padding;
      diff.link_padding.push_back(padding);
    }
    scene->setPlanningSceneDiffMsg(diff);
  }

  const std::string arm_group = "left_arm_lift";
  const std::string body_group = "whole_body_left";
  const std::string tip = "openarm_left_hand";
  const std::string shoulder = "openarm_left_link1";
  const auto * arm = model->getJointModelGroup(arm_group);

  std::map<std::string, double> home_seed;
  {
    moveit::core::RobotState s(model);
    s.setToDefaultValues(arm, "home");
    for (const auto & name : arm->getVariableNames()) {
      if (name != "lift_joint") home_seed[name] = s.getVariablePosition(name);
    }
  }

  const auto param = [&](const std::string & name) {
    rclcpp::Parameter p;
    return node->get_parameter(name, p) ? p.value_to_string() : std::string("(unset)");
  };
  std::ostringstream config;
  config << "{\"solve_type\":" << quote(param("robot_description_kinematics." + arm_group + ".solve_type"))
         << ",\"ik_timeout\":" << quote(param("robot_description_kinematics." + arm_group + ".kinematics_solver_timeout"))
         << ",\"segment_fraction_arm\":"
         << quote(param("ompl." + arm_group + ".longest_valid_segment_fraction"))
         << ",\"segment_fraction_body\":"
         << quote(param("ompl." + body_group + ".longest_valid_segment_fraction"))
         << ",\"planning_time\":" << planning_time << ",\"ik_calls\":" << ik_calls << ",\"quick\":" << quick
         << ",\"pose_times\":" << quote(pose_times_arg) << ",\"ik_first_times\":" << quote(ik_first_times_arg)
         << ",\"pose_repeats\":" << pose_repeats
         << ",\"finger_padding\":" << scene->getCollisionEnv()->getLinkPadding("openarm_left_left_finger") << "}";
  std::cout << "config " << config.str() << std::endl;

  std::vector<SceneSpec> scenes;
  for (const auto & s : split(scenes_arg, ',')) scenes.push_back(parseScene(s));

  const std::vector<std::string> gripper = {
      "openarm_left_hand", "openarm_left_left_finger", "openarm_left_right_finger", "openarm_left_wristcam_bracket"};
  const auto [grip_lo, grip_hi] = collisionExtents(homeState(model, {0.0, 0.0, 0.0}), gripper, tip);
  std::vector<double> target_xs;
  for (const auto & x : split(target_xs_arg, ',')) target_xs.push_back(std::stod(x));
  const auto targets = tableTargets(quick, grip_lo, grip_hi, target_xs);
  const auto bases = nearTableBases(quick);

  std::ostringstream json;
  json << "{\"config\":" << config.str();

  // Scene-free geometry: how far the gripper reaches below the hand frame, and how low the
  // lift can go before the arms, held in a named state, touch the robot.
  if (has("geometry")) {
    moveit::core::RobotState state = homeState(model, {0.0, 0.0, 0.0});
    for (const auto & link : gripper) {
      const auto [lo, hi] = collisionExtents(state, {link}, tip);
      std::printf(
          "%-30s in %s: x %6.3f..%6.3f  y %6.3f..%6.3f  z %6.3f..%6.3f\n", link.c_str(), tip.c_str(), lo.x(), hi.x(),
          lo.y(), hi.y(), lo.z(), hi.z());
    }
    json << ",\"gripper_extents\":{\"min\":[" << grip_lo.x() << "," << grip_lo.y() << "," << grip_lo.z()
         << "],\"max\":[" << grip_hi.x() << "," << grip_hi.y() << "," << grip_hi.z() << "]}";
    for (const auto & o : kOrientations) {
      std::printf(
          "  %-8s hand: gripper reaches %.3f m below the hand frame origin\n", o.name, reachBelow(o, grip_lo, grip_hi));
    }

    // Self-collision only, which MoveIt checks without link padding, so this scene needs none.
    planning_scene::PlanningScene empty(model);
    const auto & bounds = model->getVariableBounds("lift_joint");
    json << ",\"lift_sweep\":[";
    bool first = true;
    for (const auto & other : other_arms) {
      moveit::core::RobotState s = homeState(model, {0.0, 0.0, 0.0}, other);
      s.setToDefaultValues(model->getJointModelGroup("left_arm"), other);
      s.update();
      // Where the arms hang at the home lift height: the front and the bottom of the
      // envelope decide how close to a table edge the base can stand.
      std::vector<std::string> arm_links;
      for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
        if (link->getName().rfind("openarm_", 0) == 0) arm_links.push_back(link->getName());
      }
      const auto [arm_lo, arm_hi] = collisionExtents(s, arm_links, "base_footprint");
      std::printf(
          "arms at %-8s (lift %.3f) in base_footprint: x %6.3f..%6.3f  y %6.3f..%6.3f  z %6.3f..%6.3f\n",
          other.c_str(), s.getVariablePosition("lift_joint"), arm_lo.x(), arm_hi.x(), arm_lo.y(), arm_hi.y(),
          arm_lo.z(), arm_hi.z());
      double lowest_free = bounds.max_position_;
      std::string first_contact;
      for (double q = bounds.max_position_; q >= bounds.min_position_ - 1e-9; q -= 0.005) {
        s.setVariablePosition("lift_joint", q);
        s.update();
        if (empty.isStateColliding(s)) {
          first_contact = contactPairs(empty, s, "");
          break;
        }
        lowest_free = q;
      }
      s.setVariablePosition("lift_joint", lowest_free);
      s.update();
      const double shoulder_z = s.getGlobalLinkTransform(shoulder).translation().z();
      std::printf(
          "lift sweep, both arms at %-8s: free down to lift %.3f (shoulder z %.3f), then %s\n",
          other.c_str(), lowest_free, shoulder_z, first_contact.empty() ? "free to the bottom" : first_contact.c_str());
      json << (first ? "" : ",") << "{\"other_arm\":" << quote(other) << ",\"lowest_free\":" << lowest_free
           << ",\"shoulder_z\":" << shoulder_z << ",\"contact\":" << quote(first_contact) << ",\"envelope_min\":["
           << arm_lo.x() << "," << arm_lo.y() << "," << arm_lo.z() << "],\"envelope_max\":[" << arm_hi.x() << ","
           << arm_hi.y() << "," << arm_hi.z() << "]}";
      first = false;
    }
    json << "]";
  }

  json << ",\"scenes\":[";
  bool first_scene = true;
  for (const auto & spec : scenes) {
    const size_t objects = applyScene(*scene, spec);
    std::cout << "\n=== scene " << spec.name << " (" << objects << (spec.kind == "octomap" ? " voxels" : " objects")
              << ") ===" << std::endl;
    json << (first_scene ? "" : ",") << "{\"scene\":" << quote(spec.name) << ",\"objects\":" << objects;
    first_scene = false;

    // Collision check cost: random arm and lift poses with the base 0.20 m from the table.
    if (has("collision")) {
      random_numbers::RandomNumberGenerator rng(42);
      const auto * both = model->getJointModelGroup("both_arms_lift");
      moveit::core::RobotState state = homeState(model, {kTableNearEdge - kColumnFront - 0.20, 0.0, 0.0});
      std::vector<double> full_us, group_us;
      int colliding = 0;
      for (int i = 0; i < collision_states; ++i) {
        state.setToRandomPositions(both, rng);
        state.update();
        auto t = Clock::now();
        colliding += scene->isStateColliding(state) ? 1 : 0;
        full_us.push_back(msSince(t) * 1000.0);
        t = Clock::now();
        std::ignore = scene->isStateColliding(state, arm_group);
        group_us.push_back(msSince(t) * 1000.0);
      }
      const auto full = summarize(full_us);
      const auto group = summarize(group_us);
      std::printf(
          "collision  full robot %.1f us mean, %.1f p90 | %s only %.1f us | %d/%d states colliding\n", full.mean,
          full.p90, arm_group.c_str(), group.mean, colliding, collision_states);
      json << ",\"collision\":{\"full_us\":" << statsJson(full) << ",\"group_us\":" << statsJson(group)
           << ",\"colliding\":" << colliding << ",\"states\":" << collision_states << "}";
    }

    // IK call by call, then arm plans to the first usable solution.
    std::vector<std::pair<std::string, std::pair<moveit::core::RobotState, moveit::core::RobotState>>> arm_goals;
    if (has("ik")) {
      random_numbers::RandomNumberGenerator rng(7);
      json << ",\"ik\":[";
      bool first_ik = true;
      for (const auto & other : other_arms) for (const auto & base_case : bases) {
        const BaseCase base{base_case.name + "/right_" + other, base_case.pose};
        const auto current = homeState(model, base.pose, other);
        int ok_first = 0, ok_commander = 0, ok_any = 0;
        std::vector<double> ms_to_ok, ms_per_call;
        std::map<std::string, int> reasons;
        for (const auto & target : targets) {
          const auto seq = ikSequence(*scene, current, arm, toPose(target), tip, home_seed, ik_calls, rng);
          double elapsed = 0.0;
          for (const auto & c : seq.calls) {
            elapsed += c.ms;
            ms_per_call.push_back(c.ms);
            if (c.result != CallResult::kOk) {
              reasons[std::string(toString(c.result)) + (c.pairs.empty() ? "" : " " + c.pairs)]++;
            }
          }
          if (seq.first_ok == 0) ++ok_first;
          if (seq.first_ok >= 0 && seq.first_ok < 4) ++ok_commander;
          if (seq.first_ok >= 0) {
            ++ok_any;
            ms_to_ok.push_back(elapsed);
            moveit::core::RobotState goal(current);
            goal.setJointGroupPositions(arm, seq.solution);
            goal.update();
            arm_goals.push_back({base.name + "/" + target.name, {current, goal}});
          }
          json << (first_ik ? "" : ",") << "{\"base\":" << quote(base.name) << ",\"target\":" << quote(target.name)
               << ",\"first_ok\":" << seq.first_ok << ",\"calls\":[";
          for (size_t k = 0; k < seq.calls.size(); ++k) {
            json << (k ? "," : "") << "{\"r\":" << quote(toString(seq.calls[k].result)) << ",\"ms\":" << seq.calls[k].ms
                 << ",\"pairs\":" << quote(seq.calls[k].pairs) << "}";
          }
          json << "]}";
          first_ik = false;
        }
        const auto per_call = summarize(ms_per_call);
        const auto to_ok = summarize(ms_to_ok);
        std::printf(
            "ik %-20s targets %2zu | first call ok %2d | commander's 4 calls ok %2d | within %d calls ok %2d | %.1f "
            "ms per call | %.0f ms median to a usable solution\n",
            base.name.c_str(), targets.size(), ok_first, ok_commander, ik_calls, ok_any, per_call.mean,
            to_ok.median);
        std::vector<std::pair<int, std::string>> top;
        for (const auto & [k, v] : reasons) top.push_back({v, k});
        std::sort(top.rbegin(), top.rend());
        for (size_t k = 0; k < std::min<size_t>(top.size(), 5); ++k) {
          std::printf("      %4d x %.220s\n", top[k].first, top[k].second.c_str());
        }
      }
      json << "]";
    }

    if (has("arm_plan") && !arm_goals.empty()) {
      std::vector<double> wall;
      int ok = 0, total = 0;
      std::map<int, int> codes;
      for (const auto & [name, pair] : arm_goals) {
        for (int t = 0; t < plan_trials; ++t) {
          const auto out = plan(
              *pipeline, scene, pair.first, pair.second, arm_group, planning_time, 1, Eigen::Vector2d(-5.0, -5.0),
              Eigen::Vector2d(5.0, 5.0));
          ++total;
          if (out.ok) {
            ++ok;
            wall.push_back(out.wall_ms);
          } else {
            codes[out.code]++;
            std::printf(
                "      %s: code %d after %.0f ms, start %s\n", name.c_str(), out.code, out.wall_ms,
                scene->isStateColliding(pair.first) ? contactPairs(*scene, pair.first, "").c_str() : "free");
          }
        }
      }
      const auto s = summarize(wall);
      std::printf(
          "arm_plan   %s, 1 attempt: %d/%d ok | %.0f ms mean, %.0f median, %.0f p90, %.0f max\n", arm_group.c_str(),
          ok, total, s.mean, s.median, s.p90, s.max);
      for (const auto & [code, n] : codes) std::printf("      %4d x MoveIt code %d\n", n, code);
      json << ",\"arm_plan\":{\"ok\":" << ok << ",\"total\":" << total << ",\"wall_ms\":" << statsJson(s) << "}";
    }

    // The commander's arm groups ask for 10000 planning attempts (commander_template.cpp).
    if (has("attempts") && !arm_goals.empty()) {
      json << ",\"attempts\":[";
      bool first_a = true;
      // The same goals for every attempt count, spread over the list, and only goals whose
      // start is free: a start in collision fails every count alike. Repeats only where a
      // run is short.
      std::vector<const std::pair<moveit::core::RobotState, moveit::core::RobotState> *> plannable;
      for (const auto & entry : arm_goals) {
        if (!scene->isStateColliding(entry.second.first)) plannable.push_back(&entry.second);
      }
      const size_t goals = std::min<size_t>(plannable.size(), static_cast<size_t>(std::max(attempt_goals, 0)));
      std::printf("attempts   %zu goals of %zu with a free start\n", goals, plannable.size());
      for (const int attempts : attempt_counts) {
        std::vector<double> wall, path, duration;
        int ok = 0, total = 0;
        const int repeats = attempts > 4 ? 1 : 3;
        for (size_t g = 0; g < goals; ++g) {
          const auto & pair = *plannable[g * plannable.size() / goals];
          for (int r = 0; r < repeats; ++r) {
            const auto out = plan(
                *pipeline, scene, pair.first, pair.second, arm_group, planning_time, attempts,
                Eigen::Vector2d(-5.0, -5.0), Eigen::Vector2d(5.0, 5.0));
            ++total;
            if (out.ok) {
              ++ok;
              wall.push_back(out.wall_ms);
              path.push_back(out.joint_path);
              duration.push_back(out.duration_s);
            }
          }
        }
        const auto s = summarize(wall);
        const auto p = summarize(path);
        const auto d = summarize(duration);
        std::printf(
            "attempts   %5d: %d/%d ok | %.0f ms mean, %.0f max | joint path %.2f mean, %.2f max | motion %.2f s mean\n",
            attempts, ok, total, s.mean, s.max, p.mean, p.max, d.mean);
        json << (first_a ? "" : ",") << "{\"attempts\":" << attempts << ",\"ok\":" << ok << ",\"total\":" << total
             << ",\"wall_ms\":" << statsJson(s) << ",\"joint_path\":" << statsJson(p)
             << ",\"duration_s\":" << statsJson(d) << "}";
        first_a = false;
      }
      json << "]";
    }

    // Pose goals for pose_plan and ik_first: the launch tests' near target, and the hand
    // poses of IK solutions found above, from starts that are collision free.
    struct PoseCase
    {
      std::string name;
      moveit::core::RobotState start;
      Eigen::Isometry3d pose;
    };
    std::vector<PoseCase> pose_cases;
    if (has("pose_plan") || has("ik_first")) {
      // NEAR_ARM_TARGET of the commander's launch tests (test/whole_body_harness.py), in
      // base_footprint, with the base at the origin, away from the table.
      const Target near_arm{"test_near_arm", 0.45, 0.25, 1.00, 0.0, M_PI / 2.0, 0.0};
      for (const auto & other : other_arms) {
        pose_cases.push_back(
            {near_arm.name + "/right_" + other, homeState(model, {0.0, 0.0, 0.0}, other), toPose(near_arm)});
      }
      std::vector<const std::pair<std::string, std::pair<moveit::core::RobotState, moveit::core::RobotState>> *> free;
      for (const auto & entry : arm_goals) {
        if (!scene->isStateColliding(entry.second.first)) free.push_back(&entry);
      }
      const size_t goals = std::min<size_t>(free.size(), static_cast<size_t>(std::max(pose_goal_count, 0)));
      for (size_t g = 0; g < goals; ++g) {
        const auto & entry = *free[g * free.size() / goals];
        pose_cases.push_back({entry.first, entry.second.first, entry.second.second.getGlobalLinkTransform(tip)});
      }
    }
    const Eigen::Vector2d ws_min(-5.0, -5.0), ws_max(5.0, 5.0);

    // Pose goals with the commander's 10000 attempts. A plan fails when no IK goal state
    // turns up within the planning time, which the joint goals above never need.
    if (has("pose_plan")) {
      json << ",\"pose_plan\":[";
      bool first_p = true;
      for (const double seconds : pose_times) {
        int ok = 0, total = 0;
        std::map<int, int> codes;
        std::vector<double> wall, duration;
        for (const auto & c : pose_cases) {
          int case_ok = 0;
          std::vector<double> case_wall, case_duration;
          for (int r = 0; r < pose_repeats; ++r) {
            const auto out =
                plan(*pipeline, scene, c.start, poseGoal(model, tip, c.pose), arm_group, seconds, 10000, ws_min, ws_max);
            ++total;
            case_wall.push_back(out.wall_ms);
            if (out.ok) {
              ++ok;
              ++case_ok;
              case_duration.push_back(out.duration_s);
            } else {
              codes[out.code]++;
            }
          }
          wall.insert(wall.end(), case_wall.begin(), case_wall.end());
          duration.insert(duration.end(), case_duration.begin(), case_duration.end());
          if (case_ok < pose_repeats) std::printf("      %.1f s %s: %d/%d ok\n", seconds, c.name.c_str(), case_ok, pose_repeats);
          json << (first_p ? "" : ",") << "{\"planning_time\":" << seconds << ",\"goal\":" << quote(c.name)
               << ",\"ok\":" << case_ok << ",\"total\":" << pose_repeats
               << ",\"wall_ms\":" << statsJson(summarize(case_wall))
               << ",\"duration_s\":" << statsJson(summarize(case_duration)) << "}";
          first_p = false;
        }
        const auto w = summarize(wall);
        const auto d = summarize(duration);
        std::printf(
            "pose_plan  %s, %.1f s, 10000 attempts: %d/%d ok | %.0f ms median, %.0f max | motion %.2f s mean\n",
            arm_group.c_str(), seconds, ok, total, w.median, w.max, d.mean);
        for (const auto & [code, n] : codes) std::printf("      %4d x MoveIt code %d\n", n, code);
      }
      json << "]";
    }

    // The same goals the way the commander plans them with ik_first_enable: up to four
    // IK calls with its seeds and the full-robot clearance check, then a joint goal with
    // 10000 attempts, and the pose goal for --planning-time if either step fails. Like the
    // commander, a joint goal time at or above --planning-time plans for --planning-time.
    if (has("ik_first")) {
      random_numbers::RandomNumberGenerator rng(13);
      json << ",\"ik_first\":[";
      bool first_f = true;
      for (const double requested : ik_first_times) {
        const double seconds = std::min(requested, planning_time);
        int by_ik = 0, by_fallback = 0, total = 0, no_ik = 0;
        std::vector<double> total_ms, ik_ms, duration;
        for (const auto & c : pose_cases) {
          int case_ik = 0, case_fallback = 0, case_no_ik = 0;
          std::vector<double> case_total, case_duration;
          for (int r = 0; r < pose_repeats; ++r) {
            const auto t0 = Clock::now();
            const auto seq = ikSequence(*scene, c.start, arm, c.pose, tip, home_seed, 4, rng);
            ik_ms.push_back(msSince(t0));
            PlanOutcome out;
            if (seq.first_ok >= 0) {
              moveit::core::RobotState goal(c.start);
              goal.setJointGroupPositions(arm, seq.solution);
              goal.update();
              out = plan(*pipeline, scene, c.start, goal, arm_group, seconds, 10000, ws_min, ws_max);
            } else {
              ++case_no_ik;
            }
            if (out.ok) {
              ++case_ik;
            } else {
              out = plan(
                  *pipeline, scene, c.start, poseGoal(model, tip, c.pose), arm_group, planning_time, 10000, ws_min,
                  ws_max);
              if (out.ok) ++case_fallback;
            }
            ++total;
            case_total.push_back(msSince(t0));
            if (out.ok) case_duration.push_back(out.duration_s);
          }
          by_ik += case_ik;
          by_fallback += case_fallback;
          no_ik += case_no_ik;
          total_ms.insert(total_ms.end(), case_total.begin(), case_total.end());
          duration.insert(duration.end(), case_duration.begin(), case_duration.end());
          if (case_ik < pose_repeats) {
            std::printf(
                "      %.1f s %s: %d/%d by IK (%d without an IK solution), %d by the pose goal\n", seconds,
                c.name.c_str(), case_ik, pose_repeats, case_no_ik, case_fallback);
          }
          json << (first_f ? "" : ",") << "{\"planning_time\":" << requested << ",\"joint_goal_time\":" << seconds
               << ",\"fallback_time\":" << planning_time
               << ",\"goal\":" << quote(c.name) << ",\"by_ik\":" << case_ik << ",\"no_ik\":" << case_no_ik
               << ",\"by_fallback\":" << case_fallback << ",\"total\":" << pose_repeats
               << ",\"total_ms\":" << statsJson(summarize(case_total))
               << ",\"duration_s\":" << statsJson(summarize(case_duration)) << "}";
          first_f = false;
        }
        const auto t = summarize(total_ms);
        const auto i = summarize(ik_ms);
        const auto d = summarize(duration);
        std::printf(
            "ik_first   %s, joint goal %.1f s: %d/%d by IK, %d by the pose goal, %d failed (%d without an IK "
            "solution) | %.0f ms median, %.0f max | IK %.0f ms median | motion %.2f s mean\n",
            arm_group.c_str(), seconds, by_ik, total, by_fallback, total - by_ik - by_fallback, no_ik, t.median,
            t.max, i.median, d.mean);
      }
      json << "]";
    }

    // Whole-body goals from a start 1 m before the table, as the commander runs them.
    if (has("whole_body")) {
      json << ",\"whole_body\":[";
      bool first_wb = true;
      std::vector<Target> wb_targets;
      for (const auto & t : targets) {
        if (t.name.find("_c03_") != std::string::npos) wb_targets.push_back(t);
      }
      std::vector<int> reseed_passes = {0};
      if (wb_reseeds > 0) reseed_passes.push_back(wb_reseeds);
      for (const auto & other : other_arms) for (const int reseeds : reseed_passes) {
        const auto current = homeState(model, {0.55, 0.0, 0.0}, other);
        random_numbers::RandomNumberGenerator rng(11);
        std::mt19937 sampler_rng(5);
        std::map<std::string, int> results;
        std::vector<double> total_ms, ik_ms, plan_ms;
        std::printf("whole_body right arm %s, reseeds %d\n", other.c_str(), reseeds);
        for (const auto & target : wb_targets) {
          const auto out = wholeBodyGoal(
              *pipeline, scene, current, toPose(target), body_group, arm_group, tip, shoulder, home_seed,
              planning_time, reseeds, rng, sampler_rng);
          results[out.result]++;
          total_ms.push_back(out.total_ms);
          ik_ms.push_back(out.ik_ms);
          plan_ms.push_back(out.plan_ms);
          json << (first_wb ? "" : ",") << "{\"other_arm\":" << quote(other) << ",\"reseeds\":" << reseeds
               << ",\"target\":" << quote(target.name)
               << ",\"result\":" << quote(out.result) << ",\"total_ms\":" << out.total_ms << ",\"ik_ms\":" << out.ik_ms
               << ",\"plan_ms\":" << out.plan_ms << ",\"ik_calls\":" << out.ik_calls
               << ",\"samples_tried\":" << out.samples_tried << ",\"samples_with_ik\":" << out.samples_with_ik
               << ",\"samples_on_limit\":" << out.samples_on_limit << ",\"plans_tried\":" << out.plans_tried
               << ",\"base_travel\":" << out.base_travel << ",\"rejections\":" << mapJson(out.rejections) << "}";
          first_wb = false;
          std::printf(
              "  %-22s %-12s %6.0f ms (ik %5.0f, plan %5.0f) ik calls %3d, samples %2d with ik %2d, base %.2f m\n",
              target.name.c_str(), out.result.c_str(), out.total_ms, out.ik_ms, out.plan_ms, out.ik_calls,
              out.samples_tried, out.samples_with_ik, out.base_travel);
        }
        const auto t = summarize(total_ms);
        std::string tally;
        for (const auto & [k, v] : results) tally += " " + k + "=" + std::to_string(v);
        std::printf("  summary:%s | %.0f ms median, %.0f max\n", tally.c_str(), t.median, t.max);
      }
      json << "]";
    }
    json << "}";
  }
  json << "]}";

  std::ofstream file(output);
  file << json.str() << std::endl;
  file.close();
  if (!file) {
    RCLCPP_ERROR(node->get_logger(), "Could not write %s", output.c_str());
    return 1;
  }
  std::cout << "\nWrote " << output << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const int rc = run(argc, argv);
  rclcpp::shutdown();
  return rc;
}
