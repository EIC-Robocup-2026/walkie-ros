// Time parameterization treats the theta variable of a planar joint as a plain number,
// and OMPL returns it normalized to [-pi, pi]. A path that crosses the seam therefore
// jumps by almost 2 pi between two waypoints, and TOTG interpolates through that jump:
// the base turns the long way round while the plan still reports success.
//
// This adapter makes theta continuous along the path, so that it has to run before
// AddTimeOptimalParameterization. The values leave [-pi, pi], which is safe for what runs
// afterwards: the joint transform only needs the angle, joint goal constraints normalize
// it, and the trajectory message carries a quaternion. move_group loads it under
// base_mode:=whole_body only.
#include <cmath>
#include <string>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace openarm_bimanual_commander_cpp
{

class UnwrapPlanarHeading : public planning_interface::PlanningResponseAdapter
{
public:
  [[nodiscard]] std::string getDescription() const override
  {
    return "UnwrapPlanarHeading";
  }

  void adapt(
    const planning_scene::PlanningSceneConstPtr & /*planning_scene*/,
    const planning_interface::MotionPlanRequest & /*req*/,
    planning_interface::MotionPlanResponse & res) const override
  {
    if (!res.trajectory || res.trajectory->empty() || res.trajectory->getGroup() == nullptr) {
      return;
    }
    auto & trajectory = *res.trajectory;
    for (const auto * joint : trajectory.getGroup()->getActiveJointModels()) {
      if (joint->getType() != moveit::core::JointModel::PLANAR) {
        continue;
      }
      const int theta = trajectory.getRobotModel()->getVariableIndex(joint->getName() + "/theta");
      double previous = trajectory.getWayPointPtr(0)->getVariablePosition(theta);
      for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
        auto waypoint = trajectory.getWayPointPtr(i);
        previous += std::remainder(waypoint->getVariablePosition(theta) - previous, 2.0 * M_PI);
        waypoint->setVariablePosition(theta, previous);
      }
    }
  }
};

}  // namespace openarm_bimanual_commander_cpp

PLUGINLIB_EXPORT_CLASS(
  openarm_bimanual_commander_cpp::UnwrapPlanarHeading,
  planning_interface::PlanningResponseAdapter)
