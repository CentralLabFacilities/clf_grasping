#include <clf_mtc_server/stages/generate_all_grasp_pose.h>

#include <moveit/task_constructor/storage.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

using namespace moveit::task_constructor;

GenerateAllGraspPose::GenerateAllGraspPose(const std::string& name) : stages::GeneratePose(name)
{
  auto& p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object");
  p.declare<double>("angle_delta", 0.1, "angular steps (rad)");

  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");
}

void GenerateAllGraspPose::init(const moveit::core::RobotModelConstPtr& robot_model)
{
  InitStageException errors;
  try
  {
    stages::GeneratePose::init(robot_model);
  }
  catch (InitStageException& e)
  {
    errors.append(e);
  }

  const auto& props = properties();

  // check angle_delta
  if (props.get<double>("angle_delta") == 0.)
    errors.push_back(*this, "angle_delta must be non-zero");

  // check availability of object
  props.get<std::string>("object");
  // check availability of eef
  const std::string& eef = props.get<std::string>("eef");
  if (!robot_model->hasEndEffector(eef))
    errors.push_back(*this, "unknown end effector: " + eef);
  else
  {
    // check availability of eef pose
    const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
    const std::string& name = props.get<std::string>("pregrasp");
    std::map<std::string, double> m;
    if (!jmg->getVariableDefaultPositions(name, m))
      errors.push_back(*this, "unknown end effector pose: " + name);
  }

  if (errors)
    throw errors;
}

void GenerateAllGraspPose::onNewSolution(const SolutionBase& s)
{
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

  const auto& props = properties();
  const std::string& object = props.get<std::string>("object");
  if (!scene->knowsFrameTransform(object))
  {
    const std::string msg = "object '" + object + "' not in scene";
    if (storeFailures())
    {
      InterfaceState state(scene);
      SubTrajectory solution;
      solution.markAsFailure();
      solution.setComment(msg);
      spawn(std::move(state), std::move(solution));
    }
    else
      ROS_WARN_STREAM_NAMED("GenerateAllGraspPose", msg);
    return;
  }

  upstream_solutions_.push(&s);
}

void GenerateAllGraspPose::compute()
{
  if (upstream_solutions_.empty())
    return;
  planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

  // set end effector pose
  const auto& props = properties();
  const std::string& eef = props.get<std::string>("eef");
  const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

  robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));

  geometry_msgs::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = props.get<std::string>("object");

  double current_angle_ = 0.0;
  double angle_delta = props.get<double>("angle_delta");

  for (double altitude = 0; altitude < 2. * M_PI; altitude += angle_delta)
  {
    for (double azimuth = 0; azimuth < 2. * M_PI; azimuth += angle_delta)
    {
      for (double roll = 0; roll < 2. * M_PI; roll += M_PI / 8)
      {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(altitude, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ());

        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.rotate(q);

        InterfaceState state(scene);
        tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
        state.properties().set("target_pose", target_pose_msg);
        props.exposeTo(state.properties(), { "pregrasp", "grasp" });

        SubTrajectory trajectory;
        trajectory.setCost(0.0);
        trajectory.setComment(std::to_string(roll) + ", " + std::to_string(altitude) + ", " + std::to_string(azimuth));

        // add frame at target pose
        rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

        spawn(std::move(state), std::move(trajectory));
      }
    }
  }
}
