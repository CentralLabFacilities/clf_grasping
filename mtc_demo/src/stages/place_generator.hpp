#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit
{
namespace task_constructor
{
namespace stages
{
/** Simple IK pose generator to place an attached object in a specific pose
 *
 * The "pose" property, inherited from GeneratePose specifies the target pose
 * of the grasped object. This stage transforms this pose into a target pose for the ik_frame */
class PlaceGenerator : public GeneratePose
{
public:
  PlaceGenerator(const std::string& name = "place pose");

  void compute() override;

  void setObject(const std::string& object)
  {
    setProperty("object", object);
  }

protected:
  void onNewSolution(const SolutionBase& s) override;
};


std::ostream& operator<<(std::ostream& os, const Eigen::Vector3d& v)
{
    os << v.x() << " " << v.y() << " " << v.z();
    return os;
}
std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond& q)
{
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
    return os;
}
std::ostream& operator<<(std::ostream& os, const Eigen::Affine3d& pose)
{
    os << "(" << pose.translation() << ")[" << pose.rotation() << "]";
    return os;
}

PlaceGenerator::PlaceGenerator(const std::string& name) : GeneratePose(name)
{
  auto& p = properties();
  p.declare<std::string>("object");
  p.declare<geometry_msgs::PoseStamped>("ik_frame");
}

void PlaceGenerator::onNewSolution(const SolutionBase& s)
{
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

  const auto& props = properties();
  const std::string& object = props.get<std::string>("object");
  std::string msg;
  if (!scene->getCurrentState().hasAttachedBody(object))
    msg = "'" + object + "' is not an attached object";
  if (scene->getCurrentState().getAttachedBody(object)->getFixedTransforms().empty())
    msg = "'" + object + "' has no associated shapes";
  if (!msg.empty())
  {
    if (storeFailures())
    {
      InterfaceState state(scene);
      SubTrajectory solution;
      solution.markAsFailure();
      solution.setComment(msg);
      spawn(std::move(state), std::move(solution));
    }
    else
      ROS_WARN_STREAM_NAMED("PlaceGenerator", msg);
    return;
  }

  upstream_solutions_.push(&s);
}

void PlaceGenerator::compute()
{
  if (upstream_solutions_.empty())
    return;

     ROS_WARN_STREAM_NAMED("PlaceGenerator", "compute");

  const SolutionBase& s = *upstream_solutions_.pop();
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();
  const moveit::core::RobotState& robot_state = scene->getCurrentState();
  const auto& props = properties();

  const moveit::core::AttachedBody* object = robot_state.getAttachedBody(props.get<std::string>("object"));
  // current object_pose w.r.t. planning frame
  const Eigen::Isometry3d& orig_object_pose = object->getGlobalCollisionBodyTransforms()[0];

  const geometry_msgs::PoseStamped& pose_msg = props.get<geometry_msgs::PoseStamped>("pose");
  Eigen::Isometry3d target_pose;
  tf::poseMsgToEigen(pose_msg.pose, target_pose);
  // target pose w.r.t. planning frame
  scene->getTransforms().transformPose(pose_msg.header.frame_id, target_pose, target_pose);
  ROS_WARN_STREAM_NAMED("PlaceGenerator" , target_pose);

  const geometry_msgs::PoseStamped& ik_frame_msg = props.get<geometry_msgs::PoseStamped>("ik_frame");
  Eigen::Isometry3d ik_frame;
  tf::poseMsgToEigen(ik_frame_msg.pose, ik_frame);
  ik_frame = robot_state.getGlobalLinkTransform(ik_frame_msg.header.frame_id) * ik_frame;
  Eigen::Isometry3d object_to_ik = orig_object_pose.inverse() * ik_frame;

  // spawn the nominal target object pose, considering flip about z and rotations about z-axis
  auto spawner = [&s, &scene, &object_to_ik, this](const Eigen::Isometry3d& nominal, uint z_flips,
                                                   uint z_rotations = 10, int x_rotations = 0, int y_rotations = 0,
                                                   uint x_steps = 24, uint y_steps = 24) {
        ROS_WARN_STREAM_NAMED("PlaceGenerator", "spawn " << z_rotations << "," << x_rotations << "," << y_rotations);
    for (uint flip = 0; flip < z_flips; ++flip)
    {
      // flip about object's x-axis
      Eigen::Isometry3d object = nominal * Eigen::AngleAxisd(flip * M_PI, Eigen::Vector3d::UnitX());
      for (uint zr = 0; zr < z_rotations; ++zr)
      {
        for (int xr = -x_rotations; xr <= x_rotations; xr++)
        {
          for (int yr = -y_rotations; yr <= y_rotations; yr++)
          {
            // rotate object at target pose about world's z-axis
            Eigen::Vector3d pos = object.translation();
            object.pretranslate(-pos)
                .prerotate(Eigen::AngleAxisd(zr * 2. * M_PI / z_rotations, Eigen::Vector3d::UnitZ()))
                .prerotate(Eigen::AngleAxisd(xr * M_PI / x_steps , Eigen::Vector3d::UnitX()))
                .prerotate(Eigen::AngleAxisd(yr * M_PI / y_steps, Eigen::Vector3d::UnitY()))
                .pretranslate(pos);

            // target ik_frame's pose w.r.t. planning frame
            geometry_msgs::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = scene->getPlanningFrame();
            tf::poseEigenToMsg(object * object_to_ik, target_pose_msg.pose);

            InterfaceState state(scene);
            forwardProperties(*s.end(), state);  // forward properties from inner solutions
            state.properties().set("target_pose", target_pose_msg);

            SubTrajectory trajectory;
            int a = abs((int)zr)+1;
            int b = abs((int)xr)+1;
            int c = abs((int)yr)+1;
            trajectory.setCost( a*b*c );
            trajectory.setComment(std::to_string(xr) + ", " + std::to_string(yr) + ", " + std::to_string(zr));
            rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "place frame");

            spawn(std::move(state), std::move(trajectory));
          }
        }
      }
    }
  };

  if (object->getShapes().size() == 1)
  {
    switch (object->getShapes()[0]->type)
    {
      case shapes::CYLINDER:
        spawner(target_pose, 2);
        return;

      case shapes::BOX:
      {  // consider 180/90 degree rotations about z axis
        const double* dims = static_cast<const shapes::Box&>(*object->getShapes()[0]).size;
        spawner(target_pose, 2, (std::abs(dims[0] - dims[1]) < 1e-5) ? 4 : 2);
        return;
      }
      case shapes::SPHERE:  // keep original orientation and rotate about world's z
        target_pose.linear() = orig_object_pose.linear();
        spawner(target_pose, 1);
        return;
      default:
        ROS_WARN_STREAM_NAMED("PlaceGenerator", "mesh?, assuming cup/cylinder");
        spawner(target_pose, 1, 20, 1, 1,48,48);
    }
  }

  // any other case: only try given target pose
  spawner(target_pose, 1, 1);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
