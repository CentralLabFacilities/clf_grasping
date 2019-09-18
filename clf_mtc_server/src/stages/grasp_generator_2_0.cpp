#include <clf_mtc_server/stages/grasp_generator_2_0.h>

#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit {
namespace task_constructor {
namespace stages {

GraspGenerator::GraspGenerator(const std::string& name) : GeneratePose(name)
{
  auto& p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object");

  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");
}

void GraspGenerator::init(const moveit::core::RobotModelConstPtr& robot_model)
{
  InitStageException errors;
  try
  {
    GeneratePose::init(robot_model);
  }
  catch (InitStageException& e)
  {
    errors.append(e);
  }

  const auto& props = properties();

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

void GraspGenerator::onNewSolution(const SolutionBase& s)
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
      ROS_WARN_STREAM_NAMED("GraspGenerator", msg);
    return;
  }

  upstream_solutions_.push(&s);
}

void GraspGenerator::compute()
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
  target_pose_msg.header.frame_id = props.get<std::string>("object");//"base_footprint";

  moveit_msgs::CollisionObject object;
  scene->getCollisionObjectMsg(object, props.get<std::string>("object"));
  const uint8_t CYLINDER_CONST1 = 12; // How many steps around the cylinder
  const float DIST_TO_OBJ = 0.0;//0.22;
  const float MAX_GRIPPER_OPEN = 0.12;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    // TODO only upright cylinders? Should work with cylinders in any
    // orientation now, but more testing is needed
    // if(object.primitives[0].dimensions[0]<=MAX_GRIPPER_OPEN)//TODO
    if (2.0 * object.primitives[0].dimensions[1] <= MAX_GRIPPER_OPEN) { // Is it small enough to grasp?
      for (int a = 0; a < CYLINDER_CONST1; a++) {
        for (int b = -1; b < 2; b++) { // TODO do this in fixed steps instead, e.g. every 2 cm
          for (int c = 0; c < 2; c++) { // gripper 180 deg rotated
            float x, y, z;
            tf2::Quaternion /*pose, */grasp, total;
            //tf2::fromMsg(object.primitive_poses[0].orientation, pose);
            grasp.setRPY((c == 0 ? -M_PI / 2.0 : M_PI / 2.0), 0.0, 2.0 * M_PI / CYLINDER_CONST1 * a);
            total = /*pose **/ grasp; // TODO switch order?
            total.normalize();
            tf2::Vector3 vec(-(DIST_TO_OBJ + object.primitives[0].dimensions[1]), object.primitives[0].dimensions[0] / 3.0 * b, 0.0), vec2(1.0, 0.0, 0.0);
            tf2::Transform trans(total);
            vec = trans * vec;
            vec2 = trans * vec2;
            x = object.primitive_poses[0].position.x + vec.x();
            y = object.primitive_poses[0].position.y + vec.y();
            z = object.primitive_poses[0].position.z + vec.z();
            target_pose_msg.pose.orientation = tf2::toMsg(total);
            target_pose_msg.pose.position.x = x;
            target_pose_msg.pose.position.y = y;
            target_pose_msg.pose.position.z = z;

            InterfaceState state(scene);
            state.properties().set("target_pose", target_pose_msg);
            props.exposeTo(state.properties(), { "pregrasp", "grasp" });

            SubTrajectory trajectory;
            trajectory.setCost(0.0);
            trajectory.setComment(std::to_string(a) + ", " + std::to_string(b) + ", " + std::to_string(c));

            // add frame at target pose
            rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

            spawn(std::move(state), std::move(trajectory));
          }
        }
      }
    } else {
      ROS_ERROR("generateGrasps: Cylinder too big");
    }
  } else if (object.primitives[0].type == object.primitives[0].BOX) {
    for (int a = 0; a < 3; a++) { // Three dimensions
      if (object.primitives[0].dimensions[a] <= MAX_GRIPPER_OPEN) {
        for (int c = 0; c < 2; c++) { // Each side can have two grasp poses (90
                                      // degrees rotated) //TODO or four?
          for (int b = -1; b <= 1;
               b += 2) { // Two sides in each dimension: front/back, left/right,
                         // top/bottom
            for (int d = -1; d < 2; d++) {
              tf2::Quaternion /*pose,*/ grasp, total;
              //tf2::fromMsg(object.primitive_poses[0].orientation, pose);
              float alpha = 0.0, beta = 0.0;
              if (a == 0) {
                grasp.setEuler(M_PI / 2.0, 0.0,
                               (c == 0 && b == 1 ? M_PI : 0.0) +
                                   c * b * M_PI / 2.0);
                alpha = object.primitives[0].dimensions[c == 0 ? 2 : 1] / 2.0;
                beta = object.primitives[0].dimensions[c == 0 ? 1 : 2];
              } else if (a == 1) {
                grasp.setEuler((c == 0 && b == 1 ? M_PI : 0.0) +
                                   c * b * M_PI / 2.0,
                               M_PI / 2.0, 0.0);
                alpha = object.primitives[0].dimensions[c == 0 ? 0 : 2] / 2.0;
                beta = object.primitives[0].dimensions[c == 0 ? 2 : 0];
              } else if (a == 2) {
                grasp.setEuler(0.0, 0.0, (c == 0 && b == 1 ? M_PI : 0.0) +
                                             c * b * M_PI / 2.0);
                alpha = object.primitives[0].dimensions[c == 0 ? 0 : 1] / 2.0;
                beta = object.primitives[0].dimensions[c == 0 ? 1 : 0];
              }
              tf2::Vector3 vec(-(DIST_TO_OBJ + alpha), beta / 3.0 * d, 0.0),
                  vec2(1.0, 0.0, 0.0);
              total = /*pose **/ grasp; // TODO switch order?
              total.normalize();
              tf2::Transform trans(total);
              vec = trans * vec;
              vec2 = trans * vec2;
              target_pose_msg.pose.orientation = tf2::toMsg(total);
              target_pose_msg.pose.position.x = object.primitive_poses[0].position.x + vec.x();
              target_pose_msg.pose.position.y = object.primitive_poses[0].position.y + vec.y();
              target_pose_msg.pose.position.z = object.primitive_poses[0].position.z + vec.z();

              InterfaceState state(scene);
              state.properties().set("target_pose", target_pose_msg);
              props.exposeTo(state.properties(), { "pregrasp", "grasp" });

              SubTrajectory trajectory;
              trajectory.setCost(0.0);
              trajectory.setComment(std::to_string(a) + ", " + std::to_string(c) + ", " + std::to_string(b) + ", " + std::to_string(d));

              // add frame at target pose
              rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

              spawn(std::move(state), std::move(trajectory));
            }
          }
        }
      } else {
        printf("generateGrasps: Side %i of box too big\n", a);
      }
    }
  } else if (object.primitives[0].type ==
             object.primitives[0].SPHERE) { // TODO sphere
    if (2.0 * object.primitives[0].dimensions[0] <= MAX_GRIPPER_OPEN) {
      // regular dodecahedron (20 vertices)
      const float phi = 1.618; // golden ratio
      float vertices[20][3] = {
          {1, 1, 1},           {-1, 1, 1},         {1, -1, 1},
          {-1, -1, 1},         {1, 1, -1},         {-1, 1, -1},
          {1, -1, -1},         {-1, -1, -1},       {0, phi, 1 / phi},
          {0, -phi, 1 / phi},  {0, phi, -1 / phi}, {0, -phi, -1 / phi},
          {1 / phi, 0, phi},   {-1 / phi, 0, phi}, {1 / phi, 0, -phi},
          {-1 / phi, 0, -phi}, {phi, 1 / phi, 0},  {-phi, 1 / phi, 0},
          {phi, -1 / phi, 0},  {-phi, -1 / phi, 0}};
      for (int a = 0; a < 20; a++) {
        float x, y, z;
        Eigen::Quaternionf total;
        total.normalize();
        Eigen::Vector3f vec(vertices[a][0], vertices[a][1], vertices[a][2]);
        vec *= -(DIST_TO_OBJ + object.primitives[0].dimensions[1]) / sqrt(3.0);
        Eigen::Vector3f vec2 = -vec;
        total.setFromTwoVectors(Eigen::Vector3f(-1.0, 0.0, 0.0), vec);
        x = object.primitive_poses[0].position.x + vec.x();
        y = object.primitive_poses[0].position.y + vec.y();
        z = object.primitive_poses[0].position.z + vec.z();
        target_pose_msg.pose.orientation.x = total.x();
        target_pose_msg.pose.orientation.y = total.y();
        target_pose_msg.pose.orientation.z = total.z();
        target_pose_msg.pose.orientation.w = total.w();
        target_pose_msg.pose.position.x = x;
        target_pose_msg.pose.position.y = y;
        target_pose_msg.pose.position.z = z;

        InterfaceState state(scene);
        state.properties().set("target_pose", target_pose_msg);
        props.exposeTo(state.properties(), { "pregrasp", "grasp" });

        SubTrajectory trajectory;
        trajectory.setCost(0.0);
        trajectory.setComment(std::to_string(a));

        // add frame at target pose
        rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

        spawn(std::move(state), std::move(trajectory));
      }
    } else {
      ROS_ERROR("generateGrasps: Sphere too big");
    }
  } else {
    ROS_ERROR("generateGrasps cannot handle object type");
  }
}
}
}
}
