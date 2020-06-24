#include <clf_mtc_server/stages/grasp_generator_2_0.h>

#include <moveit/task_constructor/storage.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

//#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

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
  ROS_DEBUG("GraspGenerator::onNewSolution() called");
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

  const auto& props = properties();
  const std::string& object = props.get<std::string>("object");
  if (!scene->knowsFrameTransform(object))
  {
    const std::string msg = "object '" + object + "' not in scene";
    ROS_WARN_STREAM_NAMED("GraspGenerator", msg);
    if (storeFailures())
    {
      InterfaceState state(scene);
      SubTrajectory solution;
      solution.markAsFailure();
      solution.setComment(msg);
      spawn(std::move(state), std::move(solution));
    }
    return;
  }

  upstream_solutions_.push(&s);
}

std::multimap<float, geometry_msgs::PoseStamped> computeCylinderGrasps(const moveit_msgs::CollisionObject& object, const float MAX_GRIPPER_OPEN) {
  ROS_INFO("Computing cylinder grasps");
  const uint8_t CYLINDER_CONST1 = 12; // How many steps around the cylinder
  //const float CYLINDER_CONST2 = 0.02; // Along height
  const float DIST_TO_OBJ = 0.0;//0.22;
  std::multimap<float, geometry_msgs::PoseStamped> grasps;
  if (2.0 * object.primitives[0].dimensions[1] <= MAX_GRIPPER_OPEN) { // Is it small enough to grasp?
    for (uint8_t a = 0; a < CYLINDER_CONST1; ++a) {
      for (int b = -1; b < 2; ++b) { // TODO do this in fixed steps instead, e.g. every 2 cm
        for (const bool c : {true, false}) { // gripper 180 deg rotated
          geometry_msgs::PoseStamped target_pose_msg;
          target_pose_msg.header.frame_id = object.id;//"base_footprint";
          tf2::Quaternion grasp;
          grasp.setRPY((c ? 0.0 : M_PI), 0.0, 2.0 * M_PI / CYLINDER_CONST1 * a);
          grasp.normalize();
          tf2::Vector3 position(-(DIST_TO_OBJ + object.primitives[0].dimensions[1]), 0.0, object.primitives[0].dimensions[0] / 3.0 * b * (c ? 1.0 : -1.0));
          position = tf2::Transform(grasp) * position;
          target_pose_msg.pose.orientation = tf2::toMsg(grasp);
          tf2::toMsg(position, target_pose_msg.pose.position);

          grasps.insert({std::abs(b), target_pose_msg}); // Favour grasps in the middle
        }
      }
    }
  } else {
    ROS_ERROR("generateGrasps: Cylinder is too big (diameter=%g)", 2.0 * object.primitives[0].dimensions[1]);
  }
  return grasps;
}

std::multimap<float, geometry_msgs::PoseStamped> computeBoxGrasps(const moveit_msgs::CollisionObject& object, const float MAX_GRIPPER_OPEN) {
  ROS_INFO("Computing box grasps");
  //const float DIST_TO_OBJ = 0.0;//0.22;
  std::multimap<float, geometry_msgs::PoseStamped> grasps;
  for (int a = 0; a < 3; ++a) { // Three dimensions
    //ROS_INFO("computeBoxGrasps: a=%i, dimension=%g", a, object.primitives[0].dimensions[a]);
    if (object.primitives[0].dimensions[a] <= MAX_GRIPPER_OPEN) {
      for (int c = 0; c < 2; ++c) { // Each side can have two grasp poses (90 degrees rotated)
        for (int b = -1; b <= 1; b += 2) { // Two sides in each dimension: front/back, left/right, top/bottom
          for (int d = -1; d < 2; ++d) { // translation
            for (const bool e : {true, false}) { // gripper 180 deg rotated around x-axis
                geometry_msgs::PoseStamped target_pose_msg;
                target_pose_msg.header.frame_id = object.id;//"base_footprint";
                tf2::Quaternion grasp;
                float alpha, beta;
                if (a == 0) {
                  grasp.setEuler(M_PI / 2.0, 0.0, (c == 0 && b == 1 ? M_PI : 0.0) + c * b * M_PI / 2.0);
                  alpha = object.primitives[0].dimensions[c == 0 ? 2 : 1] / 2.0;
                  beta = object.primitives[0].dimensions[c == 0 ? 1 : 2];
                } else if (a == 1) {
                  grasp.setEuler((c == 0 && b == 1 ? M_PI : 0.0) + c * b * M_PI / 2.0, M_PI / 2.0, 0.0);
                  alpha = object.primitives[0].dimensions[c == 0 ? 0 : 2] / 2.0;
                  beta = object.primitives[0].dimensions[c == 0 ? 2 : 0];
                } else if (a == 2) {
                  grasp.setEuler(0.0, 0.0, (c == 0 && b == 1 ? M_PI : 0.0) + c * b * M_PI / 2.0);
                  alpha = object.primitives[0].dimensions[c == 0 ? 0 : 1] / 2.0;
                  beta = object.primitives[0].dimensions[c == 0 ? 1 : 0];
                }
                tf2::Vector3 vec(-alpha, 0.0, beta / 3.0 * d);
                grasp = grasp * tf2::Quaternion(M_SQRT1_2, 0.0, 0.0, (e ? M_SQRT1_2 : -M_SQRT1_2));
                grasp.normalize();
                tf2::Transform trans(grasp);
                vec = trans * vec;
                target_pose_msg.pose.orientation = tf2::toMsg(grasp);
                tf2::toMsg(vec, target_pose_msg.pose.position);

                grasps.insert({(std::abs(d)+1.0)*object.primitives[0].dimensions[a]/beta, target_pose_msg}); // Favour grasps in the middle, favour grasps on small sides
            }
          }
        }
      }
    } else {
      ROS_WARN("generateGrasps: Side %i of box too big", a);
    }
  }
  return grasps;
}

std::multimap<float, geometry_msgs::PoseStamped> computeSphereGrasps(const moveit_msgs::CollisionObject& object, const float MAX_GRIPPER_OPEN) {
  ROS_INFO("Computing sphere grasps");
  const uint8_t SPHERE_CONST1 = 12; // How many steps around the sphere
  const uint8_t SPHERE_CONST2 = 2; // 2 or 4, rotations around x-axis of gripper
  float DIST_TO_OBJ = 0.03;
  std::multimap<float, geometry_msgs::PoseStamped> grasps;
  if (2.0 * object.primitives[0].dimensions[0] <= MAX_GRIPPER_OPEN) {
    /*// regular dodecahedron (20 vertices)
    const double phi = 1.618; // golden ratio
    const double vertices[20][3] = {
        { 1.0,  1.0,  1.0},           {-1.0,  1.0,  1.0},         { 1.0, -1.0,  1.0},
        {-1.0, -1.0,  1.0},         { 1.0,  1.0, -1.0},         {-1.0,  1.0, -1.0},
        { 1.0, -1.0, -1.0},         {-1.0, -1.0, -1.0},       { 0.0, phi,  1.0 / phi},
        { 0.0, -phi, 1.0 / phi},  { 0.0, phi, -1.0 / phi}, { 0.0, -phi, -1.0 / phi},
        { 1.0 / phi, 0.0, phi},   {-1.0 / phi,  0.0, phi}, { 1.0 / phi,  0.0, -phi},
        {-1.0 / phi, 0.0, -phi}, {phi, 1.0 / phi, 0.0},  {-phi, 1.0 / phi,  0.0},
        {phi, -1.0 / phi,  0.0},  {-phi, -1.0 / phi,  0.0}};
    for (uint8_t a = 0; a < 20; ++a) {
      Eigen::Vector3d vec(vertices[a][0] / sqrt(3.0), vertices[a][1] / sqrt(3.0), vertices[a][2] / sqrt(3.0));
      Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(-1.0, 0.0, 0.0), vec);
      vec *= (DIST_TO_OBJ + object.primitives[0].dimensions[0]);
      for (uint8_t b = 0; b < 6; ++b) {
        target_pose_msg.pose.orientation = tf2::toMsg(rotation * Eigen::AngleAxisd(b * 1.047197551, Eigen::Vector3d::UnitX()));
        target_pose_msg.pose.position = tf2::toMsg(vec);

        grasps.insert({0.0f, target_pose_msg});
      }
    }*/
    /*for (uint8_t a = 0; a < 200; ++a) {
      geometry_msgs::PoseStamped target_pose_msg;
      const double u1=drand48(), u2=drand48(), u3=drand48();
      tf2::Quaternion orientation(std::sqrt(1.0-u1)*std::sin(2.0*M_PI*u2), std::sqrt(1.0-u1)*std::cos(2.0*M_PI*u2), std::sqrt(u1)*std::sin(2.0*M_PI*u3), std::sqrt(u1)*std::cos(2.0*M_PI*u3)); // Random quaternion. Source: http://planning.cs.uiuc.edu/node198.html
      //DIST_TO_OBJ = 0.1*drand48();
      tf2::Vector3 position(-(DIST_TO_OBJ + object.primitives[0].dimensions[0]), 0.0, 0.0);
      position = tf2::Transform(orientation) * position;
      if(object.header.frame_id == "base_footprint") {
        target_pose_msg.header.frame_id = "base_footprint";
        if(position.z()<0.0) {
          continue;
        }
        position += tf2::Vector3(object.primitive_poses[0].position.x, object.primitive_poses[0].position.y, object.primitive_poses[0].position.z);
      } else {
        target_pose_msg.header.frame_id = object.id;
      }
      
      target_pose_msg.pose.orientation = tf2::toMsg(orientation);
      tf2::toMsg(position, target_pose_msg.pose.position);

      grasps.insert({DIST_TO_OBJ, target_pose_msg});
    }*/
    //for (float d = 0.0f; d < 0.07f; d+=0.01f) {
      for (uint8_t a = 0; a < SPHERE_CONST1 / 2; ++a) {
        for (uint8_t b = 0; b < SPHERE_CONST1; ++b) {
          for (uint8_t c = 0; c < SPHERE_CONST2; ++c) { // rotations around x-axis of gripper
            geometry_msgs::PoseStamped target_pose_msg;
            tf2::Quaternion orientation;
            orientation.setRPY(2.0 * M_PI / SPHERE_CONST2 * c, 2.0 * M_PI / SPHERE_CONST1 * b, 2.0 * M_PI / SPHERE_CONST1 * a);
            orientation.normalize(); // Probably not necessary?
            tf2::Vector3 position(-(DIST_TO_OBJ + object.primitives[0].dimensions[0]), 0.0, 0.0);
            position = tf2::Transform(orientation) * position;
            if(object.header.frame_id == "base_footprint") {
              target_pose_msg.header.frame_id = "base_footprint";
              if(position.z()<0.0) {
                continue;
              }
              position += tf2::Vector3(object.primitive_poses[0].position.x, object.primitive_poses[0].position.y, object.primitive_poses[0].position.z);
            } else {
              target_pose_msg.header.frame_id = object.id;
            }
            target_pose_msg.pose.orientation = tf2::toMsg(orientation);
            tf2::toMsg(position, target_pose_msg.pose.position);

            grasps.insert({position.length(), target_pose_msg}); // Grasps facing the robot are more likely to succeed
          }
        }
      }
    //}
  } else {
    ROS_ERROR("generateGrasps: Sphere too big");
  }
  ROS_INFO("Got %lu sphere grasps", grasps.size());
  return grasps;
}

void GraspGenerator::compute()
{
  ROS_DEBUG("GraspGenerator::compute() called");
  if (upstream_solutions_.empty())
    return;
  planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

  // set end effector pose
  const auto& props = properties();
  const std::string& eef = props.get<std::string>("eef");
  const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

  robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));

  moveit_msgs::CollisionObject object;
  scene->getCollisionObjectMsg(object, props.get<std::string>("object"));
  const float MAX_GRIPPER_OPEN = 0.12;
  if (object.primitives.size()==0) {
    ROS_ERROR("Cannot generate grasps because %s has no primitives", props.get<std::string>("object").c_str());
    return;
  }
  std::multimap<float, geometry_msgs::PoseStamped> poses_and_costs;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    poses_and_costs = computeCylinderGrasps(object, MAX_GRIPPER_OPEN);
  } else if (object.primitives[0].type == object.primitives[0].BOX) {
    poses_and_costs = computeBoxGrasps(object, MAX_GRIPPER_OPEN);
  } else if (object.primitives[0].type == object.primitives[0].SPHERE) {
    poses_and_costs = computeSphereGrasps(object, MAX_GRIPPER_OPEN);
  } else {
    ROS_ERROR("generateGrasps cannot handle object type (%u)", object.primitives[0].type);
    return;
  }
  for(const auto& item : poses_and_costs) {
    ROS_DEBUG("Adding target pose with costs %f", item.first);
    InterfaceState state(scene);
    state.properties().set("target_pose", item.second);
    props.exposeTo(state.properties(), { "pregrasp", "grasp" });

    SubTrajectory trajectory;
    trajectory.setCost(item.first);

    // add frame at target pose
    rviz_marker_tools::appendFrame(trajectory.markers(), item.second, 0.1, "grasp frame");

    spawn(std::move(state), std::move(trajectory));
  }
} // void GraspGenerator::compute()
} // stages
} // task_constructor
} // namespace moveit
