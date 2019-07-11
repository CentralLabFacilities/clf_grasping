#include "clf_mtc_server/planning_scene.h"

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace ps
{
std::vector<moveit_msgs::AttachedCollisionObject> getAttachedObjects()
{
  ros::NodeHandle nh("~");

  ros::ServiceClient client_get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  client_get_scene.waitForExistence();

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES;
  client_get_scene.call(srv);
  auto scene = srv.response.scene;
  // ROS_INFO_STREAM("current scene: " << scene);

  auto attached_objects = scene.robot_state.attached_collision_objects;

  ROS_DEBUG_STREAM("Got Attached Objects: " << attached_objects.size());

  return attached_objects;
}

void detachObjects()
{
  ros::NodeHandle nh("~");
  ROS_DEBUG_STREAM("Detaching Objects...");

  ros::ServiceClient client_get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient client_apply_scene = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
  client_apply_scene.waitForExistence();
  client_get_scene.waitForExistence();

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES;
  client_get_scene.call(srv);
  auto scene = srv.response.scene;
  // ROS_INFO_STREAM("current scene: " << scene);

  auto attached_objects = scene.robot_state.attached_collision_objects;
  // auto objects = scene.world.collision_objects;

  moveit_msgs::PlanningScene update;
  update.is_diff = 1u;
  update.robot_state.is_diff = 1u;

  // to detach we have to also remove the removed objects from world?
  // http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html#detach-an-object-from-the-robot
  // yes, "planning_scene.world.collision_objects.push_back(attached_object.object);" still uses the remove operation
  // it does not detach and add to the world without the remove operation
  for (auto attached_object : attached_objects)
  {
    attached_object.object.operation = attached_object.object.REMOVE;
    update.robot_state.attached_collision_objects.push_back(attached_object);
    update.world.collision_objects.push_back(attached_object.object);
  }
  moveit_msgs::ApplyPlanningScene req;
  req.request.scene = update;
  // ROS_INFO_STREAM("sending scene: " << update);
  client_apply_scene.call(req);

  ros::spinOnce();
}

void clear()
{
  // moveit::planning_interface::PlanningSceneInterface psi;

  // // fetch all objects
  // auto objs = psi.getObjects();
  // std::vector<std::string> ids;
  // transform(objs.begin(), objs.end(), back_inserter(ids), [](auto val) { return val.first; });
  // for(auto s : ids) {
  //   ROS_INFO_STREAM("removing object " << s);
  // }

  // psi.removeCollisionObjects(ids);

  // ros::spinOnce();

  ros::NodeHandle nh("~");
  ROS_DEBUG_STREAM("Removing Objects...");

  ros::ServiceClient client_get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient client_apply_scene = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
  client_apply_scene.waitForExistence();
  client_get_scene.waitForExistence();

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES;
  client_get_scene.call(srv);
  auto scene = srv.response.scene;
  // ROS_INFO_STREAM("current scene: " << scene);
  auto objects = scene.world.collision_objects;

  moveit_msgs::PlanningScene update;
  update.is_diff = 1u;
  update.robot_state.is_diff = 1u;

  // to detach we have to also remove the removed objects from world?
  // http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html#detach-an-object-from-the-robot
  // yes, "planning_scene.world.collision_objects.push_back(attached_object.object);" still uses the remove operation
  // it does not detach and add to the world without the remove operation
  for (auto object : objects)
  {
    object.operation = object.REMOVE;
    update.world.collision_objects.push_back(object);
  }
  moveit_msgs::ApplyPlanningScene req;
  req.request.scene = update;
  // ROS_INFO_STREAM("sending scene: " << update);
  client_apply_scene.call(req);
}
}  // namespace ps