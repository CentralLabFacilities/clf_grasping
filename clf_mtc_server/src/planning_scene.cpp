#include "clf_mtc_server/planning_scene.h"

#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

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