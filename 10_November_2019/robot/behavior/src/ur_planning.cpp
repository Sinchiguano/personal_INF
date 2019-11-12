#include <iostream>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Pose.h>

#include <shape_msgs/Mesh.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <ur_msgs/SetIO.h>

#include "ur_planning.h"

#include "log_publisher.hpp"

#include <fstream>

inline void delay( unsigned long ms ) {
    usleep( ms * 1000 );
}

void UR_planning::setForceMode(int fm){
  ROS_INFO("Set force mode = %d", fm);

  if (fm == 1){
    force_mode = true;
  }
  else {
    force_mode = false;
  }

  force_mode_slow = fm;

  if(force_mode){
    LoggerPub log("Set_force_mode ON");
    ROS_INFO("Set force mode on and remove box bottom from planning");
    box5.operation = box5.REMOVE;
    p_scene.world.collision_objects.push_back(box5);

    p_scene.is_diff = true;
    planning_scene_diff_publisher.publish(p_scene);
  }
  else {
    LoggerPub log("Set_force_mode OFF");
    ROS_INFO("Set force mode off and include box bottom back to planning");
    box5.operation = box5.ADD;
    p_scene.world.collision_objects.push_back(box5);

    p_scene.is_diff = true;
    planning_scene_diff_publisher.publish(p_scene);
  }



}

void UR_planning::wrenchCallback(geometry_msgs::WrenchStamped msg){
  if(!force_mode){
    return;
  }

  float force_sum = fabs(msg.wrench.force.x) + fabs(msg.wrench.force.y) + fabs(msg.wrench.force.z);

  if(fabs(msg.wrench.force.z) > 70 || force_sum > 120){
    ROS_INFO("Force stop - triggered");
    ROS_INFO("Force sum = %f", force_sum);
    ROS_INFO("Wrench force(x,y,z) = %f, %f, %f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    group.stop();
    force_mode = false;
    std::stringstream ss;
    ss << "Force_mode_stop_triggered Wrench_force_x_y_z "
      << msg.wrench.force.x << " "
      << msg.wrench.force.y << " "
      << msg.wrench.force.z;
    LoggerPub log(ss.str());
  }
}

void UR_planning::poseLog(sensor_msgs::JointState msg){
  if (log_counter%50 == 0){
    std::stringstream ss;
    ss << "JOINT_STATES " <<  std::to_string(msg.position[0]) << " "
        <<  std::to_string(msg.position[1]) << " "
        <<  std::to_string(msg.position[2]) << " "
        <<  std::to_string(msg.position[3]) << " "
        <<  std::to_string(msg.position[4]) << " "
        <<  std::to_string(msg.position[5]);
    LoggerPub log(ss.str(), 0);
  }
  log_counter++;
}

void UR_planning::vacStat(ur_msgs::IOStates msg){
  io_states = msg;
}



UR_planning::UR_planning()
: group("manipulator")
{

  std::cout << "Hybrid initialize" << std::endl;
  p_scene.name = "motion_planning_scene";

  planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  client = n.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");

//KPIECEkConfigDefault
// - allows planning with the "limited" parameter set to false
// - enabling the use of the robot from any starting position
  group.setPlannerId("LBKPIECEkConfigDefault");
  //group.setPlannerId("RRTstarkConfigDefault");
  group.setPlanningTime(6);

  //change by cesar sinchiguano
  //group.setMaxVelocityScalingFactor(1);
  //group.setMaxAccelerationScalingFactor(0.38);

  group.setMaxVelocityScalingFactor(0.08);
  group.setMaxAccelerationScalingFactor(0.04);

  force_mode = false;
  force_mode_slow = 0;
  sub_wrench = n.subscribe("/wrench",1, &UR_planning::wrenchCallback, this);

  //logging
  sub_pose = n.subscribe("/joint_states",1, &UR_planning::poseLog, this);

  sub_vacuum = n.subscribe("/ur_driver/io_states",1, &UR_planning::vacStat, this);

ROS_INFO("UR_Planning: ON");

}


int UR_planning::initPlanningScene()
{

  double centerX = 0.775;
  double centerY = 0.105;
  double phi = 0.0;

  //box:
  //width: 1.00
  //lenght: 0.60
  //thickness: 0.04


  ROS_INFO("Create the collision objects in the environment");

  //Useful objects
  moveit_msgs::CollisionObject box1, box2, box3, box4, floor, table, shelf, line, line2, line3;
  //box1.link_name = box2.link_name = box3.link_name = box4.link_name = box5.link_name = "base_link";
  box1.header.frame_id = box2.header.frame_id = box3.header.frame_id = box4.header.frame_id = box5.header.frame_id
    = floor.header.frame_id = table.header.frame_id = shelf.header.frame_id
    = line.header.frame_id = line2.header.frame_id = line3.header.frame_id = "base_link";
  box1.header.stamp = box2.header.stamp = box3.header.stamp = box4.header.stamp = box5.header.stamp
     = floor.header.stamp = table.header.stamp = shelf.header.stamp = line.header.stamp = line2.header.stamp = line3.header.stamp = ros::Time::now();
  box1.id = "box1";
  box2.id = "box2";
  box3.id = "box3";
  box4.id = "box4";
  box5.id = "box5";
  floor.id = "floor";
  table.id = "table";
  shelf.id = "shelf";
  line.id = "line";
  line2.id = "line2";
  line3.id = "line3";

    //Obstacles in the factory
  moveit_msgs::CollisionObject case1, case2, stopper1, stopper2, wall1, wall2, second_box;
  case1.header.frame_id = case2.header.frame_id = stopper1.header.frame_id =
   stopper2.header.frame_id = wall1.header.frame_id = wall2.header.frame_id = second_box.header.frame_id = "base_link";
  case1.header.stamp = case2.header.stamp = stopper1.header.stamp =
   stopper2.header.stamp = wall1.header.stamp = wall2.header.stamp = second_box.header.stamp = ros::Time::now();
  case1.id = "case1";
  case2.id = "case2";
  stopper1.id = "stopper1";
  stopper2.id = "stopper2";
  wall1.id = "wall1";
  wall2.id = "wall2";
  second_box.id = "second_box";

  double box_wall_width = 0.03;
  // A default pose
  geometry_msgs::Pose position;
  position.position.x = centerX - 0.475 * sin(phi);
  position.position.y = centerY + 0.475 * cos(phi);
  position.position.z = -0.20;
  position.orientation.w = 1.0 * cos(phi/2);
  position.orientation.x = 0.0;
  position.orientation.y = 0.0;
  position.orientation.z = 1.0 * sin(phi/2);

  // Define the box
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = box_wall_width;
  primitive.dimensions[2] = 0.55;//53

  //left
  box1.primitives.push_back(primitive);
  box1.primitive_poses.push_back(position);
  //right
  position.position.x = centerX + 0.475 * sin(phi);
  position.position.y = centerY - 0.475 * cos(phi);
  box2.primitives.push_back(primitive);
  box2.primitive_poses.push_back(position);
  //front
  position.position.x = centerX - 0.275 * cos(phi);
  position.position.y = centerY - 0.275 * sin(phi);
  primitive.dimensions[0] = box_wall_width;
  primitive.dimensions[1] = 1.00;
  //primitive.dimensions[2] = 0.25;
  box3.primitives.push_back(primitive);
  box3.primitive_poses.push_back(position);
  //back
  position.position.x = centerX + 0.275 * cos(phi);
  position.position.y = centerY + 0.275 * sin(phi);
  box4.primitives.push_back(primitive);
  box4.primitive_poses.push_back(position);

  //bottom
  position.position.x = centerX;
  position.position.y = centerY;
  position.position.z = -0.33;
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.02;
  box5.primitives.push_back(primitive);
  box5.primitive_poses.push_back(position);

  //Reset orientation
  position.orientation.w = 1.0;
  position.orientation.x = 0.0;
  position.orientation.y = 0.0;
  position.orientation.z = 0.0;

  //floor
  position.position.x = 0;
  position.position.y = 0;
  position.position.z = -0.45;
  primitive.dimensions[0] = 3;
  primitive.dimensions[1] = 3;
  primitive.dimensions[2] = 0.02;
  floor.primitives.push_back(primitive);
  floor.primitive_poses.push_back(position);

  //table
  position.position.x = 0;
  position.position.y = 0;
  position.position.z = -0.25;
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.5;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(position);

  //shelf
  position.position.x = 0;
  position.position.y = -1.04;
  position.position.z = 0.16;
  primitive.dimensions[0] = 0.65;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 1.25;
  shelf.primitives.push_back(primitive);
  shelf.primitive_poses.push_back(position);


  //line3
  position.position.x = 0;
  position.position.y = -0.910;
  position.position.z = -0.1;
  primitive.dimensions[0] = 0.65;
  primitive.dimensions[1] = 0.32;
  primitive.dimensions[2] = 0.1;
  line3.primitives.push_back(primitive);
  line3.primitive_poses.push_back(position);

  //line
  position.position.x = 0;
  position.position.y = -0.905;
  position.position.z = 0.30;
  position.orientation.w = 0.98;
  position.orientation.x = 0.199;
  position.orientation.y = 0.0;
  position.orientation.z = 0.0;
  primitive.dimensions[0] = 0.65;
  primitive.dimensions[1] = 0.36;
  primitive.dimensions[2] = 0.03;
  line.primitives.push_back(primitive);
  line.primitive_poses.push_back(position);

    //line
  position.position.x = 0;
  position.position.y = -0.905;
  position.position.z = 0.05;
  primitive.dimensions[0] = 0.65;
  primitive.dimensions[1] = 0.36;

  //originaly 0.03, changed to 0.33 to prevent robots movement between the two lines
  primitive.dimensions[2] = 0.33;
  line2.primitives.push_back(primitive);
  line2.primitive_poses.push_back(position);

  box1.operation = box1.ADD;
  box2.operation = box2.ADD;
  box3.operation = box3.ADD;
  box4.operation = box4.ADD;
  box5.operation = box5.ADD;
  floor.operation = floor.ADD;
  //table.operation =table.ADD;
  shelf.operation = shelf.ADD;
  line.operation = line.ADD;
  line2.operation = line2.ADD;
  line3.operation = line3.ADD;

  p_scene.world.collision_objects.push_back(box1);
  p_scene.world.collision_objects.push_back(box2);
  p_scene.world.collision_objects.push_back(box3);
  p_scene.world.collision_objects.push_back(box4);
  //p_scene.world.collision_objects.push_back(box5);
  p_scene.world.collision_objects.push_back(floor);
  //p_scene.world.collision_objects.push_back(table);
  p_scene.world.collision_objects.push_back(shelf);
  p_scene.world.collision_objects.push_back(line);
  p_scene.world.collision_objects.push_back(line2);
  p_scene.world.collision_objects.push_back(line3);

/////////////////////////////////////////////////////////////
  //Ojects in the factory
  position.position.x = 1;
  position.position.y = -1.5;
  position.position.z = 0.5;
  position.orientation.w = 0.1;
  position.orientation.x = 0.0;
  position.orientation.y = 0.0;
  position.orientation.z = 0.0;
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 2;
  case1.primitives.push_back(primitive);
  case1.primitive_poses.push_back(position);


  position.position.x = -0.95;
  position.position.y = -1.1;
  position.position.z = 0.5;
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 2;
  case2.primitives.push_back(primitive);
  case2.primitive_poses.push_back(position);

  position.position.x = -0.95;
  position.position.y = 0.3;
  position.position.z = 0.5;
  primitive.dimensions[0] = 0.02;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 2;
  wall1.primitives.push_back(primitive);
  wall1.primitive_poses.push_back(position);

  position.position.x = 0.0;
  position.position.y = 1.2;
  position.position.z = 0.5;
  primitive.dimensions[0] = 2.5;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 2;
  wall2.primitives.push_back(primitive);
  wall2.primitive_poses.push_back(position);

  position.position.x = -0.25;
  position.position.y = 0.8;
  position.position.z = -0.2;
  primitive.dimensions[0] = 1.4;
  primitive.dimensions[1] = 0.8;
  primitive.dimensions[2] = 0.6;
  second_box.primitives.push_back(primitive);
  second_box.primitive_poses.push_back(position);


  case1.operation = case1.ADD;
  case2.operation = case2.ADD;
  stopper1.operation = stopper1.ADD;
  stopper2.operation = stopper2.ADD;
  wall1.operation = wall1.ADD;
  wall2.operation = wall2.ADD;
  second_box.operation = second_box.ADD;

  p_scene.world.collision_objects.push_back(case1);
  p_scene.world.collision_objects.push_back(case2);
  //p_scene.world.collision_objects.push_back(stopper1);
  //p_scene.world.collision_objects.push_back(stopper2);
  p_scene.world.collision_objects.push_back(wall1);
  p_scene.world.collision_objects.push_back(wall2);
  p_scene.world.collision_objects.push_back(second_box);
/////////////////////////////////////////////////////////////


  //Add meshes
  moveit_msgs::CollisionObject stand;
  stand.id = "stand";
  shapes::Mesh* m = shapes::createMeshFromResource("package://ur_sim/mesh/visual_m.stl");
  //shapes::Mesh* m = shapes::createMeshFromResource("package://ur_sim/mesh/stul.stl", resize);

  shape_msgs::Mesh stand_mesh;
  shapes::ShapeMsg stand_mesh_msg;
  shapes::constructMsgFromShape(m,stand_mesh_msg);
  stand_mesh = boost::get<shape_msgs::Mesh>(stand_mesh_msg);
  stand.meshes.resize(1);
  stand.meshes[0] = stand_mesh;

  geometry_msgs::Pose m_pose;
  m_pose.position.x = 0.0;
  m_pose.position.y = 0.0;
  m_pose.position.z = -0.52;
  m_pose.orientation.w= 1.0;
  m_pose.orientation.x= 0.0;
  m_pose.orientation.y= 0.0;
  m_pose.orientation.z= 0.0;

  stand.mesh_poses.push_back(m_pose);
  stand.meshes.push_back(stand_mesh);
  stand.mesh_poses.push_back(stand.mesh_poses[0]);
  stand.operation = stand.ADD;

  std_msgs::ColorRGBA color;
  color.r = 0.3;
  color.g = 0.3;
  color.b = 0.8;
  color.a = 0.7;

  moveit_msgs::ObjectColor obj_col;
  obj_col.id = "box1";
  obj_col.color = color;
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "box2";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "box3";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "box4";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "box5";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "shelf";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "line";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "line2";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "line3";
  p_scene.object_colors.push_back(obj_col);
  color.a = 1;
  obj_col.color = color;
  obj_col.id = "stand";
  p_scene.object_colors.push_back(obj_col);

  //color.g = 0.5;
  color.b = 0.4;
  color.g = 0.5;
  color.a = 0.5;
  obj_col.color = color;
  obj_col.id = "floor";
  p_scene.object_colors.push_back(obj_col);

//Factory objects
  obj_col.id = "case1";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "case2";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "wall1";
  p_scene.object_colors.push_back(obj_col);
  obj_col.id = "wall2";
  p_scene.object_colors.push_back(obj_col);


  p_scene.world.collision_objects.push_back(stand);


////////////////////////////////////////////////////////


//Create the item "collar" objects, adjusting planning level in the box
//  - adds a fake bottom at the level of the item
  collar1.header.frame_id = collar2.header.frame_id = collar3.header.frame_id= collar4.header.frame_id = "base_link";
  collar1.header.stamp =  collar2.header.stamp =  collar3.header.stamp =  collar4.header.stamp = ros::Time::now();
  collar1.id = "collar1";
  collar2.id = "collar2";
  collar3.id = "collar3";
  collar4.id = "collar4";

  // Define a box to be attached
  shape_msgs::SolidPrimitive collar_primitive;
  collar_primitive.type = collar_primitive.BOX;
  collar_primitive.dimensions.resize(3);
  collar_primitive.dimensions[0] = 0.27;
  collar_primitive.dimensions[1] = 0.8;
  collar_primitive.dimensions[2] = 0.005;

  collar1.primitives.push_back(collar_primitive);
  collar3.primitives.push_back(collar_primitive);

  collar_primitive.dimensions[0] = 0.26;
  collar_primitive.dimensions[1] = 0.27;
  collar2.primitives.push_back(collar_primitive);
  collar4.primitives.push_back(collar_primitive);

  geometry_msgs::Pose collar_position;
  collar1.primitive_poses.push_back(collar_position);
  collar2.primitive_poses.push_back(collar_position);
  collar3.primitive_poses.push_back(collar_position);
  collar4.primitive_poses.push_back(collar_position);

////////////////////////////////////////////////////////


//Create the object of manipulation
//Do not push in the planning scene, yet
  item.link_name = "ee_link";
  item.object.header.frame_id = "ee_link";
  item.object.header.stamp = ros::Time::now();
  item.object.id = "item";

  // Define a box to be attached
  shape_msgs::SolidPrimitive item_primitive;
  item_primitive.type = item_primitive.CYLINDER;
  item_primitive.dimensions.resize(3);
  item_primitive.dimensions[0] = 0.01;
  item_primitive.dimensions[1] = 0.085;
  //item_primitive.dimensions[2] = 0.15;

  item.object.primitives.push_back(item_primitive);

  geometry_msgs::Pose item_position;
  item.object.primitive_poses.push_back(item_position);


//Create item for visualization
  item_visualization.link_name = "base_link";
  item_visualization.object.header.frame_id = "base_link";
  item_visualization.object.header.stamp = ros::Time::now();
  item_visualization.object.id = "item_vis";

  // Define a box to be attached
  shape_msgs::SolidPrimitive item_vis_primitive;
  item_vis_primitive.type = item_vis_primitive.CYLINDER;
  item_vis_primitive.dimensions.resize(3);
  item_vis_primitive.dimensions[0] = 0.01;
  item_vis_primitive.dimensions[1] = 0.085;
  //item_primitive.dimensions[2] = 0.15;

  item_visualization.object.primitives.push_back(item_vis_primitive);

  geometry_msgs::Pose item_vis_position;
  item_visualization.object.primitive_poses.push_back(item_vis_position);


  //Addition to manipulator model
  obs.link_name = "ee_link";
  obs.object.header.frame_id = "ee_link";
  obs.object.header.stamp = ros::Time::now();
  obs.object.id = "obs";

  // Define a box to be attached
  shape_msgs::SolidPrimitive obs_primitive;
  obs_primitive.type = obs_primitive.BOX;
  obs_primitive.dimensions.resize(3);
  obs_primitive.dimensions[0] = 0.10;
  obs_primitive.dimensions[1] = 0.03;
  obs_primitive.dimensions[2] = 0.04;

  obs.object.primitives.push_back(obs_primitive);

  geometry_msgs::Pose obs_position;
  obs_position.position.x = 0;
  obs_position.position.y = -0.03;
  obs_position.position.z = -0.28;
  obs_position.orientation.w = 1.0;
  obs_position.orientation.x = 0;
  obs_position.orientation.y = 0;
  obs_position.orientation.z = 0;

  obs.object.primitive_poses.push_back(obs_position);
  obs.object.operation = obs.object.ADD;
  p_scene.world.collision_objects.push_back(obs.object);


////////////////////////////////////////////////////////


  ROS_INFO("Publish updated planning scene");
  p_scene.is_diff = true;
  planning_scene_diff_publisher.publish(p_scene);

  ros::Duration(1).sleep();

  obs.object.operation = obs.object.REMOVE;
  p_scene.world.collision_objects.push_back(obs.object);
  group.attachObject(obs.object.id);

  ros::Duration(1).sleep();

  return 1;
}


int UR_planning::attachItem(){

  geometry_msgs::Pose item_position;
  item_position.position.x = 0;
  item_position.position.y = 0;
  item_position.position.z = 0;//0.01;
  item_position.orientation.w = 1.0;
  item_position.orientation.x = 0;
  item_position.orientation.y = 0;
  item_position.orientation.z = 0;

  item.object.primitive_poses.at(0) = item_position;
  item.object.operation = item.object.ADD;
  p_scene.world.collision_objects.push_back(item.object);

  p_scene.is_diff = true;
  planning_scene_diff_publisher.publish(p_scene);
  ros::Duration(0.2).sleep();

  item.object.operation = item.object.REMOVE;
  group.attachObject(item.object.id);

  //ros::Duration(1).sleep();
  ROS_INFO("Item added");

  return 1;
}


int UR_planning::detachItem(bool remove)
{

  group.detachObject(item.object.id);

  ROS_INFO("Item detached");

  if(remove)
  {
  item.object.operation = item.object.REMOVE;
  p_scene.world.collision_objects.push_back(item.object);
  p_scene.is_diff = true;
  planning_scene_diff_publisher.publish(p_scene);

  ros::Duration(0.2).sleep();
  ROS_INFO("Item removed");
  }
  return 1;
}

int UR_planning::item_vis(double x, double y,double z)
{

  geometry_msgs::Pose item_vis_position;
  item_vis_position.position.x = x;
  item_vis_position.position.y = y;
  item_vis_position.position.z = z;//0.01;
  item_vis_position.orientation.w = 1.0;
  item_vis_position.orientation.x = 0;
  item_vis_position.orientation.y = 0;
  item_vis_position.orientation.z = 0;

  item_visualization.object.primitive_poses.at(0) = item_vis_position;
  item_visualization.object.operation = item_visualization.object.ADD;
  p_scene.world.collision_objects.push_back(item_visualization.object);

  p_scene.is_diff = true;
  planning_scene_diff_publisher.publish(p_scene);
  ros::Duration(0.2).sleep();

  //ros::Duration(1).sleep();
  ROS_INFO("item_visualization added");

  return 1;
}



void UR_planning::set_item_level(double x, double y,double z, bool set)
{

  //hole radius + 1/2 * collar primitive
  double collar_radius = 0.13 + 0.135;

  if(set){
    ROS_INFO("Create collar at item level");
    geometry_msgs::Pose collar_position;
    collar_position.position.x = x + collar_radius;
    collar_position.position.y = y;
    collar_position.position.z = z;
    collar_position.orientation.w = 1.0;

    collar1.primitive_poses.at(0) = collar_position;

    collar_position.position.x = x - collar_radius;
    collar3.primitive_poses.at(0) = collar_position;

    collar_position.position.x = x;
    collar_position.position.y = y + collar_radius;
    collar2.primitive_poses.at(0) = collar_position;

    collar_position.position.y = y - collar_radius;
    collar4.primitive_poses.at(0) = collar_position;

    collar1.operation = collar1.ADD;
    collar2.operation = collar2.ADD;
    collar3.operation = collar3.ADD;
    collar4.operation = collar4.ADD;
    p_scene.world.collision_objects.push_back(collar1);
    p_scene.world.collision_objects.push_back(collar2);
    p_scene.world.collision_objects.push_back(collar3);
    p_scene.world.collision_objects.push_back(collar4);

    p_scene.is_diff = true;
    planning_scene_diff_publisher.publish(p_scene);
    ros::Duration(0.2).sleep();
  }
  else
  {
    ROS_INFO("Remove item collar");

    collar1.operation = collar1.REMOVE;
    collar2.operation = collar2.REMOVE;
    collar3.operation = collar3.REMOVE;
    collar4.operation = collar4.REMOVE;
    p_scene.world.collision_objects.push_back(collar1);
    p_scene.world.collision_objects.push_back(collar2);
    p_scene.world.collision_objects.push_back(collar3);
    p_scene.world.collision_objects.push_back(collar4);

    p_scene.is_diff = true;
    planning_scene_diff_publisher.publish(p_scene);
  }

}


YAML::Node trajectory_toYAML(trajectory_msgs::JointTrajectory &trajectory) {
  YAML::Node main;

  {
    YAML::Node header;
    header["seq"] = trajectory.header.seq;
    header["stamp"] = 0;
    header["frame_id"] = "/world";
    main["header"] = header;
  }

  main["joint_names"]= std::vector<std::string>{"shoulder_pan_joint",
  "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

  YAML::Node points;
  for (auto elem : trajectory.points) {
    YAML::Node elem_n;
    //auto pos = *elem.positions;
    elem_n["positions"] = elem.positions;
    elem_n["velocities"] = elem.velocities;
    elem_n["accelerations"] = elem.accelerations;
    elem_n["time_from_start"] = elem.time_from_start.toSec();
    points.push_back(elem_n);
  }
  main["points"] = points;

  return main;
}

trajectory_msgs::JointTrajectory readYAML(std::string const &data_fname){
  trajectory_msgs::JointTrajectory trajectory;
  auto input_node = YAML::LoadFile(data_fname);
  trajectory.header.frame_id = "/world";
  trajectory.joint_names = std::vector<std::string>{"shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

  auto points_node = input_node["points"];

  for (auto elem : points_node) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = elem["positions"].as<std::vector<double>>();
    point.velocities = elem["velocities"].as<std::vector<double>>();
    point.accelerations = elem["accelerations"].as<std::vector<double>>();
    ros::Duration d(elem["time_from_start"].as<double>());
    point.time_from_start = d;
    trajectory.points.push_back(point);
  }

  return trajectory;
}


//example use:
//load_trajectory("output/trajectory.yaml");
int UR_planning::load_trajectory(std::string const &data_fname){
      moveit::planning_interface::MoveGroup::Plan saved_plan;
      moveit_msgs::RobotTrajectory saved_trajectory;
      saved_trajectory.joint_trajectory = readYAML(data_fname);
      saved_plan.trajectory_ = saved_trajectory;

      ROS_INFO("Robot executing loaded plan (no obstacle avoidance)");
      group.execute(saved_plan);
}

std::vector<double> UR_planning::get_joints()
{

  std::vector<double> joints (group.getCurrentJointValues());
  ROS_INFO( "joints start : %f, %f, %f, %f, %f, %f ", joints[0], joints[1] , joints[2] , joints[3] , joints[4], joints[5] );
  return joints;
}


int UR_planning::moveTo_joints(std::vector<double> joints)
{
  LoggerPub log("PLAN_TRAJECTORY");
  bool ENABLE_EXECUTION = true;
  bool save_trajectory = false;
  std::string data_fname = "output/trajectory.yaml";

  int numOfTries = 3;
  //change by cesar sinchiguano
  float max_vel = 0.1;//1;//0.08
  float max_acc = 0.08;//0.38;//0.04
  if(force_mode_slow == 1)
  {
    max_vel = 0.02;
    max_acc = 0.02;
  }
  group.setMaxVelocityScalingFactor(max_vel);
  group.setMaxAccelerationScalingFactor(max_acc);

  //std::vector<double> joints3 (group.getCurrentJointValues());
  //ROS_INFO( "joints start : %f %f %f %f %f %f ", joints3[0], joints3[1] , joints3[2] , joints3[3] , joints3[4], joints3[5] );

  //Specify the target
  group.setJointValueTarget(joints);
  //Plan the motion and then move the group
  ROS_INFO("Plan trajectory");
  if(this->planTrajectory(numOfTries))
  {
    moveit_msgs::RobotTrajectory trajectory = this->my_plan.trajectory_;
    if(save_trajectory)
    {
      std::ofstream output_str(data_fname);
      output_str << trajectory_toYAML(trajectory.joint_trajectory);
    }

    LoggerPub log2("PLAN_TRAJECTORY_SUCCESSFUL");
    if(ENABLE_EXECUTION) group.move();
    return 1;
  } else  {
    LoggerPub log3("PLAN_TRAJECTORY_UNSUCCESSFUL");
    ROS_INFO("Planning unsuccessfull");
    return -1;
  }

}


int UR_planning::moveTo(geometry_msgs::Pose pose)
{
  LoggerPub log("PLAN_TRAJECTORY");
  bool ENABLE_EXECUTION = true;

  bool simulation = false;
  bool save_trajectory = false;
  std::string data_fname = "output/trajectory.yaml";

  int numOfTries = 3;
  float max_vel = 0.1;//0.08;//1;
  float max_acc = 0.08;//0.38;
  if(simulation){
  //change by cesar sinchiguano
  max_vel = 0.2;//0.5;
  max_acc = 0.1;//0.5;
  }
  if(force_mode_slow == 1)
  {
    max_vel = 0.02;
    max_acc = 0.02;
  }
  group.setMaxVelocityScalingFactor(max_vel);
  group.setMaxAccelerationScalingFactor(max_acc);

  //Necessary - don`t thik about it too much
  geometry_msgs::Pose pose2 = pose;
  pose2.position.z +=0.01;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pose2);
  waypoints.push_back(pose);

  //Specify the target
  group.setPoseTarget(pose);



    moveit_msgs::RobotTrajectory trajectory, traj_temp;

    //ROS_INFO("Planning");
    double fraction = -1;
    fraction = group.computeCartesianPath(waypoints, 0.05, 1000, trajectory);

    //When cartesian path fails
    if( fraction < 1 || force_mode_slow == 0) {
      ROS_INFO("fraction: =  %f Trying IKT planner (pose: x: %f, y: %f, z: %f)"
      , fraction, pose.position.x, pose.position.y, pose.position.z);

      auto state = *group.getCurrentState();
      state.update();
      if (state.setFromIK(state.getJointModelGroup("manipulator"), pose,"ee_link")) {
        ROS_INFO("IKT MOVE");
        group.setJointValueTarget(state);
        moveit::planning_interface::MoveGroup::Plan plan;
        if(group.plan(plan)){
          LoggerPub log1("PLAN_TRAJECTORY_SUCCESSFUL IKT");
          if(ENABLE_EXECUTION) group.execute(plan);
          return 1;
        }

      }

    }

    moveit::planning_interface::MoveGroup::Plan plan;
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, max_vel , max_acc);
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    //ros::Duration(1).sleep();
    plan.trajectory_ = trajectory;

    if(fraction == 1){
      //if(fraction<1) {
        ROS_INFO("Moving (fraction: =  %f) pose: x: %f, y: %f, z: %f", fraction, pose.position.x, pose.position.y, pose.position.z);
      //}

    if(save_trajectory){
      std::ofstream output_str(data_fname);
      output_str << trajectory_toYAML(trajectory.joint_trajectory);
    }
      LoggerPub log2("PLAN_TRAJECTORY_SUCCESSFUL CARTESIAN");
      if (ENABLE_EXECUTION) group.execute(plan);
      return 1;
    }
    else{

      //Do not even try standard planner in force mode
      if (force_mode_slow > 0){
        LoggerPub log3("PLAN_TRAJECTORY_UNSUCCESSFUL FORCE_MODE");
        ROS_INFO("No standard planner in force mode");
        return -2;
      }



      group.setPoseTarget(pose);
      ROS_INFO("Cannot set IKT, Trying standard planner");
      if(this->planTrajectory(numOfTries)) {
        LoggerPub log4("PLAN_TRAJECTORY_SUCCESSFUL STANDARD_PLANNER");
        if(ENABLE_EXECUTION) group.move();
        return 1;
      }
      else {
        LoggerPub log5("PLAN_TRAJECTORY_UNSUCCESSFUL");
        ROS_INFO("Planning unsuccessfull");
        return -1;
      }
    }

}



bool UR_planning::planTrajectory(int numOfTries){
  //Attempt to plan the trajectory (10 attempts)
  for(int i = 0; i<numOfTries; i++){
    if(group.plan(my_plan)){


    auto traj = my_plan.trajectory_;
    double plan_length = 0;

    //Get total length of the plan
    for(int w = 0; w < traj.joint_trajectory.points.size() -1; w++){
      plan_length += fabs(traj.joint_trajectory.points[w].positions[0] - traj.joint_trajectory.points[w + 1].positions[0]);
      plan_length += fabs(traj.joint_trajectory.points[w].positions[1] - traj.joint_trajectory.points[w + 1].positions[1]);
      plan_length += fabs(traj.joint_trajectory.points[w].positions[2] - traj.joint_trajectory.points[w + 1].positions[2]);
      plan_length += fabs(traj.joint_trajectory.points[w].positions[3] - traj.joint_trajectory.points[w + 1].positions[3]);
      plan_length += fabs(traj.joint_trajectory.points[w].positions[4] - traj.joint_trajectory.points[w + 1].positions[4]);
      plan_length += fabs(traj.joint_trajectory.points[w].positions[5] - traj.joint_trajectory.points[w + 1].positions[5]);
    }
    ROS_INFO("Attempt n.%d, PLAN LENGTH: %f", i, plan_length);

    //Do not execute plan if the plan is too long
    if(plan_length > 30) {
      ROS_INFO("Attempt n.%d, planning FAILED - plan too long", i);
      return false;
    }

      ROS_INFO("Attempt n.%d, planning SUCCESFULL. Visualizing plan", i);
      //ros::Duration(1).sleep();
      return true;
    }
    ROS_INFO("Attempt n.%d, planning FAILED - no plan found", i);
    //ros::Duration(1).sleep();
  }
  return false;
}

int UR_planning::suction(bool on_off){

  ur_msgs::SetIO srv;
  srv.request.fun = 1;
  //sets the output at pin 0 (we need 0, 1 and 2)

  //light set on 4, 5, 6

  if(on_off){
    srv.request.state = 1.0;
  }
  else {
    srv.request.state = 0.0;
  }

for(int i = 0; i < 3; i++){
  srv.request.pin = i;
 if (client.call(srv))
  {
    ROS_INFO("Output %d: set to %f", srv.request.pin, srv.request.state);

  }
  else
  {
    ROS_ERROR("Failed to call service SetIO");
  }
}

  return 1;
}

std::vector<bool> UR_planning::vacuum_status(){

  std::vector<bool> status;
  //ur_msgs::IOStates srv;
  for(int i = 0; i < 6; i++){

    status.push_back(io_states.digital_in_states.at(i).state);

  }

  return status;
}


int UR_planning::light(uint8_t intensity){

  //ROS_INFO("Light intensity %d", intensity);

  ur_msgs::SetIO srv;
  srv.request.fun = 1;

  //light set on 4, 5, 6

  srv.request.pin = 4;
  srv.request.state = (intensity & 1);
  if (client.call(srv))
  {
    //ROS_INFO("Output %d: set to %f", srv.request.pin, srv.request.state);
  }
  else
  {
    ROS_ERROR("Failed to call service SetIO");
    return -1;
  }

  srv.request.pin = 5;
  srv.request.state = (intensity & 2) >> 1;
  if (client.call(srv))
  {
    //ROS_INFO("Output %d: set to %f", srv.request.pin, srv.request.state);
  }
  else
  {
    ROS_ERROR("Failed to call service SetIO");
    return -1;
  }

  srv.request.pin = 6;
  srv.request.state = (intensity & 4) >> 2;
  if (client.call(srv))
  {
    //ROS_INFO("Output %d: set to %f", srv.request.pin, srv.request.state);
  }
  else
  {
    ROS_ERROR("Failed to call service SetIO");
    return -1;
  }

    return 1;
}
