#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/tf.h>

#include <vector>

#include "planning_exec_interface/MoveHome.h"
#include "planning_exec_interface/MoveToCartRPY.h"
#include "planning_exec_interface/MoveToJoint.h"
#include "planning_exec_interface/ReplyInt.h"

class PlanningExecCbContainer {
public:
  std::string name_group_ = "manipulator";
  bool success_plan_ = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  int num_dof_ = 0;
  std::string name_tool_frame_ = "link_t";

  PlanningExecCbContainer(std::string name_group, int num_dof,
                          std::string name_tool_frame) {
    name_group_ = name_group;
    num_dof_ = num_dof;
    name_tool_frame_ = name_tool_frame;
  }

  bool cbPlanToCartRPY(planning_exec_interface::MoveToCartRPY::Request &req,
                       planning_exec_interface::MoveToCartRPY::Response &res) {

    tf::Quaternion tf_q;
    // NOTE The below assignment for RPY is strange:
    //      yaw -> Y, pitch -> X, roll -> Z
    tf_q.setEuler(req.pose_rpy[4], req.pose_rpy[3], req.pose_rpy[5]);
    geometry_msgs::Pose pose;
    pose.position.x = req.pose_rpy[0];
    pose.position.y = req.pose_rpy[1];
    pose.position.z = req.pose_rpy[2];
    pose.orientation.x = tf_q.getX();
    pose.orientation.y = tf_q.getY();
    pose.orientation.z = tf_q.getZ();
    pose.orientation.w = tf_q.getW();

    moveit::planning_interface::MoveGroupInterface move_group(name_group_);
    ROS_DEBUG_STREAM("req.name_frame_ref: " << req.name_frame_ref);
    move_group.setPoseReferenceFrame(req.name_frame_ref);
    move_group.setPoseTarget(pose, name_tool_frame_);
    move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
    move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);

    success_plan_ = (move_group.plan(plan_) ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_plan_) {
      res.reply = 0;
    } else {
      res.reply = -1;
    }
    return success_plan_;
  }

  bool cbPlanToJoint(planning_exec_interface::MoveToJoint::Request &req,
                     planning_exec_interface::MoveToJoint::Response &res) {

    // FIXME Velocity and acceleration limit does not seem to be working
    //       (changing it does not seem to have any effect on the robot's speed)
    moveit::planning_interface::MoveGroupInterface move_group(name_group_);
    move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
    move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
    ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
    ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

    ROS_DEBUG_STREAM("name_group_: " << name_group_);

    std::vector<double> joint_angles;
    if (num_dof_ != 0) {
      joint_angles.clear();

      for (int i = 0; i < num_dof_; i++) {
        joint_angles.push_back(req.joint_points.at(i));
        ROS_DEBUG_STREAM("req.joint_points: " << req.joint_points.at(i));
      }
    } else {
      ROS_ERROR_STREAM("Please supply the amount of DOF for planning group \""
                       << name_group_ << "\".");
      ros::shutdown();
    }

    move_group.setJointValueTarget(joint_angles);

    success_plan_ = (move_group.plan(plan_) ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_plan_) {
      res.reply = 0;
      return true;
    } else {
      res.reply = -1;
      return false;
    }
  }

  bool cbExecutePlan(planning_exec_interface::ReplyInt::Request &req,
                     planning_exec_interface::ReplyInt::Response &res) {

    moveit::planning_interface::MoveGroupInterface move_group(name_group_);
    bool success_exec = false;
    ROS_DEBUG("Planned successfully: %s", (success_plan_) ? "true" : "false");
    if (success_plan_) {
      success_exec = (move_group.execute(plan_) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    ROS_DEBUG("Executed successfully: %s", (success_exec) ? "true" : "false");

    if (success_exec) {
      res.reply = 0;
      return true;
    } else {
      res.reply = -1;
      return false;
    }
  }

  bool cbMoveToJoint(planning_exec_interface::MoveToJoint::Request &req,
                     planning_exec_interface::MoveToJoint::Response &res) {

    // FIXME Velocity and acceleration limit does not seem to be working
    //       (changing it does not seem to have any effect on the robot's speed)
    moveit::planning_interface::MoveGroupInterface move_group(name_group_);
    move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
    move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
    ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
    ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

    ROS_DEBUG_STREAM("name_group_: " << name_group_);

    std::vector<double> joint_angles;
    if (num_dof_ != 0) {
      joint_angles.clear();

      for (int i = 0; i < num_dof_; i++) {
        joint_angles.push_back(req.joint_points.at(i));
        ROS_DEBUG_STREAM("req.joint_points: " << req.joint_points.at(i));
      }
    } else {
      ROS_ERROR_STREAM("Please supply the amount of DOF for planning group \""
                       << name_group_ << "\".");
      ros::shutdown();
    }

    move_group.setJointValueTarget(joint_angles);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success_plan = (move_group.plan(plan) ==
                         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    bool success_exec = false;
    if (success_plan) {
      success_exec = (move_group.execute(plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    if (success_exec) {
      res.reply = 0;
      return true;
    } else {
      res.reply = -1;
      return false;
    }
  }

  bool cbMoveHome(planning_exec_interface::MoveHome::Request &req,
                  planning_exec_interface::MoveHome::Response &res) {

    // FIXME Velocity and acceleration limit does not seem to be working
    //       (changing it does not seem to have any effect on the robot's speed)
    moveit::planning_interface::MoveGroupInterface move_group(name_group_);
    move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
    move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
    ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
    ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

    std::vector<double> joint_angles;
    if (num_dof_ != 0) {
      joint_angles.clear();
      joint_angles.resize(num_dof_, 0.0);
    } else {
      ROS_ERROR_STREAM("Please supply the amount of DOF for planning group \""
                       << name_group_ << "\".");
      ros::shutdown();
    }

    move_group.setJointValueTarget(joint_angles);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success_plan = (move_group.plan(plan) ==
                         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    bool success_exec = false;
    if (success_plan) {
      success_exec = (move_group.execute(plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    if (success_exec) {
      res.reply = 0;
      return true;
    } else {
      res.reply = -1;
      return false;
    }
  }
};

int main(int argc, char **argv) {

  //// Initializations
  ros::init(argc, argv, "planning_execution");

  //// Setup node handles for
  //     - move_home and move_to service callbacks
  ros::NodeHandle nh_move;

  //// Setup custom callback queues for
  //     - move_home and move_to service callbacks
  ros::CallbackQueue queue_move;
  nh_move.setCallbackQueue(&queue_move);

  //// Start an AsyncSpinner with two threads for
  //     - calls to service 'move_home' and
  //     - trajectory planning and execution (moveit_group needs its own async
  //       spinner)
  ros::AsyncSpinner spin_move(1, &queue_move);
  spin_move.start();
  ros::AsyncSpinner spin_plan(1);
  spin_plan.start();

  //// Get parameters from server
  std::vector<std::string> names_group;
  // TODO Probe for zero entries in names_group
  nh_move.getParam("/planning_execution/names_group", names_group);
  int num_groups = names_group.size();
  std::vector<std::string> names_tool_frame;
  nh_move.getParam("/planning_execution/names_tool_frame", names_tool_frame);
  std::vector<int> num_dof_groups;
  // ROS_DEBUG_STREAM("names_group[0]: " << names_group[0]);
  ROS_DEBUG_STREAM("num_groups: " << num_groups);
  nh_move.getParam("/planning_execution/num_dof_groups", num_dof_groups);

  //// Advertise services
  //     - move_home
  //     - plan_to_rpy
  //     - plan_to_quat
  //     - execute_plan
  //     - move_to_rpy
  //     - move_to_quat
  std::vector<PlanningExecCbContainer> planning_exec_cb_containers;
  std::vector<ros::ServiceServer> service_servers;

  for (int i = 0; i < num_groups; i++) {
    PlanningExecCbContainer pe_cb_container(names_group[i], num_dof_groups[i],
                                            names_tool_frame[i]);
    planning_exec_cb_containers.push_back(pe_cb_container);
  }

  /// move_home_* services
  for (int i = 0; i < num_groups; i++) {

    std::ostringstream oss;
    oss << "/planning_execution/move_home_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv = nh_move.advertiseService(
        name_service, &PlanningExecCbContainer::cbMoveHome,
        &planning_exec_cb_containers[i]);

    service_servers.push_back(srv);
  }

  /// move_to_joint_* services
  for (int i = 0; i < num_groups; i++) {

    std::ostringstream oss;
    oss << "/planning_execution/move_to_joint_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv = nh_move.advertiseService(
        name_service, &PlanningExecCbContainer::cbMoveToJoint,
        &planning_exec_cb_containers[i]);

    service_servers.push_back(srv);
  }

  /// plan_to_cart_rpy_* service
  for (int i = 0; i < num_groups; i++) {

    std::ostringstream oss;
    oss << "/planning_execution/plan_to_cart_rpy_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv = nh_move.advertiseService(
        name_service, &PlanningExecCbContainer::cbPlanToCartRPY,
        &planning_exec_cb_containers[i]);

    service_servers.push_back(srv);
  }

  /// plan_to_joint_* service
  for (int i = 0; i < num_groups; i++) {

    std::ostringstream oss;
    oss << "/planning_execution/plan_to_joint_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv = nh_move.advertiseService(
        name_service, &PlanningExecCbContainer::cbPlanToJoint,
        &planning_exec_cb_containers[i]);

    service_servers.push_back(srv);
  }

  /// execute_plan_* service
  for (int i = 0; i < num_groups; i++) {

    std::ostringstream oss;
    oss << "/planning_execution/execute_plan_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv = nh_move.advertiseService(
        name_service, &PlanningExecCbContainer::cbExecutePlan,
        &planning_exec_cb_containers[i]);

    service_servers.push_back(srv);
  }

  while (ros::ok()) {

    ros::Duration(0.1).sleep();
  }
}
