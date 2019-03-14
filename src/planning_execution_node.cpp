#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <vector>

#include "planning_exec_interface/MoveHome.h"

class PlanningExecutionGroup {
public:
  std::string planning_group_ = "manipulator";
  bool success_plan_ = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  int num_dof_ = 0;


  PlanningExecutionGroup(std::string planning_group, int num_dof){
    planning_group_ = planning_group;
    num_dof_ = num_dof;
  }


  bool cbMoveHome(planning_exec_interface::MoveHome::Request &req,
                  planning_exec_interface::MoveHome::Response &res) {

    // FIXME Velocity and acceleration limit does not seem to be working
    //       (changing it does not seem to have any effect on the robot's speed)
    moveit::planning_interface::MoveGroupInterface move_group(planning_group_);
    move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
    move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
    ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
    ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

    std::vector<double> joint_angles;
    if(num_dof_ != 0){
      joint_angles.clear();
      joint_angles.resize(num_dof_, 0.0);
    }
    else {
      ROS_ERROR_STREAM("Please supply the amount of DOF for planning group \""
                        << planning_group_ << "\".");
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


int main(int argc, char **argv){


  // --- Initializations
  ros::init(argc, argv, "planning_execution");


  // --- Setup node handles for
  //     - move_home and move_to service callbacks
  ros::NodeHandle nh_move;


  // --- Setup custom callback queues for
  //     - move_home and move_to service callbacks
  ros::CallbackQueue queue_move;
  nh_move.setCallbackQueue(&queue_move);


  // --- Start an AsyncSpinner with two threads for
  //     - calls to service 'move_home' and
  //     - trajectory planning and execution (moveit_group needs its own async
  //       spinner)
  ros::AsyncSpinner spin_move(1, &queue_move);
  spin_move.start();
  ros::AsyncSpinner spin_plan(1);
  spin_plan.start();


  // --- Get parameters from server
  std::vector<std::string> names_group;
  // TODO Probe for zero entries in names_group
  nh_move.getParam("/planning_execution/names_group", names_group);
  int num_groups = names_group.size();
  std::vector<int> num_dof_groups;
  //ROS_DEBUG_STREAM("names_group[0]: " << names_group[0]);
  ROS_DEBUG_STREAM("num_groups: " << num_groups);
  nh_move.getParam("/planning_execution/num_dof_groups", num_dof_groups);

  // --- Advertise services
  //     - move_home
  //     - plan_to_rpy
  //     - plan_to_quat
  //     - execute_plan
  //     - move_to_rpy
  //     - move_to_quat
  std::vector<PlanningExecutionGroup> planning_exec_groups;
  std::vector<ros::ServiceServer> service_servers;

  for(int i=0; i<num_groups; i++){

    PlanningExecutionGroup pe_group(names_group[i], num_dof_groups[i]);
    planning_exec_groups.push_back(pe_group);
    std::ostringstream oss;
    oss << "/planning_execution/move_home_" << names_group[i];
    std::string name_service = oss.str();
    ROS_DEBUG_STREAM("name_service: " << name_service);
    ros::ServiceServer srv =
        nh_move.advertiseService(name_service,
                                 &PlanningExecutionGroup::cbMoveHome,
                                 &pe_group);
    service_servers.push_back(srv);
  }

  while (ros::ok()) {

    ros::Duration(0.1).sleep();

  }


}
