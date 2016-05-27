/*
 * move_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_FETCH_H
#define MOVE_FETCH_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <Eigen/Dense>

namespace move_fetch
{

/*
 * Gripper closing using actionlib
 * rostopic pub -1 /gripper_controller/gripper_action/goal control_msgs/GripperCommandActionGoal '{header:{}, goal_id:{}, goal:{command : {position : 0.01, max_effort : 1000}}}' // 1cm or 2cm between fingers?
 *   topic: /gripper_controller/gripper_action/goal (Fetch robot manual)
 *   type:  control_msgs/GripperCommandActionGoal   (Fetch robot manual)
 *   goal.command.position = 0.01
 *   goal.command.max_effort = 1000
 * 
 * Gripper opening using actionlib
 * rostopic pub -1 /gripper_controller/gripper_action/goal control_msgs/GripperCommandActionGoal '{header:{}, goal_id:{}, goal:{command : {position : 0.10, max_effort : 1000}}}'
 */

class MoveFetch
{
public:
    MoveFetch(const ros::NodeHandle& node_handle);
    ~MoveFetch();

    void closeGripper();
    void openGripper();
    
    void run(const std::string& group_name);

protected:
	void loadStaticScene();
    void loadActions();
    void initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                             const std::vector<robot_state::RobotState>& robot_states, int index = 0);
    bool initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states, int action);
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

    void plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res);
    bool computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand = false);

    void renderStartGoalStates(const robot_state::RobotState& start_state, const robot_state::RobotState& goal_state);
    void drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to);
	void drawEndeffectorPosition(int id, const Eigen::Vector3d& position);
    void drawResults(moveit_msgs::DisplayTrajectory& display_trajectory);
    void animateResults(moveit_msgs::DisplayTrajectory& display_trajectory);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;
    ros::Publisher start_state_display_publisher_;
    ros::Publisher goal_state_display_publisher_;
    
    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

	std::string group_name_;

    robot_state::RobotStatePtr last_goal_state_;

    // hard-coded source/target positions of motion planning
    std::vector<Eigen::Vector3d> action_targets_;
    Eigen::Vector3d action_source_;

};

}

#endif /* MOVE_FETCH_H */
