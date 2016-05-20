/*
 * move_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_FETCH_H
#define MOVE_FETCH_H

#include <moveit_msgs/RobotTrajectory.h>

namespace move_fetch
{

class MoveFetch
{
public:
        MoveFetch(const ros::NodeHandle& node_handle);
        ~MoveFetch();

        void run(const std::string& group_name);

protected:
	void loadStaticScene();
        void initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                 std::vector<robot_state::RobotState>& robot_states, int index = 0);
        bool initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states);
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

        void plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp);
        bool computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand = false);

        void renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state);
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

	std::string group_name_;

        robot_state::RobotStatePtr last_goal_state_;

        Eigen::Affine3d mat_rivet_magazine_;
};

}

#endif /* MOVE_FETCH_H */
