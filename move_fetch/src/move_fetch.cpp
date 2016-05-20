/*

License

ITOMP Optimization-based Planner
Copyright © and trademark ™ 2014 University of North Carolina at Chapel Hill.
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation
for educational, research, and non-profit purposes, without fee, and without a
written agreement is hereby granted, provided that the above copyright notice,
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North
Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
without any accompanying services from the University of North Carolina at Chapel
Hill or the authors. The University of North Carolina at Chapel Hill and the
authors do not warrant that the operation of the program will be uninterrupted
or error-free. The end-user understands that the program was developed for research
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the author chpark@cs.unc.edu

*/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <move_fetch/move_fetch.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <sched.h>
#include <limits>
#include <resource_retriever/retriever.h>
//#include <pcpred/prediction/kinect_predictor.h>
#include <vector>

namespace move_fetch
{

MoveFetch::MoveFetch(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{

}

MoveFetch::~MoveFetch()
{
}

void MoveFetch::run(const std::string& group_name)
{
}

bool MoveFetch::initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states)
{
}

void MoveFetch::initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                       std::vector<robot_state::RobotState>& robot_states, int index)
{
}

bool MoveFetch::isStateSingular(robot_state::RobotState& state)
{
}

void MoveFetch::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp)
{
}

void MoveFetch::loadStaticScene()
{
}

void MoveFetch::renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state)
{
}

bool MoveFetch::isStateCollide(const robot_state::RobotState& state)
{
    return false;
}

bool MoveFetch::computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand)
{
    return true;
}

void MoveFetch::drawEndeffectorPosition(int id, const Eigen::Vector3d& position)
{
}

void MoveFetch::drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
}

void MoveFetch::drawResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
}


void MoveFetch::animateResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
}


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_fetch");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::NodeHandle node_handle("~");

    move_fetch::MoveFetch* move_fetch = new move_fetch::MoveFetch(node_handle);
    move_fetch->run("arm");
    delete move_fetch;

    return 0;
}
