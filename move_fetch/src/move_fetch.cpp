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

#include <move_fetch/move_fetch.h>

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
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

static const int PLANNER_INDEX = -1;

MoveFetch::MoveFetch(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{
}

MoveFetch::~MoveFetch()
{
}

void MoveFetch::run(const std::string& group_name)
{
    // scene initialization
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }
    
    loadStaticScene();
    loadActions();

    // planner initialization
    group_name_ = group_name;
    
    // load ITOMP plugin
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    std::string planner_plugin_name;
    if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try
    {
        cpu_set_t mask;
        if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_getaffinity failed");
        itomp_planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_setaffinity failed");

        if (!itomp_planner_instance_->initialize(robot_model_, node_handle_.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << itomp_planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }
    
    // publisher initialization
    display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    start_state_display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state", 1, true);
    goal_state_display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1, true);
    ros::Duration sleep_time(0.5);
    sleep_time.sleep();
    
    // planning
    while (ros::ok())
    {
        // TODO: recieve planning request
        
        // task initialization
        std::vector<Eigen::Affine3d> end_effector_poses;
        std::vector<robot_state::RobotState> robot_states;
        if (initTask(end_effector_poses, robot_states) == false)
            continue;
        
        // plan subtasks
        moveit_msgs::DisplayTrajectory display_trajectory;
        for (int index = 0; index < end_effector_poses.size() - 1; ++index)
        {
            moveit_msgs::MotionPlanResponse response;
            planning_interface::MotionPlanRequest req;
            planning_interface::MotionPlanResponse res;

            // set start / goal states and fill planning request
            initStartGoalStates(req, end_effector_poses, robot_states, index);

            // trajectory optimization using ITOMP
            plan(req, res);
            res.getMessage(response);

            // store last goal state for next subtask
            last_goal_state_.reset(new robot_state::RobotState(res.trajectory_->getLastWayPoint()));

            // display trajectories
            if (index == 0)
                display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory);
            display_publisher_.publish(display_trajectory);
        }
        
        // TODO: (plan once in current code)
        break;
    }
    
    // clean up
    itomp_planner_instance_.reset();
    planning_scene_.reset();
    robot_model_.reset();

    ROS_INFO("Done");
}

bool MoveFetch::initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states)
{
    end_effector_poses.resize(0);

    robot_state::RobotState& start_state = planning_scene_->getCurrentStateNonConst();
    if (last_goal_state_)
    {
        start_state = *last_goal_state_;
    }
    
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d(0, 1, 0));
    const Eigen::Vector3d z_offset(0., 0., 0.15);
    
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    start_pose.translate(action_source_);
    start_pose.translate(z_offset);
    start_pose.rotate(rotation);
    
    // set tasks
    const int action = 0; // TODO: acquire action label
    for (int i = 0; i < 1; ++i)
    {
        end_effector_poses.push_back(start_pose);

        const int target_index = action;
        Eigen::Affine3d goal_pose = Eigen::Affine3d::Identity();
        goal_pose.translate(action_targets_[target_index]);
        goal_pose.translate(z_offset);
        goal_pose.rotate(rotation);
        end_effector_poses.push_back(goal_pose);
    }

    end_effector_poses.push_back(start_pose);

    // IK for finding robot states
    robot_states.resize(end_effector_poses.size(), start_state);

    for (int i = 0; i < robot_states.size(); ++i)
    {
        robot_states[i].update();
        if (computeIKState(robot_states[i], end_effector_poses[i]) == false)
        {
            ROS_INFO("IK Fail");
            return false;
        }
    }

    ROS_INFO("IK Success");

    return true;
}

void MoveFetch::initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                    const std::vector<robot_state::RobotState>& robot_states, int index)
{
    const std::string end_effector_name = "wrist_roll_link";

    drawPath(index, end_effector_poses[index].translation(), end_effector_poses[index + 1].translation());
    ros::WallDuration sleep_t(0.001);
    sleep_t.sleep();

    const robot_state::RobotState& start_state = last_goal_state_ ? *last_goal_state_ : robot_states[index];
    const robot_state::RobotState& goal_state = robot_states[index + 1];
    renderStartGoalStates(start_state, goal_state);

    // set start state
    // Copy from start_state to req.start_state
    unsigned int num_joints = start_state.getVariableCount();
    req.start_state.joint_state.name = start_state.getVariableNames();
    req.start_state.joint_state.position.resize(num_joints);
    req.start_state.joint_state.velocity.resize(num_joints);
    req.start_state.joint_state.effort.resize(num_joints);
    memcpy(&req.start_state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_joints);
    if (start_state.hasVelocities())
        memcpy(&req.start_state.joint_state.velocity[0], start_state.getVariableVelocities(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.velocity[0], 0, sizeof(double) * num_joints);
    if (start_state.hasAccelerations())
        memcpy(&req.start_state.joint_state.effort[0], start_state.getVariableAccelerations(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.effort[0], 0, sizeof(double) * num_joints);

    // set goal state
    req.goal_constraints.clear();

    const Eigen::Affine3d& transform = end_effector_poses[index + 1];
    Eigen::Vector3d trans = transform.translation();
    Eigen::Quaterniond rot = Eigen::Quaterniond(transform.linear());

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = robot_model_->getModelFrame();
    goal_pose.pose.position.x = trans(0);
    goal_pose.pose.position.y = trans(1);
    goal_pose.pose.position.z = trans(2);
    goal_pose.pose.orientation.x = rot.x();
    goal_pose.pose.orientation.y = rot.y();
    goal_pose.pose.orientation.z = rot.z();
    goal_pose.pose.orientation.w = rot.w();
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    /*// print joint values
    std::stringstream ss;
    ss << "Start state : ";
    ss.precision(std::numeric_limits<double>::digits10);
    for (int i = 0; i < start_state.getVariableCount(); ++i)
        ss << std::fixed << start_state.getVariablePositions()[i] << " ";
    ROS_INFO(ss.str().c_str());
    */
}

bool MoveFetch::isStateSingular(robot_state::RobotState& state)
{
    // check singularity
    Eigen::MatrixXd jacobianFull = (state.getJacobian(planning_scene_->getRobotModel()->getJointModelGroup(group_name_)));
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
    int rows = svd.singularValues().rows();
    double min_value = svd.singularValues()(rows - 1);

    const double threshold = 1e-3;
    if (min_value < threshold)
        return true;
    else
        return false;
}

void MoveFetch::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
    req.group_name = group_name_;
    req.allowed_planning_time = 300.0;
    req.num_planning_attempts = 1;

    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.25;
    req.workspace_parameters.min_corner.z = -0.25;
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 0.75;
    req.workspace_parameters.max_corner.z = 1.75;

    ROS_INFO("Available planners :");
    std::vector<std::string> algorithms;
    itomp_planner_instance_->getPlanningAlgorithms(algorithms);
    for (unsigned int i = 0; i < algorithms.size(); ++i)
    {
        if (algorithms[i].find(group_name_) != std::string::npos)
            ROS_INFO("%d : %s", i, algorithms[i].c_str());
    }

    if (PLANNER_INDEX != -1)
        req.planner_id = algorithms[PLANNER_INDEX];

    req.planner_id = "ITOMP_replanning";

    planning_interface::PlanningContextPtr context =
        itomp_planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return;
    }
}

void MoveFetch::loadStaticScene()
{
    moveit_msgs::PlanningScene planning_scene_msg;
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle_.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    if (!environment_file.empty())
    {
        double scale;
        node_handle_.param("/itomp_planner/environment_model_scale", scale, 1.0);
        environment_position.resize(3, 0);
        if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
        {
            XmlRpc::XmlRpcValue segment;
            node_handle_.getParam("/itomp_planner/environment_model_position", segment);
            if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                int size = segment.size();
                for (int i = 0; i < size; ++i)
                {
                    double value = segment[i];
                    environment_position[i] = value;
                }
            }
        }

        // Collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = robot_model_->getModelFrame();
        collision_object.id = "environment";
        geometry_msgs::Pose pose;
        pose.position.x = environment_position[0];
        pose.position.y = environment_position[1];
        pose.position.z = environment_position[2];
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file, Eigen::Vector3d(scale, scale, scale));
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        collision_object.operation = collision_object.ADD;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;
        planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
    }

    planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void MoveFetch::loadActions()
{
    std::vector<double> actions;
    node_handle_.getParam("/itomp_planner/action_positions", actions);
    for (int i=0; i<actions.size() / 3 && i < 4; i++)
        action_targets_.push_back( Eigen::Vector3d( actions[3*i+0], actions[3*i+1], actions[3*i+2] ) );

    node_handle_.getParam("/itomp_planner/source_position", actions);
    action_source_ = Eigen::Vector3d( actions[0], actions[1], actions[2] );
}

void MoveFetch::renderStartGoalStates(const robot_state::RobotState& start_state, const robot_state::RobotState& goal_state)
{
    // display start / goal states
    int num_variables = start_state.getVariableNames().size();
    moveit_msgs::DisplayRobotState disp_start_state;
    disp_start_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_start_state.state.joint_state.name = start_state.getVariableNames();
    disp_start_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_start_state.state.joint_state.position[0],
           start_state.getVariablePositions(), sizeof(double) * num_variables);
    disp_start_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names = robot_model_->getLinkModelNames();
    for (unsigned int i = 0; i < link_model_names.size(); ++i)
    {
        std_msgs::ColorRGBA color;

        color.a = 0.5;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.5;

        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_start_state.highlight_links.push_back(obj_color);
    }
    start_state_display_publisher_.publish(disp_start_state);
    
    moveit_msgs::DisplayRobotState disp_goal_state;
    disp_goal_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
    disp_goal_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_goal_state.state.joint_state.position[0], goal_state.getVariablePositions(), sizeof(double) * num_variables);
    disp_goal_state.highlight_links.clear();
    for (int i = 0; i < link_model_names.size(); ++i)
    {
        std_msgs::ColorRGBA color;
        color.a = 0.5;
        color.r = 0.0;
        color.g = 0.5;
        color.b = 1.0;
        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_goal_state.highlight_links.push_back(obj_color);
    }
    goal_state_display_publisher_.publish(disp_goal_state);
}

bool MoveFetch::isStateCollide(const robot_state::RobotState& state)
{
    return false;
}

bool MoveFetch::computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand)
{
    // compute waypoint ik solutions

    const robot_state::JointModelGroup* joint_model_group = ik_state.getJointModelGroup(group_name_);

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    bool found_ik = false;

    robot_state::RobotState org_start(ik_state);
    int i = 0;

    if (rand)
        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, log(-3) / log(10));

    while (true)
    {
        found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1, moveit::core::GroupStateValidityCallbackFn(), options);
        ik_state.update();

        found_ik &= !isStateCollide(ik_state);

        if (found_ik && isStateSingular(ik_state))
            found_ik = false;

        if (found_ik)
            break;

        ++i;

        double dist = log(-3 + 0.001 * i) / log(10);

        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, dist);

        break;
    }

    if (found_ik)
    {
        //ROS_INFO("IK solution found after %d trials", i + 1);
    }
    else
    {
        ROS_INFO("Could not find IK solution");
    }
    return found_ik;
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
    // output is not buffered
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    ros::init(argc, argv, "move_itomp");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::NodeHandle node_handle("~");

    move_fetch::MoveFetch* move_fetch = new move_fetch::MoveFetch(node_handle);
    move_fetch->run("arm");
    delete move_fetch;

    return 0;
}
