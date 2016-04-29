// From Moveit! tutorial: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// mesh shapes
#include <geometric_shapes/shape_operations.h>

// conversion from rpy to quaternion
#include <tf/transform_datatypes.h>


int main(int argc, char **argv)
{
    ros::init (argc, argv, "moveit_object_publisher");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Moveit object publisher");
    fflush(stdout);

    ros::NodeHandle node_handle("~");
    ros::Duration sleep_time(5.0);
    sleep_time.sleep();


// BEGIN_TUTORIAL
//
// ROS API
// ^^^^^^^
// The ROS API to the planning scene publisher is through a topic interface
// using "diffs". A planning scene diff is the difference between the current
// planning scene (maintained by the move_group node) and the new planning
// scene desired by the user.

// Advertise the required topic
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Note that this topic may need to be remapped in the launch file
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (ros::ok() && planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    ROS_INFO("Planning scene publisher ready");
    fflush(stdout);

    // TODO: get object information through rosparam
    moveit_msgs::CollisionObject object;
    if (node_handle.hasParam("objects"))
    {
        XmlRpc::XmlRpcValue segment;

        ROS_INFO("Adding objects");
        fflush(stdout);

        node_handle.getParam("objects", segment);

        if (segment.getType() == XmlRpc::XmlRpcValue::TypeStruct && segment.size() > 0)
        {
            for (XmlRpc::XmlRpcValue::iterator it = segment.begin(); it != segment.end(); it++)
            {
                // object name is not used
                std::string object_name = it->first;

                ROS_INFO("Adding %s", object_name.c_str());
                fflush(stdout);

                std::string filename;
                std::vector<double> pose_param;

                if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                    filename = static_cast<std::string>(it->second);
                    pose_param.resize(6, 0.);
                }
                else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    filename = static_cast<std::string>(it->second[0]);

                    int size = it->second.size();
                    for (int i = 1; i < size; ++i)
                        pose_param.push_back( static_cast<double>(it->second[i]) );

                    if (pose_param.size() != 6)
                    {
                        ROS_ERROR("object %s pose is not defined properly. (x y z r p y) is required.");
                        fflush(stderr);
                        return 1;
                    }
                }

                tf::Quaternion q = tf::createQuaternionFromRPY(pose_param[3], pose_param[4], pose_param[5]);

                geometry_msgs::Pose pose;
                pose.position.x = pose_param[0];
                pose.position.y = pose_param[1];
                pose.position.z = pose_param[2];
                pose.orientation.w = q.w();
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();

                shapes::Mesh* mesh = shapes::createMeshFromResource(filename);
                shape_msgs::Mesh msg_mesh;
                shapes::ShapeMsg mesh_msg;
                shapes::constructMsgFromShape(mesh, mesh_msg);
                msg_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

                object.meshes.push_back(msg_mesh);
                object.mesh_poses.push_back(pose);
                object.operation = moveit_msgs::CollisionObject::ADD;

                delete mesh;
            }
        }
    }


// Add an object into the environment
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Add the object into the environment by adding it to
// the set of collision objects in the "world" part of the
// planning scene. Note that we are using only the "object"
// field of the attached_object message here.
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    ROS_INFO("Moveit object published");
    sleep_time.sleep();

    ros::shutdown();
    return 0;
}
