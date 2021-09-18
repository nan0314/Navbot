/// \file
/// \brief This node simulates the environment that the navbot lives in.
///
/// PARAMETERS:
///     enable_replan (bool): true if unknown obtacles and replanning is desired
///     world_frame (string): name of world frame
/// PUBLISHES:
///     known_obstacles (visualization_msgs::MarkerArray): known obstacles in environment
///     unknown_obstacles (visualization_msgs::MarkerArray): unknown obstacles in environment
/// SUBSCRIBES:
///     navbot_path (nav_msgs::Path): path of navbot poses
/// SERVICES:
///     N/A
/// TRANSFORMS:
///     N/A

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "navbotsim/obstacles.hpp"
#include "navbot_plan/replan.h"

#include <iostream>
#include <vector>

static geometry_msgs::Pose pose;
static bool pose_recieved = false;

void pose_callback(nav_msgs::Path msg){

    pose = msg.poses[msg.poses.size()-1].pose;
    pose_recieved = true;

}

int main(int argc, char **argv)
{
    using std::string;
    using std::vector;
    using namespace obstacles;

    // initialize node
    ros::init(argc, argv, "world");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // publishers and subscribers
    ros::Subscriber pose_sub = n.subscribe("navbot_path", 5, pose_callback);
    ros::Publisher known_pub = n.advertise<visualization_msgs::MarkerArray>("known_obstacles", 5);
    ros::Publisher unknown_pub = n.advertise<visualization_msgs::MarkerArray>("unknown_obstacles", 5);
    ros::ServiceClient client = n.serviceClient<navbot_plan::replan>("replan");


    // get ros params
    bool enable_replan;
    string world_frame;
    
    ros::param::get("enable_replan", enable_replan);
    ros::param::get("world_frame",world_frame);

    // create obstacles   
    visualization_msgs::MarkerArray known_msg = known_obstacles(world_frame);
    visualization_msgs::MarkerArray unknown_msg;
    if (enable_replan){
        unknown_msg = unknown_obstacles(world_frame,10); 
    }

    while (ros::ok())
    {

        // Check for new obstacles when navbot pose is recieved;
        if (pose_recieved and enable_replan){

            int obstacle_index = check_obstacles(known_msg,unknown_msg,pose);
            if (obstacle_index >= 0){
                known_pub.publish(known_msg);
                unknown_pub.publish(unknown_msg);
                navbot_plan::replan srv;
                srv.request.index = obstacle_index;
                client.call(srv);
            }

            pose_recieved = false;
        }

        known_pub.publish(known_msg);
        unknown_pub.publish(unknown_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}