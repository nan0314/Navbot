/// \file
/// \brief This node simulates the environment that the navbot lives in.
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "navbot_plan/navbot_plan.hpp"

static visualization_msgs::MarkerArray obstacles;
static std::vector<double> pose = {0,0,0};

void obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles = msg;
}

void pose_callback(const nav_msgs::Path msg){

    int i = msg.poses.size() - 1;
    pose = {};
    pose.push_back(msg.poses[i].pose.position.x);
    pose.push_back(msg.poses[i].pose.position.y);
    pose.push_back(msg.poses[i].pose.position.z);

}


int main(int argc, char **argv)
{
    using std::vector;
    using namespace navbot_plan;

    ros::init(argc, argv, "planner");

    ros::NodeHandle n;

    // publishers and subscribers
    ros::Subscriber obstacles_sub = n.subscribe("known_obstacles", 10, obstacle_callback);
    ros::Subscriber pose_sub = n.subscribe("navbot_path", 5, pose_callback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 5);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PointStamped>("waypoint",5);
    
    ros::Rate loop_rate(10);

    while(obstacles.markers.size() == 0){
        ros::spinOnce();
    }

    // Find initial path
    vector<vector<double>> points = theta_star({30,30,5},{-30,-30,5},obstacles,3);

    // select initial goal
    int index = 1;
    geometry_msgs::PointStamped goal_msg;
    goal_msg.header.frame_id = "world";
    goal_msg.point.x = points[index][0];
    goal_msg.point.y = points[index][1];
    goal_msg.point.z = points[index][2];

    nav_msgs::Path path = nav_path(points,"world");

    while (ros::ok())
    {

        double dist = sqrt(pow(goal_msg.point.x - pose[0],2) + pow(goal_msg.point.y - pose[1],2) + pow(goal_msg.point.z - pose[2],2));

        if (dist < 2.5 and index < points.size()-1){
            index++;
            goal_msg.point.x = points[index][0];
            goal_msg.point.y = points[index][1];
            goal_msg.point.z = points[index][2];
        }
        path_pub.publish(path);
        goal_pub.publish(goal_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}