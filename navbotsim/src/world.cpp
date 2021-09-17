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
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "navbotsim/obstacles.hpp"
#include "navbot_plan/replan.h"

#include <iostream>
#include <vector>

ros::Publisher known_pub;
ros::Publisher unknown_pub;
visualization_msgs::MarkerArray unknown_msg;
visualization_msgs::MarkerArray known_msg;
static ros::ServiceClient client;

void pose_callback(nav_msgs::Path msg){

    geometry_msgs::Pose pose = msg.poses[msg.poses.size()-1].pose;
    int count = 0;
    for (int i = 0; i<unknown_msg.markers.size(); i++){
        visualization_msgs::Marker obstacle = unknown_msg.markers[i];
        if (obstacle.action != 0){
            continue;
        }
        double magnitude = sqrt(pow(obstacle.scale.x,2) + pow(obstacle.scale.y,2));
        double dx = obstacle.pose.position.x - pose.position.x;
        double dy = obstacle.pose.position.y - pose.position.y;
        double dist = sqrt(pow(dx,2) + pow(dy,2));

        if (dist < 7 + magnitude){
            count++;
            visualization_msgs::Marker copy;
            copy.header = obstacle.header;
            copy.id = obstacle.id;
            copy.type = obstacle.type;
            copy.action = 0;
            copy.pose = obstacle.pose;
            copy.scale = obstacle.scale;
            copy.color.b = 1;
            copy.color.g = 1;
            copy.color.a = 0.75;
            unknown_msg.markers[i].action = 2;
            known_msg.markers.push_back(copy);


            navbot_plan::replan srv;
            srv.request.index = i;
            client.call(srv);
        }

    }

}

int main(int argc, char **argv)
{
    using std::vector;

    ros::init(argc, argv, "world");

    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("navbot_path", 5, pose_callback);
    known_pub = n.advertise<visualization_msgs::MarkerArray>("known_obstacles", 5);
    unknown_pub = n.advertise<visualization_msgs::MarkerArray>("unknown_obstacles", 5);
    client = n.serviceClient<navbot_plan::replan>("replan");

    // create obstacles   
    known_msg = obstacles::known_obstacles("world");
    unknown_msg = obstacles::unknown_obstacles("world",10);   

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        known_pub.publish(known_msg);
        unknown_pub.publish(unknown_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}