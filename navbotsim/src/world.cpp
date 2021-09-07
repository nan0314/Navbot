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
#include "std_msgs/String.h"
#include "navbotsim/obstacles.hpp"

#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    using std::vector;

    ros::init(argc, argv, "world");

    ros::NodeHandle n;

    ros::Publisher known_pub = n.advertise<visualization_msgs::MarkerArray>("known_obstacles", 5);
    ros::Publisher unknown_pub = n.advertise<visualization_msgs::MarkerArray>("unknown_obstacles", 5);

    // create obstacles   
    visualization_msgs::MarkerArray known_msg = obstacles::known_obstacles("world");
    visualization_msgs::MarkerArray unknown_msg = obstacles::unknown_obstacles("world",10);

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