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
#include "navbot_plan/replan.h"


#include <stack>

static visualization_msgs::MarkerArray obstacles;
static visualization_msgs::MarkerArray unknown;
static geometry_msgs::PointStamped goal_msg;
static std::vector<double> pose = {0,0,0};
static std::vector<double> goal;
static std::stack<std::vector<double>> waypoints;


bool replan_service(navbot_plan::replan::Request  &req,
         navbot_plan::replan::Response &res)
{
    using std::vector;

    std::cout << "here" << std::endl;
    int index = req.index;
    visualization_msgs::Marker obstacle = unknown.markers[index];
    visualization_msgs::MarkerArray check;
    check.markers.push_back(obstacle);
    navbot_plan::Node robot = navbot_plan::Node(pose,nullptr);
    navbot_plan::Node goal_node = navbot_plan::Node(goal,nullptr);
    vector<vector<double>> new_path;
    if (!robot.valid_successor(check,goal,1)) {
        std::cout << "yuh" << std::endl;
        new_path = navbot_plan::a_star(pose,goal,obstacles,5);        
    } else if (!goal_node.valid_successor(check,waypoints.top(),1)){
        std::cout << "yuh" << std::endl;
        new_path = navbot_plan::a_star(pose,waypoints.top(),obstacles,5);        
    } else {
        std::cout << "what?" << std::endl;
        return true;
    }

    for (int i = new_path.size()-1; i>0; i--){
            waypoints.push(new_path[i]);
        }

    // select initial goal
    
    goal = waypoints.top();
    waypoints.pop();
    goal_msg.header.frame_id = "world";
    goal_msg.point.x = goal[0];
    goal_msg.point.y = goal[1];
    goal_msg.point.z = goal[2];

    return true;
}

void obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles = msg;
}

void unknown_callback(const visualization_msgs::MarkerArray& msg)
{
    unknown = msg;
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
    using std::stack;
    using namespace navbot_plan;

    ros::init(argc, argv, "planner");

    ros::NodeHandle n;

    // publishers and subscribers
    ros::Subscriber obstacles_sub = n.subscribe("known_obstacles", 10, obstacle_callback);
    ros::Subscriber unkown_sub = n.subscribe("unknown_obstacles", 10, unknown_callback);
    ros::Subscriber pose_sub = n.subscribe("navbot_path", 5, pose_callback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 5);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PointStamped>("waypoint",5);
    ros::ServiceServer service = n.advertiseService("replan", replan_service);
    
    ros::Rate loop_rate(10);

    while(obstacles.markers.size() == 0){
        ros::spinOnce();
    }

    // Find initial path
    vector<vector<double>> points = theta_star({30,30,5},{-30,-30,5},obstacles,5);

    // Create waypoint stack
    for (int i = points.size()-1; i>0; i--){
        waypoints.push(points[i]);
    }

    // select initial goal
    goal = waypoints.top();
    waypoints.pop();
    goal_msg.header.frame_id = "world";
    goal_msg.point.x = goal[0];
    goal_msg.point.y = goal[1];
    goal_msg.point.z = goal[2];

    nav_msgs::Path path = nav_path(points,"world");

    while (ros::ok())
    {

        double dist = sqrt(pow(goal_msg.point.x - pose[0],2) + pow(goal_msg.point.y - pose[1],2) + pow(goal_msg.point.z - pose[2],2));

        if (dist < 2.5 and !waypoints.empty()){
            goal = waypoints.top();
            waypoints.pop();
            goal_msg.point.x = goal[0];
            goal_msg.point.y = goal[1];
            goal_msg.point.z = goal[2];
        }
        path_pub.publish(path);
        goal_pub.publish(goal_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}