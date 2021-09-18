/// \file
/// \brief This node handles planning and replanning of robot path. Also handles 
///        transition between path waypoints as path is executed.
///
/// PARAMETERS:
///     world_frame (string): name of the world frame
/// PUBLISHES:
///     path_pub (nav_msgs::Path): publishes original path found at initial state/time
///     replan_pub (nav_msgs::Path): publishes most recent replan path adjustment
///     goal_pub (geometry_msgs::PointStamped): publishes the location of the current waypoint goal in path
///                                             for navbot to track
/// SUBSCRIBES:
///     obstacles_sub (visualization_msgs::MarkerArray): known obstacles
///     unknown_sub (visualization_msgs::MarkerArray): unknown obstacles-- used when unknown obstacles becomes detected
///                                                    for replanning purposes
///     pose_sub (navbot_path): provides navbot pose data
/// SERVICES:
///     replan (navbot_plan::replan): triggers replanning of path in the event of newly detected obstacle
/// TRANSFORMS:
///     N/A

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "navbot_plan/navbot_plan.hpp"
#include "navbot_plan/replan.h"


#include <stack>

static int unknown_index;                               // index of newly detected obstacle
static bool obstacle_detected = false;                  // true if obstacle has been newly detected
static visualization_msgs::MarkerArray obstacles;       // known obstacles
static visualization_msgs::MarkerArray unknown;         // unkown obstacles
static std::vector<double> pose = {0,0,0};              // navbot pose


/// \brief saves index of newly detected obstacle and triggers replan
/// \param req - navbot_plan::Request data
/// \param res - navbot_plan::Response data
/// \return true
bool replan_service(navbot_plan::replan::Request  &req,
         navbot_plan::replan::Response &res)
{

    unknown_index = req.index;
    obstacle_detected = true;

    return true;
}

/// \brief saves known obstacles
/// \param msg - known obstacles
void obstacle_callback(const visualization_msgs::MarkerArray& msg)
{
    obstacles = msg;
}

/// \brief saves unknown obstacles
/// \param msg unknown obstacles
void unknown_callback(const visualization_msgs::MarkerArray& msg)
{
    unknown = msg;
}

/// \brief saves most recent navbot pose from navbot path
/// \param msg navbot path
void pose_callback(const nav_msgs::Path msg){

    int i = msg.poses.size() - 1;   // index of last navbot pose from navbot path
    pose = {};
    pose.push_back(msg.poses[i].pose.position.x);
    pose.push_back(msg.poses[i].pose.position.y);
    pose.push_back(msg.poses[i].pose.position.z);

}


int main(int argc, char **argv)
{
    using std::vector;
    using std::stack;
    using std::string;
    using namespace navbot_plan;

    // initialize node
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // publishers and subscribers
    ros::Subscriber obstacles_sub = n.subscribe("known_obstacles", 10, obstacle_callback);
    ros::Subscriber unkown_sub = n.subscribe("unknown_obstacles", 10, unknown_callback);
    ros::Subscriber pose_sub = n.subscribe("navbot_path", 5, pose_callback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 5);
    ros::Publisher replan_pub = n.advertise<nav_msgs::Path>("replan_path",5);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PointStamped>("waypoint",5);
    ros::ServiceServer service = n.advertiseService("replan", replan_service);

    // ros params
    string world_frame;
    ros::param::get("world_frame",world_frame);

    // wait for environment to be created
    while(obstacles.markers.size() == 0){
        ros::spinOnce();
    }

    // Find initial path
    vector<vector<double>> points = theta_star({30,30,5},{-30,-30,5},obstacles,8);

    // Create waypoint stack
    stack<vector<double>> waypoints;
    for (int i = points.size()-1; i>0; i--){
        waypoints.push(points[i]);
    }

    // select initial goal
    vector<double> goal = waypoints.top();
    waypoints.pop();

    geometry_msgs::PointStamped goal_msg;
    goal_msg.header.frame_id = world_frame;
    goal_msg.point.x = goal[0];
    goal_msg.point.y = goal[1];
    goal_msg.point.z = goal[2];

    nav_msgs::Path path = nav_path(points,world_frame);
    nav_msgs::Path replan_msg;
    replan_msg.header.frame_id = world_frame;

    while (ros::ok())
    {
        // Replan path if necessary
        if (obstacle_detected){
            // grab new obstacle and replan if necessary
            visualization_msgs::Marker obstacle = unknown.markers[unknown_index];
            replan_path(obstacles,obstacle,replan_msg,pose,goal,waypoints,world_frame);

            // update goal based on plan    
            goal = waypoints.top();
            waypoints.pop();
            goal_msg.header.frame_id = world_frame;
            goal_msg.point.x = goal[0];
            goal_msg.point.y = goal[1];
            goal_msg.point.z = goal[2];

            obstacle_detected = false;
        }

        // Normal procedure-- update goal if within distance and publish messages
        double dist = sqrt(pow(goal_msg.point.x - pose[0],2) + pow(goal_msg.point.y - pose[1],2) + pow(goal_msg.point.z - pose[2],2));

        if (dist < 2.5 and !waypoints.empty()){
            goal = waypoints.top();
            waypoints.pop();
            goal_msg.point.x = goal[0];
            goal_msg.point.y = goal[1];
            goal_msg.point.z = goal[2];
        }

        path_pub.publish(path);             // publish path
        goal_pub.publish(goal_msg);         // publish current waypoint goal
        replan_pub.publish(replan_msg);     // publish replan path

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}