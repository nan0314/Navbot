/// \file
/// \brief Library for setting up navbot environment obstacles.

#ifndef OBSTACLES_INCLUDE_GUARD_HPP
#define OBSTACLES_INCLUDE_GUARD_HPP

#include <geometry_msgs/Quaternion.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <string>
#include <cstdlib> 
#include <ctime>



namespace obstacles{
    
    using std::string;

    /// \brief creates a rectangular prism obstacle marker
    ///        
    /// \param x - x location of obstacle
    /// \param y - y location of obstacle
    /// \param z - z location of obstacle
    /// \param w - width of obstacle
    /// \param l - length of obstacle
    /// \param h - height of obstacle
    /// \param known - true if obstacle is known
    /// \param frame - frame where obstacle is located
    /// \param id - obstacle id
    /// \return obstacle marker
    visualization_msgs::Marker create(double x, double y, double z, double w, double l, double h, bool known, string frame, int id);


    /// \brief creates a marker array containing known obstacles for the initial state
    ///        of the navigation problem
    /// \return marker array containing initial known obstacles
    visualization_msgs::MarkerArray known_obstacles(string frame);

    /// \brief creates a marker array containing unknown obstacles for the inital state
    ///        of the navigation problem
    /// \param
    /// \return marker array containing initial unknown obstacles
    visualization_msgs::MarkerArray unknown_obstacles(string frame, int random);

    int check_obstacles(visualization_msgs::MarkerArray &known, visualization_msgs::MarkerArray &unknown, geometry_msgs::Pose navbot_pose);
}

#endif
