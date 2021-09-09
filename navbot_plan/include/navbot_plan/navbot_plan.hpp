/// \file
/// \brief Library for setting up navbot environment obstacles.

#ifndef NAVBOT_PLAN_INCLUDE_GUARD_HPP
#define NAVBOT_PLAN_INCLUDE_GUARD_HPP

#include <cmath>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace navbot_plan{

    using std::vector;

    struct Node{
        
        vector<double> pos;
        double f,g,h;
        Node *parent;

        /// \brief Default constructor - creates a Node object without
        ///        initializing any values
        Node();

        /// \brief Constructor - copies the values of a Node object into
        ///        a new Node object
        Node(Node* node);

        /// \brief Constructor - creates a Node object with initializing 
        ///        the pos and parent member variables
        /// \param pos - node position
        /// \param parent - points to the parent node
        Node(const vector<double> &pos, Node *parent);

        /// \brief Constructor - creates a Node object initializing all
        ///        member variables
        /// \param pos - node position
        /// \param f - node cost (g + h)
        /// \param g - cost of path up to this node
        /// \param h - hueristic future cost estimate
        /// \param parent - points to the parent node
        Node(const vector<double> &pos, const double &f, const double &g, const double &h, Node *parent);

        /// \brief Compares the cost of two nodes
        /// \param right - input node to compare to
        /// \return true if node has lower cost than input node
        bool operator<(const Node &right);

        /// \brief Returns the path of points from start to this node
        /// \return path from start to node
        vector<vector<double>> getPath();

        /// \brief Returns the locations of valid successors of a node in 3D space.
        ///        A successor represents a point in space where the next node in a
        ///        path could be created/selected.
        /// \param obstacles - invalid locations for nodes
        /// \param step - distance between node and successors
        /// \return maximum of 26 valid successor 
        vector<vector<double>> getSuccessors(const visualization_msgs::MarkerArray &obstacles, const double &step);

        /// \brief Checks to see if a given point in space is a valid successor. Splits
        ///        the line between the node and point into segments of length dp, checking
        ///        if any of the segment endpoints are within an obstacle
        /// \param obstacles - invalid locations for nodes
        /// \param point - point to check valid or not valid
        /// \param dp - length between checked endpoints
        /// \returns true if no endpoints (including point) are within an obstacle
        bool valid_successor(const visualization_msgs::MarkerArray &obstacles, const vector<double> &point, const double &dp);

        /// \brief Outputs the position of a node as a string
        /// \return position of node
        std::string string();

    };

    /// \brief Takes an input position vector and returns it as a string
    /// \return outputs position "pos[0] pos[1] ... pos[n]"
    std::string to_string(const vector<double> &pos);

    /// \brief Calculates euclidean distance between two points
    /// \param a - vector representing a point
    /// \param b - vector representing a second point
    /// \return distance between two points
    double distance(const vector<double> &a, const vector<double> &b);

    /// \brief Checks if a point is within an invalid location with a 10% buffer for safety. 
    /// \param obstacle - invalid location bounding box
    /// \param point - point to check
    /// \return true if point not in obstacle
    bool is_valid(const visualization_msgs::Marker &obstacle, const vector<double> &point);

    /// \brief creates N-1 vector of points evenly spaced along line between points
    ///        a and b
    /// \param a - start of line for interpolation
    /// \param b - end of line for interpolation
    /// \return vector of points between a and b-- include b
    vector<vector<double>> interpolate(const vector<double> &a, const vector<double> &b, const int &N);


    bool cmp(Node* a, Node* b);
    
    /// \brief finds the optimal path between start and end, avoiding obstacles.
    /// \param start - starting point of path
    /// \param end - end point of path
    /// \param obstacles - invalid locations for the path to cross
    /// \param step - distance between nodes in the path.
    /// \return vector of points that form the optimal path
    vector<vector<double>> a_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step);

    /// \brief performs A* with active pruning at each node.
    /// \param start - starting point of path
    /// \param end - end point of path
    /// \param obstacles - invalid locations for the path to cross
    /// \param step - distance between nodes in the path.
    /// \return vector of points that form the optimal path
    vector<vector<double>> theta_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step);

    /// \brief performs theta* assuming line of sight between parent/child nodes.
    ///        checks line of sight when node is expanded.
    /// \param start - starting point of path
    /// \param end - end point of path
    /// \param obstacles - invalid locations for the path to cross
    /// \param step - distance between nodes in the path.
    /// \return vector of points that form the optimal path
    vector<vector<double>> lazy_theta_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step);

    /// \brief converts a path represented by a vector<vector<double>
    ///        into a nav_msgs::Path
    /// \param path - vector containing path of points {x,y,z}
    /// \return same path represented as poses in a nav_msgs::Path
    nav_msgs::Path nav_path(const vector<vector<double>> &path, const std::string &frame);

}


#endif
