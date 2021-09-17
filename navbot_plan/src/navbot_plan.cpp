#include "navbot_plan/navbot_plan.hpp"

namespace navbot_plan{

    ///////////////////////////
    // Node Functions
    ///////////////////////////

    Node::Node() {};

    Node::Node(Node* node) : pos{node->pos}, f{node->f}, g{node->g}, h{node->h}, parent{node->parent} {}

    Node::Node(const vector<double> &pos, Node *parent) : pos{pos}, parent{parent} {}

    Node::Node(const vector<double> &pos, const double &f, const double &g, const double &h, Node *parent) : 
        pos{pos}, f{f}, g{g}, h{h}, parent{parent} {}

    vector<vector<double>> Node::getPath(){

        vector<vector<double>> path = {pos};
        Node current = *this;

        while (current.parent != nullptr){
            current = *current.parent;
            path.push_back(current.pos);
        }

        return path;
    }

    vector<vector<double>> Node::getSuccessors(const visualization_msgs::MarkerArray &obstacles, const double &step){

        vector<vector<double>> successors;

        vector<double> X = {pos[0] - step, pos[0], pos[0] + step};
        vector<double> Y = {pos[1] - step, pos[1], pos[1] + step};
        vector<double> Z = {pos[2] - step, pos[2], pos[2] + step};

        for (auto x : X){
            for (auto y : Y){
                for (auto z : Z){
                    if ((x == pos[0] and y == pos[1] and z == pos[2]) or z < 5 or !valid_successor(obstacles,{x,y,z},step/5)){
                        continue;
                    } else{
                        successors.push_back({x,y,z});
                    }
                }
            }
        }

        return successors;
    }

    bool Node::valid_successor(const visualization_msgs::MarkerArray &obstacles, const vector<double> &point, const double &dp){

        // calculate distance between node and new point
        double dist = distance(pos,point);

        // calculate number of points needed
        int N = int(dist/dp);

        // interpolate points
        vector<vector<double>> points = interpolate(pos,point,N);

        // Check if all points are valid
        for (auto p : points){
            for (auto obstacle : obstacles.markers){
                if (!is_valid(obstacle,p)){
                    return false;
                }
            }
        }

        return true;

    }

    bool Node::operator < (const Node &right){
        return f < right.f;
    }

    std::string Node::string(){
        std::string out = to_string(pos);
        return out;
    }
    

    ////////////////////////////
    // General Functions
    ////////////////////////////

    std::string to_string(const vector<double> &pos){
        std::string out = "";
        for (auto p : pos){
            out += std::to_string(p) + " ";
        }
        return out;
    }

    double distance(const vector<double> &a, const vector<double> &b){

        double out = 0;
        for (int i = 0; i < a.size(); i++){
            out += pow(a[i] - b[i],2);
        }

        return sqrt(out);
    }

    bool is_valid(const visualization_msgs::Marker &obstacle, const vector<double> &point){

        // Calculate bounds for each dimension with 15% buffer
        double x_lower = obstacle.pose.position.x - 1.15*obstacle.scale.x/2;
        double x_upper = obstacle.pose.position.x + 1.15*obstacle.scale.x/2;
        double y_lower = obstacle.pose.position.y - 1.15*obstacle.scale.y/2;
        double y_upper = obstacle.pose.position.y + 1.15*obstacle.scale.y/2;
        double z_lower = obstacle.pose.position.z - 1.15*obstacle.scale.z/2;
        double z_upper = obstacle.pose.position.z + 1.15*obstacle.scale.z/2;

        bool in_x = point[0] > x_lower and point[0] < x_upper;
        bool in_y = point[1] > y_lower and point[1] < y_upper;
        bool in_z = point[2] > z_lower and point[2] < z_upper;

        if (in_x and in_y and in_z){
            return false;
        }else{
            return true;
        }

    }

    vector<vector<double>> interpolate(const vector<double> &a, const vector<double> &b, const int &N){

        // x,y,z single step sizes for point interpolation
        double dx = (b[0] - a[0])/N;
        double dy = (b[1] - a[1])/N;
        double dz = (b[2] - a[2])/N;

        // interpolate points
        vector<vector<double>> out;

        for (int i = 1; i<N-1; i++){
            out.push_back({a[0] + i*dx, a[1] + i*dy, a[2] + i*dz});
        }

        out.push_back(b);

        return out;

    }

    vector<vector<double>> a_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step){

        using std::set;
        using std::string;
        using std::unordered_map;

        // initialize open and closed lists
        Node end_node(end,nullptr);
        Node starting_node(start,0,0,0,nullptr);

        set<Node*> open_list;
        unordered_map<string,Node*> nodes;
        unordered_map<string,int> closed_list;
        set<Node*>::iterator index;

        open_list.insert(&starting_node);
        nodes[to_string(start)] = &starting_node;
        closed_list[to_string(start)] = 1;

        
        while(!open_list.empty()){

            // Find the node with lowest f value and select this as current node
            Node* current_node = *open_list.begin();

            // std::cout << to_string(current_node->pos) << std::endl;
            // If current_node is the destination, then we are done.
            if (distance(current_node->pos,end) <= step){
                end_node.parent = current_node;
                break;
            }

            // Remove current_node from the open_list 
            open_list.erase(current_node);

            // Add current_node to closed list
            closed_list[current_node->string()] = 1;

            // Evaluate successor nodes
            vector<vector<double>> successors = current_node->getSuccessors(obstacles,step);
            // std::cout << successors.size() << std::endl;
            for (auto successor : successors){

                // If already evaluated completely (in closed list) skip the node
                if (closed_list.find(to_string(successor)) != closed_list.end() ) {
                    continue;
                }

                // Calculate g score for this node on this current path
                double tempG = current_node->g + distance(current_node->pos,successor);

                // If node is new, add it to the open list. If the node is not new but is already part
                // of a better path, skip the node.
                index = open_list.find(nodes[to_string(successor)]);
                Node* next_node;
                if (index != open_list.end()){
                    next_node = *index;
                    if (tempG<=next_node->g){
                        next_node->parent = current_node;
                        next_node->g = tempG;
                        open_list.erase(index);
                    } else {
                        continue;
                    }
                } else {
                    next_node = new Node(successor, current_node);
                    next_node->g = tempG;
                    next_node->h = distance(successor,end);
                }

                next_node->f = next_node->g + next_node->h;
                open_list.insert(next_node);
                nodes[to_string(next_node->pos)] = next_node;
                
            }

        }

        // form path from linked nodes
        vector<vector<double>> path = end_node.getPath();

        std::reverse(path.begin(), path.end());

        // std::cout << std::endl;
        std::cout << end_node.parent->g + distance(end,end_node.parent->pos) << std::endl;
        std::cout << path.size() << std::endl;

        return path;
    }

    vector<vector<double>> theta_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step){

        using std::set;
        using std::string;
        using std::unordered_map;

        // initialize open and closed lists
        Node end_node(end,nullptr);
        Node starting_node(start,0,0,0,nullptr);

        set<Node*> open_list;
        unordered_map<string,Node*> nodes;
        unordered_map<string,int> closed_list;
        set<Node*>::iterator index;

        open_list.insert(&starting_node);
        nodes[to_string(start)] = &starting_node;
        closed_list[to_string(start)] = 1;

        
        while(!open_list.empty()){

            // Find the node with lowest f value and select this as current node
            Node* current_node = *open_list.begin();

            // std::cout << to_string(current_node->pos) << std::endl;
            // If current_node is the destination, then we are done.
            if (distance(current_node->pos,end) <= step){
                end_node.parent = current_node;
                break;
            }

            // Remove current_node from the open_list 
            open_list.erase(current_node);

            // Add current_node to closed list
            closed_list[current_node->string()] = 1;

            // Evaluate successor nodes
            vector<vector<double>> successors = current_node->getSuccessors(obstacles,step);
            // std::cout << successors.size() << std::endl;
            for (auto successor : successors){

                // If already evaluated completely (in closed list) skip the node
                if (closed_list.find(to_string(successor)) != closed_list.end() ) {
                    continue;
                }

                // Calculate g score for this node on this current path
                double tempG = current_node->g + distance(current_node->pos,successor);

                // If node is new, add it to the open list. If the node is not new but is already part
                // of a better path, skip the node.
                index = open_list.find(nodes[to_string(successor)]);
                Node* next_node;
                if (index != open_list.end()){
                    next_node = *index;
                    if (tempG<=next_node->g){
                        next_node->parent = current_node;
                        next_node->g = tempG;
                        open_list.erase(index);
                    } else {
                        continue;
                    }
                } else {
                    next_node = new Node(successor, current_node);
                    next_node->g = tempG;
                    next_node->h = distance(successor,end);
                }

                // Theta* code -- not sure why ! is required but that's the only way to make it work
                if (current_node->parent and current_node->parent->valid_successor(obstacles,successor,step/5)){
                    double cost = current_node->parent->g + distance(current_node->parent->pos,successor);
                    if (cost <= tempG){
                        next_node->parent = current_node->parent;
                        next_node->g = cost;
                    }
                }

                next_node->f = next_node->g + next_node->h;
                open_list.insert(next_node);
                nodes[to_string(next_node->pos)] = next_node;
                
            }

        }

        // form path from linked nodes
        vector<vector<double>> path = end_node.getPath();

        std::reverse(path.begin(), path.end());

        // std::cout << std::endl;
        std::cout << end_node.parent->g + distance(end,end_node.parent->pos) << std::endl;
        std::cout << path.size() << std::endl;

        return path;
    }

    vector<vector<double>> lazy_theta_star(const vector<double> &start, const vector<double> &end, 
    const visualization_msgs::MarkerArray &obstacles, const double &step){

        /*

            NOTE!

            performance is unexpectedly worse than theta*

            TODO optimize data structures, general implementation and test against theta*

        */
       
        using std::set;
        using std::string;
        using std::unordered_map;

        // initialize open and closed lists
        Node end_node(end,nullptr);
        Node starting_node(start,0,0,0,nullptr);

        set<Node*> open_list;
        unordered_map<string,Node*> nodes;
        unordered_map<string,Node*> closed_list;
        set<Node*>::iterator index;

        open_list.insert(&starting_node);
        nodes[to_string(start)] = &starting_node;
        
        while(!open_list.empty()){

            // Find the node with lowest f value and select this as current node
            Node* current_node = *open_list.begin();

            // std::cout << to_string(current_node->pos) << std::endl;
            // If current_node is the destination, then we are done.
            if (distance(current_node->pos,end) <= step){
                end_node.parent = current_node;
                break;
            }

            // Remove current_node from the open_list 
            open_list.erase(current_node);

            // Add current_node to closed list
            closed_list[current_node->string()] = current_node;

            // Evaluate successor nodes
            vector<vector<double>> successors = current_node->getSuccessors(obstacles,step);

            if (current_node->parent and !current_node->parent->valid_successor(obstacles,current_node->pos,step/5)){
                double min = 999999999999;
                for (auto successor : successors){
                    if (closed_list.find(to_string(successor)) != closed_list.end() ){
                        double cost = closed_list[to_string(successor)]->g + distance(current_node->pos,closed_list[to_string(successor)]->pos);
                        if (cost < min){
                            min = cost;
                            current_node->parent = closed_list[to_string(successor)];
                            current_node->g = cost;
                            current_node->f = current_node->g + current_node->h;
                        }
                    }
                }
            }

            // std::cout << successors.size() << std::endl;
            for (auto successor : successors){

                // If already evaluated completely (in closed list) skip the node
                if (closed_list.find(to_string(successor)) != closed_list.end() ) {
                    continue;
                }

                // Calculate g score for this node on this current path
                double tempG = current_node->g + distance(current_node->pos,successor);

                // If node is new, add it to the open list. If the node is not new but is already part
                // of a better path, skip the node.
                index = open_list.find(nodes[to_string(successor)]);
                Node* next_node;
                if (index != open_list.end()){
                    next_node = *index;
                    if (tempG<=next_node->g){
                        next_node->parent = current_node;
                        next_node->g = tempG;
                        open_list.erase(index);
                    } else {
                        continue;
                    }
                } else {
                    next_node = new Node(successor, current_node);
                    next_node->g = tempG;
                    next_node->h = distance(successor,end);
                }

                if (current_node->parent){
                    double cost = current_node->parent->g + distance(current_node->parent->pos,successor);
                    if (cost <= tempG){
                        next_node->parent = current_node->parent;
                        next_node->g = cost;
                    }
                }

                next_node->f = next_node->g + next_node->h;
                open_list.insert(next_node);
                nodes[to_string(next_node->pos)] = next_node;
                
            }

        }

        // form path from linked nodes
        vector<vector<double>> path = end_node.getPath();

        std::reverse(path.begin(), path.end());

        // std::cout << std::endl;
        std::cout << end_node.parent->g + distance(end,end_node.parent->pos) << std::endl;
        std::cout << path.size() << std::endl;

        return path;
    }


    nav_msgs::Path nav_path(const vector<vector<double>> &path, const std::string &frame){

        geometry_msgs::PoseStamped pose;
        nav_msgs::Path poses;
        
        poses.header.frame_id = frame;
        pose.header.frame_id = frame;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        for (auto point : path){
            pose.pose.position.x = point[0];
            pose.pose.position.y = point[1];
            pose.pose.position.z = point[2];

            poses.poses.push_back(pose);
        }

        return poses;
    }
}