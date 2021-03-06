#include "navbotsim/obstacles.hpp"


namespace obstacles{

    using visualization_msgs::MarkerArray;
    using visualization_msgs::Marker;

    Marker create(double x, double y, double z, double w, double l, double h, bool known, string frame, int id){

        // create obstacle object - assign relevant parameters
        Marker obstacle;

        obstacle.header.frame_id = frame;
        obstacle.id = id;
        obstacle.type = 1;                      // specifies rectangular prism
        obstacle.action = 0;                    // add marker/obstacle
        obstacle.pose.position.x = x;
        obstacle.pose.position.y = y;
        obstacle.pose.position.z = z + h/2;
        obstacle.pose.orientation.x = 0;
        obstacle.pose.orientation.y = 0;
        obstacle.pose.orientation.z = 0;
        obstacle.pose.orientation.w = 1;
        obstacle.scale.x = w;
        obstacle.scale.y = l;
        obstacle.scale.z = h;

        // assign color based on known/unknown status
        if (known){
            obstacle.color.b = 1;
            obstacle.color.g = 1;
            obstacle.color.a = 0.75;
        } else {
            obstacle.color.r = 1;
            obstacle.color.a = 0.25;
        }

        return obstacle;
    }

    visualization_msgs::MarkerArray known_obstacles(string frame){

        visualization_msgs::MarkerArray known;

        known.markers.push_back(create(0,0,0,5,5,45,true,frame,0));
        known.markers.push_back(create(1,4,0,5,10,20,true,frame,1));
        known.markers.push_back(create(0,3,0,7,5,30,true,frame,2));

        known.markers.push_back(create(25,-20,0,7,20,8,true,frame,3));

        known.markers.push_back(create(-20,20,0,10,7,13,true,frame,4));
        known.markers.push_back(create(-20,17,0,5,5,25,true,frame,5));

        known.markers.push_back(create(9,27,0,7,7,15,true,frame,6));

        known.markers.push_back(create(24,12,0,15,8,10,true,frame,7));
        
        known.markers.push_back(create(-5,15,0,6,6,32,true,frame,8));        

        known.markers.push_back(create(10,-17,0,8,5,20,true,frame,9));

        known.markers.push_back(create(-1,-22,0,6,7,20,true,frame,10));

        known.markers.push_back(create(-14,-17,0,15,7,18,true,frame,11));

        known.markers.push_back(create(-16,-3,0,5,5,34,true,frame,13));
        known.markers.push_back(create(-15,-5,0,6,9,23,true,frame,14));
        known.markers.push_back(create(-16,-2,0,10,7,15,true,frame,15));

        return known;
    }

    visualization_msgs::MarkerArray unknown_obstacles(string frame, int random){

        visualization_msgs::MarkerArray unknown;
        srand ( time(NULL) );

        unknown.markers.push_back(create(14,-5,0,5,5,29,false,frame,16));
        unknown.markers.push_back(create(0,-9,3,5,14,20,false,frame,17));
        unknown.markers.push_back(create(-10,9,3,14,5,20,false,frame,18));

        // int x,y,z;
        // for (int i = 0; i<random; i++){
        //     x = rand() % 40 - 20;
        //     y = rand() % 40 - 20;
        //     z = rand() % 15 + 10;

        //     unknown.markers.push_back(create(x,y,z,3,3,3,false,frame,19+i));

        // }

        return unknown;
    }

    int check_obstacles(MarkerArray &known, MarkerArray &unknown, geometry_msgs::Pose navbot_pose){

        for (int i = 0; i<unknown.markers.size(); i++){
            Marker obstacle = unknown.markers[i];
            if (obstacle.action != 0){
                continue;
            }
            double magnitude = sqrt(pow(obstacle.scale.x,2) + pow(obstacle.scale.y,2));
            double dx = obstacle.pose.position.x - navbot_pose.position.x;
            double dy = obstacle.pose.position.y - navbot_pose.position.y;
            double dist = sqrt(pow(dx,2) + pow(dy,2));

            if (dist < 7 + magnitude){

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
                unknown.markers[i].action = 2;
                known.markers.push_back(copy);

                return i;                
            }

        }

        return -1;

    }

    
}