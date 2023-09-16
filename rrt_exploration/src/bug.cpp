#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"


#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint, start;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points;
int height,width;
float resolution,Xstartx,Xstarty,init_map_x,init_map_y, origin_position_x, origin_position_y;
std::vector<std::vector<int>> occupancy_grid;
std::vector<int8_t> occupancy_data;


void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

 // Odczytaj szerokość i wysokość mapy
    width = msg->info.width;
    height = msg->info.height;
    float origin_orientation_w = msg->info.origin.orientation.w;
    float origin_orientation_x = msg->info.origin.orientation.x;
    float origin_orientation_y = msg->info.origin.orientation.y;
    float origin_orientation_z = msg->info.origin.orientation.z;
    origin_position_x = msg->info.origin.position.x;
    origin_position_y = msg->info.origin.position.y;

    // Odczytaj rozdzielczość mapy (metry na piksel)
    resolution = msg->info.resolution;
    // ROS_INFO("resolution, %d", resolution);
    // Odczytaj dane o zajętości (0 - wolne, 100 - zajęte, -1 - nieznane)
    occupancy_data = msg->data;
    

    // Przekształć dane o zajętości na dwuwymiarowy wektor
    occupancy_grid.resize(height, std::vector<int>(width, 0));


    mapData=*msg;
    
    // ROS_INFO("callback, %ld", mapData.data.size());
    // ROS_INFO("origin_orientation_w, %f", origin_orientation_w);
    // ROS_INFO("origin_orientation_x, %f", origin_orientation_x);
    // ROS_INFO("origin_orientation_y, %f", origin_orientation_y);
    // ROS_INFO("origin_orientation_z, %f", origin_orientation_z);

    // ROS_INFO("origin_position_x, %f", origin_position_x);
    // ROS_INFO("origin_position_y, %f", origin_position_y);

    
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);

}




int main(int argc, char** argv)
{
    // generate the same numbers as in the original C test program
    ros::init(argc, argv, "BUG");
    ros::NodeHandle nh;

    std::string map_topic,base_frame_topic;

    std::string ns;
    ns=ros::this_node::getName();

    ros::param::param<std::string>(ns+"/map_topic", map_topic, "argo_mini/map");
    
    ros::Subscriber sub= nh.subscribe(map_topic, 2 ,mapCallBack);	
    ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 2 ,rvizCallBack);	

    ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);

    // Inicjalizacja tf2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate rate(2); 

ros::Time start_time = ros::Time::now();
    // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (ros::ok() && mapData.data.size() < 1 && (ros::Time::now() - start_time).toSec() < 200.0)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    if (mapData.data.size() < 1)
    {
        ROS_ERROR("Nie odebrano mapy w czasie.");
        return 1; // Wyjście z programu z kodem błędu
    }


//visualizations  points and lines..
points.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
	
points.ns= "markers";
points.id = 0;


points.type = points.POINTS;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
points.pose.orientation.w =1.0;
points.scale.x=0.3; 
points.scale.y=0.3; 


points.color.r = 255.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=1.0;
points.lifetime = ros::Duration();

geometry_msgs::Point p;  


while(points.points.size()<5)
{
ros::spinOnce();

pub.publish(points) ;
}




std::vector<float> temp1;
temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);
	
std::vector<float> temp2; 
temp2.push_back(points.points[2].x);
temp2.push_back(points.points[0].y);


init_map_x=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);

temp2.push_back(points.points[0].x);
temp2.push_back(points.points[2].y);

init_map_y=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

Xstartx=(points.points[0].x+points.points[2].x)*.5;
Xstarty=(points.points[0].y+points.points[2].y)*.5;





geometry_msgs::Point trans;
trans=points.points[4];
std::vector< std::vector<float>  > V; 
std::vector<float> xnew; 
// xnew.push_back( trans.x);xnew.push_back( trans.y);  
// V.push_back(xnew);

points.points.clear();
pub.publish(points) ;

std::vector<float> frontiers;

// goal.header.stamp=ros::Time(0);
// goal.header.frame_id=mapData.header.frame_id;
// goal.point.x=Xstartx; //dla xy i x/y_pixel pojawiaja sie frontiers ale sa nie na mapie 
// goal.point.y=Xstarty;
// goal.point.z=0.0;

// targetspub.publish(goal);
// Main loop

while (ros::ok()){



    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            occupancy_grid[i][j] = occupancy_data[i * width + j];
            float x = (i * resolution ) + origin_position_y; //- 17.7999
            float y = (j * resolution )+ origin_position_x; //- 21.0
            int x_pixel = static_cast<int>(x);  // Przeliczenie na piksele
            int y_pixel = static_cast<int>(y);
            int occupancy_value = occupancy_grid[i][j];
            // ROS_INFO("i, %ld", x_pixel);
            // ROS_INFO("j, %ld", y_pixel);
            // ROS_INFO("w, %ld", width);
            // ROS_INFO("h, %ld", height);
           
        
            
            if (occupancy_value == 100){
                // ROS_INFO("og, %d", occupancy_value);
                exploration_goal.header.stamp=ros::Time(0);
                // geometry_msgs::PointStamped exploration_goal;
                exploration_goal.header.frame_id=mapData.header.frame_id;
                exploration_goal.point.x=y_pixel; //dla xy i x/y_pixel pojawiaja sie frontiers ale sa nie na mapie 
                exploration_goal.point.y=x_pixel;
                exploration_goal.point.z=0.0;
                
                p.x=y_pixel; 
                p.y=x_pixel; 
                p.z=0.0;
                points.points.push_back(p);
                pub.publish(points) ;
                targetspub.publish(exploration_goal);
                points.points.clear();

                // try {
                //     geometry_msgs::PointStamped transformed_point;
                //     tf_buffer.transform(exploration_goal, transformed_point, "map");
                //     ROS_INFO("Punkt w ramce mapy: %f, %f, %f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
                //     targetspub.publish(transformed_point);
                // } catch (tf2::TransformException &ex) {
                //     ROS_WARN("Nie można przekształcić punktu do ramki mapy: %s", ex.what());
                // }
            }
        }
    }
    

ros::spinOnce();
rate.sleep();
}

return 0;

}









