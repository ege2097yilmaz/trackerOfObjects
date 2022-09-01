//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
// #include <ros/console.h>

// STL
#include <iostream>
#include <bits/stdc++.h>
#include <cmath>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

// for logging
// if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//    ros::console::notifyLoggerLevelsChanged();
// }

// defining constants
#define M_PI 3.14159265358979323846
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) //this is used for computing radian
// # define my_sizeof(type) ((char *)(&type+1)-(char*)(&type)) //sizeof macro to get arrat size

std::string input_topic = "/lrrObjects", output_topic = "/cloud_out", frame_id = "base_link";

// defining Pointcloud object for PCL lib
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// visualizer class
class arrayMarker{
public:
    arrayMarker(){ // consructor

    // RVİZ marker
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_array", 1);
    // poincloud publiher object
    pub = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);
    // subscriber for radar topic 
    sub = n.subscribe(input_topic, 1000, &arrayMarker::chatterCallback, this); }

    float x_pos, y_pos, ins_theta = 40.0, increment = 0.0;
    int i, radar = 128; //radar datas

    int height = 1, width = 256; //hieht is 1 for 2D data
    sensor_msgs::PointCloud2 output;

    void chatterCallback(const std_msgs::Float32MultiArrayConstPtr& msg); //subs callback
    void visualizer(); //RVİZ 

    float near_clip = 0.02, far_clip = 70, view_angle = 40.0; //long rande radar has maximum range is 70 m and nearest range is 0.02 
    // these infos are in radar datasheet
    //here linear distacne was scaled by 10 times
private:
    // ros objects
    ros::Publisher marker_pub;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
};

void arrayMarker::chatterCallback(const std_msgs::Float32MultiArrayConstPtr& msg){

    ROS_INFO_THROTTLE(6, "converting from array to pointcloud of state is active");

    // defining view angle
    float view_angle_r ;
    view_angle_r = degreesToRadians(view_angle);
    // receving float32multiarrays from radar bag file
    std::vector<float> depth_raw = msg->data;

     // defining PCL cloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // int k = 1;
    unsigned int datalen = height * width; //setting data length
    float scale = (far_clip - near_clip) / 1.0; //maximum and minimum distance from radar datas
    
    std::vector<float> x_scale, y_scale;
    float f = float(float(std::max(height, width)) / 2) / float(tan(view_angle_r  / 2));

    // ROS_INFO("Callback is working");
    ros::Rate r(30); //frequency

    // storing every pointcloud points into the point pcl pattern inside the resolution
    for(i = 0; i < (radar + 1); i++)
    {
        float value = msg->data[i];
        if(value <= 0.0){value = 70.0;}

        //defining new reference for width variable
        x_pos = -1*value * std::cos(degreesToRadians(ins_theta + increment)); 
        y_pos = value * std::sin(degreesToRadians(ins_theta + increment));

        // pcl points storage
        pcl::PointXYZRGB p;
        p.x = x_pos; // //storage the datas into scales
        p.y = y_pos;
        p.z = 0.0; //always this is zero
        // it is assumed that radar datas are 2D
        p.r = 0;
        p.g = 0;
        p.b = 0;

        // putting variables into the pcl::point object
        cloud->points.push_back(p);

        // for the next point
        increment += 0.46; 

        // ros publishing pointcloud2 process
        pcl::toROSMsg(*cloud.get(),output);
        output.header.frame_id = frame_id;
        
        // std::cout<<"detected: "<<x_pos<<" and "<< y_pos<<std::endl;
        visualizer();
    }

    pub.publish (output);
    increment = 0.0;
    r.sleep();
}
void arrayMarker::visualizer(){
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // visualizer object
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = i; //changing index for every cube
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15; //defining cubes diameterz
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.r = 1.0f; //visualize parameters
    marker.color.g = 0.0f; //make sure alpha is more than 0.0 to be viewed
    marker.color.b = 0.1f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_WARN_ONCE("Please create a subscriber to the marker");
    // std::cout<<"X_POS: ******************"<<x_pos<<std::endl;
    marker_pub.publish(marker);}

int main( int argc, char** argv )
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "points_and_lines");
    ROS_INFO("STARTING float32multiarray to pointcloud ...");

    // Create a ROS node handle
    ros::NodeHandle n; 

      
    // input and output sub and pub
    arrayMarker arrayMarker; 
    
    // Don't exit the program.
    ros::spin();
    return 0;
}