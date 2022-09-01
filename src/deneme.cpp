// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

// STL
#include <iostream>
#include <bits/stdc++.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

// defining Pointcloud object for PCL lib
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// defining constants
#define M_PI 3.14159265358979323846
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) //this is used for computing radian
# define my_sizeof(type) ((char *)(&type+1)-(char*)(&type)) //sizeof macro to get arrat size

// ros publisher object
ros::Publisher pub;

//ros topics input and output and alsı frame name
std::string input_topic = "/lrrObjects", output_topic = "/cloud_out", frame_id = "camera_link_optical";

// objects which are relatee to frame of the datas coming from radar
int height = 1, width = 256; //hieht is 1 for 2D data 
//width data was scaled 2 time to visualize better in rviz

float near_clip = 0.02, far_clip = 70, view_angle = 40.0; //long rande radar has maximum range is 70 m and nearest range is 0.02 
                                                         // these infos are in radar datasheet
                                                         //here linear distacne was scaled by 10 times

void cloud_cb (const std_msgs::Float32MultiArrayConstPtr& msg)
 {
  // defining view angle
  float view_angle_r ;
  view_angle_r = degreesToRadians(view_angle);

  // receving float32multiarrays from radar bag file
  std::vector<float> depth_raw = msg->data;

  // int size = my_sizeof(depth_raw)/my_sizeof(depth_raw[0]);
  // std::cout<<"msg size is: "<<size<<std::endl;//to figure out the amount of data

  // converting 0.0 datas to maximum distance
  for( int index = 0; index < 129; index++){
    if (depth_raw[index] == 0.0){depth_raw[index] = 7;}
      // std::cout<<"depth_rawis: "<<index<<std::endl;}   
  }

  // defining PCL cloud object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // int k = 1;
  unsigned int datalen = height * width; //setting data length
  float scale = (far_clip - near_clip) / 1.0; //maximum and minimum distance from radar datas
  
  std::vector<float> x_scale, y_scale;
  float f = float(float(std::max(height, width)) / 2) / float(tan(view_angle_r  / 2)); 
  
  // storing every pointcloud points into the point pcl pattern inside the resolution
  for (int j = 0; j < height; j++){
    float y = (j - height / 2.0);  //defining new reference for width variable
    for (int i = 0; i < width; i++){
      int k = j * width + i;
      float x = -(i - width / 2.0);
      x_scale.push_back(float(x / f)); //storage the datas into scales
      y_scale.push_back(float(y / f));

      float depth = near_clip + scale * depth_raw[k];
      float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
     
      // float xyz[3] = {a*k, b*k, c};
      
      // pcl points storage
      pcl::PointXYZRGB p;
      p.x = xyz[0];
      p.y = xyz[1];
      p.z = xyz[2];
      p.r = 0;
      p.g = 0;
      p.b = 0;

      // putting variables into the pcl::point object
      cloud->points.push_back(p);

      // a = a + 0.000001; these are for previous attempts
      // b = b + 0.0000000002;
      // c = c - 0.0000001;
      // k++;
    }
  }

  // ros publishing pointcloud2 process
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud.get(),output);
  output.header.frame_id = frame_id;

  // publish
  pub.publish (output);
  return;
 }

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init (argc, argv, "my_pcl");
  ROS_INFO("STARTING float32multiarray to pointcloud ...");

  // Create a ROS node handle
  ros::NodeHandle nh;
  
  // input and output sub and pub
  ros::Subscriber sub = nh.subscribe (input_topic, 1000, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

  // Don't exit the program.
  ros::spin();
  return 0;
}