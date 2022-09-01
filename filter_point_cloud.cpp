// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>

// STL
#include <iostream>

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

// defining Pointcloud object for PCL lib
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// ros publisher object
ros::Publisher pub;

//ros topics input and output and alsı frame name
std::string input_topic = "/cloud_out", output_topic = "/filtered_cloud", frame_id = "base_link";

// callback for noise pointcloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_DEBUG_THROTTLE(10,"filtering function is activated ...");

    // converting rosmsg to pcl data
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *input_cloud);
    
    //filtering for NAN datas in the topic
    // std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*input_cloud, *source_cloud, *indices);

    // // printing indices into the terminal
    // std::cout<<"indices are: "<<indices->size()<<std::endl;
    // std::cout<<"input-cloud indices are: "<<input_cloud->size()<<std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(source_cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*source_cloud); //filtering

    // Publish the filtered cloud
    sensor_msgs::PointCloud2 output; // Create a container for the data.
    pcl::toROSMsg(*source_cloud.get(),output);
    output.header.frame_id = frame_id;

    pub.publish (output);
    // std::cout<<"published onto"<<output_topic<<std::endl;
}

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init (argc, argv, "filterer_pointcloud");
  ROS_INFO("Running filtrer node ...");

  // Create a ROS node handle
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe (input_topic, 1000, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

  // Don't exit the program.
  ros::spin();
  return 0;
}