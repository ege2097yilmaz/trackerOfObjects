# trackerOfObjects
multiple object tracking with longe range radar sensor

This package was created for multiple object tracking with ROS firmware. src folder cotains cpp files to develop algorthms. It is necesary to install Eigen, OPENCV 4.0 and PCL Libraries. These can be installed through ROS installation. ROS version is NOETİC-DESKTOP. the algorithm can track 10 object simultaneusly.


For installation;

cd /home/($username)/($your workspace)/src

git clone https://github.com/ege2097yilmaz/trackerOfObjects.git

cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin_make (if it is running slowly, "catkin_make -DCMAKE_BUILD_TYPE=Release" is recomended)


To run the algorithms, firstly run "roslaunch deneme main.launch" file, then run "rosbag play lrr_test_data.bag", which is in one_sensor_bag file. There are two bag files in this folder. Both can be runned through this command. 


![image](https://user-images.githubusercontent.com/109589040/187860323-9db87209-4e56-4872-b27d-7ebbda564e91.png)

![image](https://user-images.githubusercontent.com/109589040/187862505-aaa87246-e8dc-4a4d-875c-10dde754bf07.png)


After then, RVIZ can be seen with red cubes and also coloured cubes that indicate cluster centroid points, which are generated by tracker_node. There are three main nodes in the ROS. First is publishing radar data into /lrrObjects topics, second is to convert from float32multiarray to poincloud type then pulich the topic named as /filtered_cloud in order to use PCL library. Finally, third node try to find cluster centroid point using PCL, OPENCV liraries and FusionEKF.h file.  

References:

https://github.com/praveen-palanisamy/multiple-object-tracking-lidar

https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013

https://github.com/prat96/Centroid-Object-Tracking

https://pcl.readthedocs.io/projects/tutorials/en/latest

This implemetation referenced these sources.
