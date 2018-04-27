#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  float lowest = 100;
  for (int i = 0; i < rangeDataNum; ++i) {
    if (laserScanData->ranges[i] < lowest) {
      lowest = laserScanData->ranges[i];
    }
  }
  ROS_INFO("I heard: [%f]", lowest);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserProcessing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1000, laserScanCallback);
  ros::spin();
  return 0;
}
