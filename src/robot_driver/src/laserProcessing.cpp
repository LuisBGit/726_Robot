#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  float lowest = 100;
  float lowestX = 0;
  float lowestY = 0;
  for (int i = 0; i < rangeDataNum; ++i) {
    if (laserScanData->ranges[i]< lowest) {
      lowest = laserScanData->ranges[i];
      lowestX = laserScanData->ranges[i] * sin((laserScanData->angle_increment * i)* PI / 180);
      lowestY = laserScanData->ranges[i] * cos((laserScanData->angle_increment * i)* PI / 180);
    }
  }
  ROS_INFO("Cloest Object is at Coordinate: [%f], [%f]", lowestX, lowestY);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserProcessing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1000, laserScanCallback);
  ros::spin();
  return 0;
}
