#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  float closest = 1000;
  float closestX = 1000;
  float closestY = 1000;
  float angle = 0;
  float count = 0;
  float smallestXInc = 9999;
  float conv = 180/PI;


 //Determine which is the closest point
  for (int i = 0; i < rangeDataNum; ++i) {
    if (laserScanData->ranges[i]< closest) {
      closest = laserScanData->ranges[i];
      closestX = laserScanData->ranges[i] * cos((laserScanData->angle_increment * i)  );
      closestY = laserScanData->ranges[i] * sin((laserScanData->angle_increment * i) );
      angle = i * laserScanData->angle_increment * (180/PI);
    }
    count = i;
    //ROS_INFO(" [%f] ", i * laserScanData->angle_increment * conv);
  }

  float startPoint = 0;

  //Find first point
  for (int i = 1; i < rangeDataNum; ++i) {
    float diff = laserScanData->ranges[i] - laserScanData->ranges[i - 1];
    if (diff < -0.05) {
      startPoint = i;
      ROS_INFO("i: [%f], i -1: [%f]", laserScanData->ranges[i], laserScanData->ranges[i - 1]);
      break;
    }

  }
  float endPoint = 0;
  //Seems to be working without walls
  for (int i = rangeDataNum - 2; i >= 0 ; --i) {
    float diff = laserScanData->ranges[i] - laserScanData->ranges[i + 1];
    if (diff < -0.05) {
      endPoint = i;
      break;
    }
    //ROS_INFO(" End Angle Differences: [%f]",diff);
  }



  ROS_INFO("Cloest Object is at Coordinate: [%f], [%f] at angle [%f] with distance of [%f]", closestX, closestY, angle, closest);
  //ROS_INFO("Angle Min = [%f], Angle Max = [%f], Angle Increment = [%f], Count numbers = [%f]", laserScanData->angle_min, laserScanData->angle_max, laserScanData->angle_increment,rangeDataNum);

  ROS_INFO("Edges at [%f], [%f]", startPoint * laserScanData->angle_increment* conv, endPoint * laserScanData->angle_increment * conv);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserProcessing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1000, laserScanCallback);
  ros::spin();
  return 0;
}
