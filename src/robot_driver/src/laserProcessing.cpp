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
  float angle = 0;
  float count = 0;
  float smallestXInc = 9999;
  float conv = 180/PI;


 //Determine which is the closest point
  for (int i = 0; i < rangeDataNum; i++) {
    if (laserScanData->ranges[i]< lowest) {
      lowest = laserScanData->ranges[i];
      lowestX = laserScanData->ranges[i] * sin((laserScanData->angle_increment * i)* PI / 180);
      lowestY = laserScanData->ranges[i] * cos((laserScanData->angle_increment * i)* PI / 180);
      angle = i * laserScanData->angle_increment * (180/PI);
    }
    count = i;
    //ROS_INFO(" [%f] ", i * laserScanData->angle_increment * conv);
  }

  float startPoint = 0;

  //Find first point
  for (int i = 1; i < rangeDataNum; i++) {
    float diff = (laserScanData->ranges[i]*  cos((laserScanData->angle_increment * i)* PI / 180)) - (laserScanData->ranges[i - 1]  * cos((laserScanData->angle_increment * (i - 1)* PI / 180)));
    if (diff > 0) {
      startPoint = i;

      break;
    }
  }
  ROS_INFO("[%f]", startPoint);
  float endPoint = 0;
  //Seems to be working without walls
  for (int i = rangeDataNum - 2; i >= 0 ; i--) {
    float diff = (laserScanData->ranges[i]*  cos((laserScanData->angle_increment * i)* PI / 180)) - (laserScanData->ranges[i + 1]  * cos((laserScanData->angle_increment * (i + 1)* PI / 180)));
    if (diff < 0) {
      endPoint = i;
      break;
    }
  }



//  ROS_INFO("Cloest Object is at Coordinate: [%f], [%f] at angle [%f]", lowestX, lowestY, angle);
  ROS_INFO("Angle Min = [%f], Angle Max = [%f], Angle Increment = [%f], Count numbers = [%f]", laserScanData->angle_min, laserScanData->angle_max, laserScanData->angle_increment,rangeDataNum);


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
