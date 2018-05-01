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
  float prevSlope = 0;
  int closestElement  = 0;
 //Determine which is the closest point
  for (int i = 0; i < rangeDataNum; ++i) {
    if (laserScanData->ranges[i]< closest) {
      closest = laserScanData->ranges[i];
      closestX = laserScanData->ranges[i] * cos((laserScanData->angle_increment * i)  );
      closestY = laserScanData->ranges[i] * sin((laserScanData->angle_increment * i) );
      angle = i * laserScanData->angle_increment * (180/PI);
      closestElement = i;
    }
    count = i;
    //ROS_INFO(" [%f] ", i * laserScanData->angle_increment * conv);
  }





  float startPoint = 0;
  float endPoint = rangeDataNum - 1;
  bool startFound = false;
  bool endFound = false;

  //Find first point
  for (int i = 1; i < rangeDataNum; i++) {
    float diff = laserScanData->ranges[i] - laserScanData->ranges[i - 1];

    if (diff <= -0.05 && startFound == false) {
      startFound = true;
      startPoint = i;
      //ROS_INFO("i: [%f], i -1: [%f]", laserScanData->ranges[i], laserScanData->ranges[i - 1]);
    } else if (diff >= 0.05 && endFound == false && startFound == true){
      endFound = true;
      endPoint = i - 1;
    }

  }


  int slopeCheck = 0;
  float startY = laserScanData->ranges[startPoint] * sin((laserScanData->angle_increment * startPoint));
  float startX = laserScanData->ranges[startPoint] * cos((laserScanData->angle_increment * startPoint));
  float endY = laserScanData->ranges[endPoint] * sin((laserScanData->angle_increment * endPoint));
  float endX = laserScanData->ranges[endPoint] * cos((laserScanData->angle_increment * endPoint));
  ROS_INFO("Edges at [%f], [%f]", startPoint * laserScanData->angle_increment* conv, endPoint * laserScanData->angle_increment * conv);
  ROS_INFO("Edges at element [%f], [%f]", startPoint, endPoint);
  ROS_INFO("Length Start: [%f], End: [%f]", laserScanData->ranges[startPoint], laserScanData->ranges[endPoint]);
  ROS_INFO("Start Coords [%f], [%f]", startX, startY);
  ROS_INFO("End Coords [%f], [%f]", endX, endY);

  if (startPoint > endPoint) {
    ROS_INFO("Edges mixed up");
  }

  if (startFound == false ) {
    ROS_INFO("A start edge was not found");
  }

  if (endFound == false) {
    ROS_INFO("An end edge was not found");
  }

  //ROS_INFO("startX: [%f], endX: [%f]", startX, endX);
  if ((fabs(startX - endX) > sqrt(2)) || (fabs(startX - endX) == 0) || startFound == false || endFound == false)
 {
   ROS_INFO("No OBJECT");
 } else {
   ROS_INFO("OBJECT");
 }
/*
  float slopeRef = fabs((closestY - startY) / (closestX - startX));


  for (int i = startPoint; i <= closestElement; ++i) {
    float currentY = laserScanData->ranges[i] * sin((laserScanData->angle_increment * i));
    float currentX = laserScanData->ranges[i] * cos((laserScanData->angle_increment * i));
    float slope = fabs((currentY - startY) / (currentX - startX));

    if ((slope  <= (slopeRef + (0.10 * slopeRef ))) && (slope  >= (slopeRef - (0.10 * slopeRef )))) {
      slopeCheck++;
      ROS_INFO("INSIDE TOLERANCE:, Within [%d]", slopeCheck);
    }
    ROS_INFO("Slope ref: [%f], Slope: [%f]", slopeRef, slope);

  }

  if (slopeCheck >= 5) {
    ROS_INFO("SQUARE");
  } else {
    ROS_INFO("CIRCLE");
  }

  //ROS_INFO("Start X: [%f], closestX: [%f], startY: [%f], closestY: [%f], Slope: [%f]", startX, closestX, startY, closestY, slopeRef);




*/

  //ROS_INFO("Cloest Object is at Coordinate: [%f], [%f] at angle [%f] with distance of [%f]", closestX, closestY, angle, closest);
  //ROS_INFO("Angle Min = [%f], Angle Max = [%f], Angle Increment = [%f], Count numbers = [%f]", laserScanData->angle_min, laserScanData->angle_max, laserScanData->angle_increment,rangeDataNum);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserProcessing");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1000, laserScanCallback);
  ros::spin();
  return 0;
}
