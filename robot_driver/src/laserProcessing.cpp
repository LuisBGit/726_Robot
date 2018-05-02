#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265
#define angleInc laserScanData->angle_increment



void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  float conv = 180/PI;
  float prev1 = 0;
  float next1 = 0;
  float prevDiff = 0;
  float nextDiff = 0;
  float current = 0;

 //Determine which is the closest point
 float startPoint = 0;
 float endPoint = rangeDataNum - 1;
 bool startFound = false;
 bool endFound = false;


  for (int i = 1; i < rangeDataNum - 1; i += 1) {

    //float prev2 = laserScanData->ranges[i -2];
    prev1 = laserScanData->ranges[i -1];
    next1 = laserScanData->ranges[i +1];
    //float next2 = laserScanData->ranges[i +2];
    current = laserScanData->ranges[i];


    prevDiff = current - prev1;
    nextDiff = next1 - current;

    if (prevDiff <= -0.05 && startFound == false)  {
      startFound = true;
      startPoint = i;

      ROS_INFO("Start CORNER AT [%f] degrees", angleInc * i * conv);
    } else if ((nextDiff >= 0.05) && endFound == false) {
      ROS_INFO("End CORNER AT [%f] degrees", angleInc * i * conv);
      endFound = true;
      endPoint = i;
    }

  }
  ROS_INFO("\n \n \n");


  int slopeCheck = 0;
  float startY = laserScanData->ranges[startPoint] * sin((laserScanData->angle_increment * startPoint));
  float startX = laserScanData->ranges[startPoint] * cos((laserScanData->angle_increment * startPoint));
  float endY = laserScanData->ranges[endPoint] * sin((laserScanData->angle_increment * endPoint));
  float endX = laserScanData->ranges[endPoint] * cos((laserScanData->angle_increment * endPoint));
  float startLength = laserScanData->ranges[startPoint];
  float endLength = laserScanData->ranges[endPoint];

  float midPoint = (endPoint + startPoint) / 2;
  float midLength = laserScanData->ranges[midPoint];
  float a= startLength;
  float b = endLength;

  float c = sqrt((a*a) + (b*b) - (2*a*b*cos(fabs((angleInc * startPoint) - (angleInc * endPoint)))));
  ROS_INFO("Start X, Start Y: [%f], [%f]", startX, startY);
  ROS_INFO("End X, End Y: [%f], [%f]", endX, endY);
  ROS_INFO("c: [%f]", c);

  //ROS_INFO("startX: [%f], endX: [%f]", startX, endX);
  if (c > sqrt(2) || startFound == false || endFound == false)
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
