#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265


void checkGradient(int point1, int point2, float &gradient) {

}

void findXY(int point, float &x, float &y) {
  x = laserScanData->ranges[point] * cos(laserScanData->angle_increment * point);
  y = laserScanData->ranges[point] * cos(laserScanData->angle_increment * point);
}

bool within(float value, float compare, float percent)
{
  if (value >= (compare - (percent * compare/100)) || value <= (compare + (percent * compare/100))) {
    return true;
  } else {
    return false;
  }
}

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
 float c = 0;

  for (int i = 1; i < (rangeDataNum - 1); i++) {

    prev1 = laserScanData->ranges[i -1];
    next1 = laserScanData->ranges[i +1];
    current = laserScanData->ranges[i];


    prevDiff = current - prev1;
    nextDiff = next1 - current;

    if (prevDiff <= -0.05 && startFound == false)  {
      startFound = true;
      startPoint = i;
    } else if ((nextDiff >= 0.05) && endFound == false) {
      endFound = true;
      endPoint = i;
    }
  }
  ROS_INFO("\n\n");

  if (startFound == true && endFound == true) {
    int slopeCheck = 0;
    float startY = laserScanData->ranges[startPoint] * sin(laserScanData->angle_increment * startPoint);
    float startX = laserScanData->ranges[startPoint] * cos((laserScanData->angle_increment * startPoint));
    float endY = laserScanData->ranges[endPoint] * sin((laserScanData->angle_increment * endPoint));
    float endX = laserScanData->ranges[endPoint] * cos((laserScanData->angle_increment * endPoint));
    float a = laserScanData->ranges[startPoint];
    float b = laserScanData->ranges[endPoint];


    c = sqrt((a*a) + (b*b) - (2*a*b*cos(fabs((laserScanData->angle_increment * startPoint) - (laserScanData->angle_increment * endPoint)))));
/*
    ROS_INFO("Start X, Start Y: [%f], [%f]", startX, startY);
    ROS_INFO("End X, End Y: [%f], [%f]", endX, endY);
    ROS_INFO("c: [%f], Angle: [%f]", c, fabs((laserScanData->angle_increment * startPoint) - (laserScanData->angle_increment * endPoint)) * conv);*/
  }

  //ROS_INFO("c: [%f]", c);
  if (c > sqrt(2) || startFound == false || endFound == false || c == 0)
 {
   ROS_INFO("No OBJECT");
 } else {
   ROS_INFO("OBJECT");
   float midPoint = fabs((startPoint + endPoint) / 2)

  }


 }


  //ROS_INFO("Cloest Object is at Coordinate: [%f], [%f] at angle [%f] with distance of [%f]", closestX, closestY, angle, closest);
  //ROS_INFO("Angle Min = [%f], Angle Max = [%f], Angle Increment = [%f], Count numbers = [%f]", laserScanData->angle_min, laserScanData->angle_max, laserScanData->angle_increment,rangeDataNum);



int main(int argc, char **argv)
{
  ros::init(argc, argv, "dataProcess");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, laserScanCallback);

	ros::Rate loop_rate(10);// loop 10 Hz

	while(ros::ok()) // publish the velocity set in the call back
	{
		ros::spinOnce();
		loop_rate.sleep();


	}
  return 0;
}
