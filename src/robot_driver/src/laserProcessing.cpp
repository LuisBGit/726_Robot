#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265




void findXY(float length, float angle, float &x, float &y) {
  x =  length * cos(angle);
  y = length * sin(angle);
}

float gradientFinder(float length1, float length2, float angle1, float angle2) {
  float x1 =0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  findXY(length1, angle1, x1, y1);
  findXY(length2, angle2, x2, y2);

  float gradient = fabs((y2 - y1)/(x2 - x1));
  ROS_INFO("Gradient: [%f]", gradient);
  return gradient;

}

bool within(float value, float compare, float percent) {
  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
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

  //if (startFound == true && endFound == true) {
    float startY = 0;
    float startX = 0;
    float endY = 0;
    float endX = 0;
    float startAngle = laserScanData->angle_increment * startPoint;
    float endAngle = laserScanData->angle_increment * endPoint;
    float startLength = laserScanData->ranges[startPoint];
    float endLength = laserScanData->ranges[endPoint];
    findXY(startLength, startAngle, startX, startY);
    findXY(endLength, endAngle, endX, endY);
    c = sqrt((startLength*startLength) + (endLength*endLength) - (2*startLength*endLength*cos(fabs((startAngle) - (endAngle)))));

    /*ROS_INFO("Start X, Start Y: [%f], [%f]", startX, startY);
    ROS_INFO("End X, End Y: [%f], [%f]", endX, endY);
    ROS_INFO("c: [%f], Angle: [%f]", c, fabs((laserScanData->angle_increment * startPoint) - (laserScanData->angle_increment * endPoint)) * conv);*/

  //}
  //ROS_INFO("c: [%f]", c);
    if (c > sqrt(2) || startFound == false || endFound == false || c == 0)
    {
     ROS_INFO("No OBJECT");
    } else {
     ROS_INFO("OBJECT");
     int startCheck = 0;
     int endCheck = 0;
     float midPoint = fabs((startPoint + endPoint) / 2);
     float midAngle = laserScanData->angle_increment * midPoint;
     float midLength = laserScanData->ranges[midPoint];
     float slopeRefStart = gradientFinder(midLength, startLength, startAngle, midAngle);
     float slopeRefEnd = gradientFinder(midLength, endLength,endAngle,  midAngle);
     for (int i = startPoint; i <endPoint; i++) {
       if (within(gradientFinder(laserScanData->ranges[i], startLength, laserScanData->angle_increment * i, startAngle), slopeRefStart, 5)) {
          startCheck++;
       } else if (within(gradientFinder(laserScanData->ranges[i], endLength, laserScanData->angle_increment * i, endAngle), slopeRefEnd, 5)) {
         endCheck++;
       }
     }
     ROS_INFO("Start Count [%d]", startCheck );
     ROS_INFO("End Count [%d]", endCheck );
     if (endCheck > 5 || startCheck > 5) {
       ROS_INFO("SQUARE");
     } else {
       ROS_INFO("CIRCLE");
     }


    }
  }


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
