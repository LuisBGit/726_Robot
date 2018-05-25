#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.14159265

float refStart= 0;
float refEnd = 0;


float getMagnitude(float x, float y) {
	return sqrt((x*x) + (y*y));
}


void findXY(float length, float angle, float &x, float &y) {
  x =  length * cos(angle);
  y = length * sin(angle);
}

void getParams(const sensor_msgs::LaserScan::ConstPtr& laserScanData, int point, float &length, float &angle, float &x, float &y) {
  length = laserScanData->ranges[point];
  angle = laserScanData->angle_increment * point;
	findXY(length, angle, x, y);
}


float gradientFinder(float length1, float length2, float angle1, float angle2) {
  float x1 =0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  findXY(length1, angle1, x1, y1);
  findXY(length2, angle2, x2, y2);

  float gradient = fabs((y2 - y1)/(x2 - x1));
  //ROS_INFO("Gradient: [%f], start Gradient: [%f], end Gradient: [%f]", gradient, refStart, refEnd);
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
    float startAngle = laserScanData->angle_increment * (startPoint + 1);
    float endAngle = laserScanData->angle_increment * (endPoint - 1);
    float startLength = laserScanData->ranges[startPoint];
    float endLength = laserScanData->ranges[endPoint];
    findXY(startLength, startAngle, startX, startY);
    findXY(endLength, endAngle, endX, endY);
    c = sqrt((startLength*startLength) + (endLength*endLength) - (2*startLength*endLength*cos((endPoint - startPoint) * laserScanData->angle_increment)));

    ROS_INFO("Start X, Start Y: [%f], [%f]", startX, startY);
    ROS_INFO("End X, End Y: [%f], [%f]", endX, endY);
    ROS_INFO("Start Angle: [%f], End Angle: [%f]", startAngle * conv, endAngle * conv);

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
     float slopeRefStart = gradientFinder(midLength, startLength, midAngle, startAngle);
     float slopeRefEnd = gradientFinder(midLength, endLength,midAngle,  startAngle);
     refStart = slopeRefStart;
     refEnd = slopeRefEnd;
     for (int i = startPoint; i <endPoint; i++) {
       float curretLength = laserScanData->ranges[i];
       float currentAngle = laserScanData->angle_increment * i;
       float currentToStart = gradientFinder(startLength, curretLength, startAngle, currentAngle);
       float endToCurrent = gradientFinder(endLength, curretLength, endAngle, currentAngle);
       if (within(currentToStart, slopeRefStart, 5)) {
          startCheck++;
       } else if (within(endToCurrent, slopeRefEnd, 5)) {
         endCheck++;
       }
       ROS_INFO("I: [%d], Angle: [%f]", i, currentAngle * conv);
       ROS_INFO("Current to Start: [%f], Start Reference: [%f]", currentToStart, slopeRefStart);
       ROS_INFO("End to Current: [%f], Start Reference: [%f]", endToCurrent, slopeRefEnd);
       ROS_INFO("\n\n");
     }
     ROS_INFO("Start Count [%d]", startCheck );
     ROS_INFO("End Count [%d]", endCheck );
     if (endCheck > 10 || startCheck > 10) {
       ROS_INFO("Crate");
	   float corner = 0;
       float cornerPoint = 0;
	   float cornerAngle = 0;
	   for (int i = startPoint + 1; i < endPoint - 1; i++) {
		 prev1 = laserScanData->ranges[i -1];
    	 next1 = laserScanData->ranges[i +1];
         current = laserScanData->ranges[i];
		 prevDiff = current - prev1;
    	 nextDiff = next1 - current;
		 if (prevDiff <0 && nextDiff > 0) {
			cornerPoint = i;
			corner = laserScanData->ranges[i];
			cornerAngle = laserScanData->angle_increment * i;
		 }


	   }
	   float cornerX = 0;
	   float cornerY = 0;
	   findXY(corner, cornerAngle, cornerX, cornerY);
	   float side1 = roundf(getMagnitude(cornerX - startX, cornerY - startY) * 100) /100;
	   float side2 = roundf(getMagnitude(endX - cornerX, endY - cornerY) * 100) / 100;
	   ROS_INFO("Side1: [%f], Side2: [%f]", side1, side2);
     } else {
       ROS_INFO("Barrel");


	   //float length1 = roundf((startLength * tan((M_PI/2) - startAngle)) * 100 ) / 100;
	   //float length2 = roundf((endLength * tan((M_PI/2) - (M_PI - endAngle))) * 100 ) / 100;
	   //float average = (length1 + length2)/2;

		 float closest = 999;
		 int closestPoint = 0;
		 float closestAngle =0;
		 for (int i = startPoint; i <= endPoint; i++) {

				if (closest > laserScanData->ranges[i]) {
					closest = laserScanData->ranges[i];
					closestAngle = laserScanData->angle_increment * i;
					closestPoint = i ;
				}

		 }
		 float closestX = 0;
		 float closestY = 0;

		 findXY(closest, closestAngle, closestX, closestY);
		 float arcWidth = fabs(closest - startY);
		 float chordWidth = fabs(endX - startX);
		 float radius = roundf(((4 * arcWidth * arcWidth) + (chordWidth*chordWidth)) / (8*arcWidth) * 100)/ 100;
		 ROS_INFO("h: [%f], chordWidth: [%f]", arcWidth, chordWidth);
	   ROS_INFO("Chord Style Radius: [%f], Diameter: [%f]", radius, radius * 2);
 	 	 //ROS_INFO("COS style Radius: [%f], Diameter: [%f]", average, average * 2);

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
