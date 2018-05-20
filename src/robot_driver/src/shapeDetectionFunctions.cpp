#include "movement.h"

//Author: Luis Borja
//Title: Shape Identifier



#define PI 3.14159265


//Get absolute length of a vector
float getMagnitude(float x, float y) {
	return sqrt((x*x) + (y*y));
}

//Find the X and Y coordinates based on laser data not local x and y
void findXY(float length, float angle, float &x, float &y) {
  x =  length * cos(angle);
  y = length * sin(angle);
}

//Get lenght, angle, x and y of a point
void getParams(const sensor_msgs::LaserScan::ConstPtr& laserScanData, int point, float &length, float &angle, float &x, float &y) {
  length = laserScanData->ranges[point];
  angle = laserScanData->angle_increment * point;
	findXY(length, angle, x, y);
}

//Finds a gradient between two points
float gradientFinder(float length1, float length2, float angle1, float angle2) {
  float x1 =0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  findXY(length1, angle1, x1, y1);
  findXY(length2, angle2, x2, y2);

  float gradient = fabs((y2 - y1)/(x2 - x1));
  return gradient;

}

//checks if withinTolerance a percentage value
bool withinTolerance(float value, float compare, float percent) {
  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
    return true;
  } else {
    return false;
  }
}

//Detects if an object is present, finds two corners via the change in lengths
bool objectDetection(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint) {
  bool startFound = false;
  bool endFound = false;
  bool found = false;
  float conv = 180/PI;
  for (int i = 1; i < size - 1; i++) {
    float tolerance = 0.1;
    float prev = i - 1;
    float next = i + 1;
    float nextLength, nextAngle, nextX, nextY;
    float prevLength, prevAngle, prevX, prevY;
    float currentLength, currentAngle, currentX, currentY;
    getParams(laserScanData, prev, prevLength, prevAngle, prevX, prevY);
    getParams(laserScanData, i, currentLength, currentAngle, currentX, currentY);
    getParams(laserScanData, next, nextLength, nextAngle, nextX, nextY);
    if ((((currentLength - prevLength) <= -0.05)) && startFound == false) {
      startPoint = i;
      startFound = true;
    } else if (((nextLength - currentLength) >= 0.05) && endFound == false) {
      endPoint = i;
      endFound = true;
    }

  }



  if (startFound && endFound) {
    float startLength, startAngle, startX, startY;
    float endLength, endAngle, endX, endY;
    getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
    getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);
    float c = sqrt((startLength*startLength) + (endLength*endLength) - (2*startLength*endLength*cos((endPoint - startPoint) * laserScanData->angle_increment)));
    if (c == 0 || c > sqrt(2)) {
      ROS_INFO("WALL");
      return false;
    } else {
      ROS_INFO("OBSTACLE");
      return true;
    }
  } else {
    ROS_INFO("NOTHING");
    return false;
  }
}
//Finds the closest point of the robot by finding shorest length
int getClosestPoint(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint) {
  int closestPoint = startPoint;
  float closestLength, closestAngle, closestX, closestY;
  for (int i = startPoint; i < endPoint; i++) {

    float closestLength, closestAngle, closestX, closestY;
    float currentLength, currentAngle, currentX, currentY;
    getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
    getParams(laserScanData, i, currentLength, currentAngle, currentX, currentY);
    if (closestLength > currentLength) {
      closestPoint = i;
    }
  }
  return closestPoint;
}

shape objectDetectionV2(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint) {
  float xArray[512];
  float yArray[512];
  for (int i = 0; i < size; i++) {
    
  }


}

//Detects what shape it is using a gradient check if linear then it is a container
shape shapeDetection(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint) {
  int startCheck = 0;
  int endCheck = 0;
  float closestLength, closestAngle, closestX, closestY;
  int closestPoint = getClosestPoint(laserScanData, size, startPoint, endPoint);
  float startLength, startAngle, startX, startY;
  float endLength, endAngle, endX, endY;
  getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
  getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
  getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);

  float slopeRefStart = gradientFinder(closestLength, startLength, closestAngle, startAngle);
  float slopeRefEnd = gradientFinder(closestLength, endLength,closestAngle,  startAngle);

  for (int i = startPoint; i <endPoint; i++) {
    float currentLength = laserScanData->ranges[i];
    float currentAngle = laserScanData->angle_increment * i;
    float currentToStart = gradientFinder(startLength, currentLength, startAngle, currentAngle);
    float endToCurrent = gradientFinder(endLength, currentLength, endAngle, currentAngle);
    if (withinTolerance(currentToStart, slopeRefStart, 5)) {
       startCheck++;
    } else if (withinTolerance(endToCurrent, slopeRefEnd, 5)) {
      endCheck++;
    }

  }
  if (endCheck > 10 || startCheck > 10) {
    return Container;
  } else {
    return Barrel;
  }

}
//Retrieves the dimensions of the rectangle subtracts the closest point from the end points and gets magnitude
void rectangleDimensions(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, float &width, float &length, int &startPoint, int &endPoint) {
  int closestPoint = getClosestPoint(laserScanData, size, startPoint, endPoint);
  float closestLength, closestAngle, closestX, closestY;
  float startLength, startAngle, startX, startY;
  float endLength, endAngle, endX, endY;
  getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
  getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);
  getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
  float side1 = getMagnitude(closestX - startX, closestY - startY);
  float side2 = getMagnitude(endX - closestX, endY - closestY);
  if (side1 > side2) {
    width = side2;
    length = side1;
  } else {
    width = side1;
    length = side2;
  }

}
//Gets the radius of the circle by using chord length calcs
void circleDimensions(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, float &radius, int &startPoint, int &endPoint) {
  int closestPoint = getClosestPoint(laserScanData, size, startPoint, endPoint);
  float closestLength, closestAngle, closestX, closestY;
  float startLength, startAngle, startX, startY;
  float endLength, endAngle, endX, endY;
  getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
  getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);
  getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
  float arcWidth = fabs(closestLength - startY);
  float chordWidth = fabs(endX - startX);
  radius = ((4 * arcWidth * arcWidth) + (chordWidth*chordWidth)) / (8*arcWidth);

}
//CallBack
/*void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
  shape object;
  float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
  float conv = 180/PI;
  float x,y,length, angle;
  int startPoint, endPoint;
  float yOffset = 0.05 + (0.5 * 0.46);

  bool result = objectDetection(laserScanData, rangeDataNum, startPoint, endPoint);
  if (result) {
	//An object has been found, ID it
    ROS_INFO("OBJECT FOUND!! - ID: %d", 1);
	//Shape detection of the object
    object = shapeDetection(laserScanData,rangeDataNum, startPoint, endPoint);
    int closestPoint = getClosestPoint(laserScanData, rangeDataNum, startPoint, endPoint);
    float closestLength, closestAngle, closestX, closestY;
    float startLength, startAngle, startX, startY;
    float endLength, endAngle, endX, endY;
    float localX, localY;
    getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
    getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);
    getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
	//Appropriate dimension and coordinate output
    if (object == Container) {
      float length, width;
      rectangleDimensions(laserScanData, rangeDataNum, width, length, startPoint, endPoint);
      localX = ((startY + endY) / 2) + yOffset;
      localY = (startX + endX)/ 2;

      ROS_INFO("Type: Container, Length: %.2fcm, Width: %.2fcm, X: %.2f, Y: %.2f ", length * 100, width * 100, localX, localY);
    } else {
      float radius;
      circleDimensions(laserScanData, rangeDataNum, radius, startPoint, endPoint);
      localX = closestY + radius + yOffset;
      localY = (startX + endX)/ 2;
      ROS_INFO("Type: Barrel, Radius: [%.2f]cm, X: [%.2f], Y: [%.2f]", radius * 100, localX, localY);
    }

  } else {
    ROS_INFO("NO OBJECT FOUND");
  }

}*/
