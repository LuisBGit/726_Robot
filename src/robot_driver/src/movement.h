#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <std_msgs/Int8.h>
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265
extern float conv;
extern float maxPoint;
extern int count;

extern float qx;
extern float qx;
extern float qy;
extern float qz;
extern float qw;
extern float x;
extern float y;
extern float yaw;
extern geometry_msgs::Twist velocityCommand;


extern float loopX[4];
extern float loopY[4];
extern float loopYaw[4];
extern float loopLinear[4];

enum moveSequence {
	ANGLE,
	LINEAR
};


enum STATE {
	findWall,
	findCorner,
	loop,
	dodge,
  identify,
  done
};

enum shape {
  Barrel,
  Container,
  Unknown,
  NotObject
};


extern int object; //0 = none, 1 = object, 2 = wall

extern STATE currentSTATE;
extern moveSequence currentMovement;
void systemFSM();
static void toEulerAngle(float qx, float qy, float qz, float qw, float& yaw);
bool within(float value, float compare, float percent);
float control(float input, float feedBack, float kp);


STATE findFirstWall();
STATE findFirstCorner();
STATE idealLoop();
STATE dodgeObstacle();

float getMagnitude(float x, float y);
//Get absolute length of a vector
float getMagnitude(float x, float y);
//Find the X and Y coordinates based on laser data not local x and y
void findXY(float length, float angle, float &x, float &y);
//Get lenght, angle, x and y of a point
void getParams(const sensor_msgs::LaserScan::ConstPtr& laserScanData, int point, float &length, float &angle, float &x, float &y);
//Finds a gradient between two points
float gradientFinder(float length1, float length2, float angle1, float angle2);

//checks if withinTolerance a percentage value
bool withinTolerance(float value, float compare, float percent);
//Detects if an object is present, finds two corners via the change in lengths
bool objectDetection(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint);
//Finds the closest point of the robot by finding shorest length
int getClosestPoint(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint);
//Detects what shape it is using a gradient check if linear then it is a container
shape shapeDetection(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint);
//Retrieves the dimensions of the rectangle subtracts the closest point from the end points and gets magnitude
void rectangleDimensions(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, float &width, float &length, int &startPoint, int &endPoint);
//Gets the radius of the circle by using chord length calcs
void circleDimensions(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, float &radius, int &startPoint, int &endPoint);

shape objectDetectionV2(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, int &startPoint, int &endPoint);
