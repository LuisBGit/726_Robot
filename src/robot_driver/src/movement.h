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
	done
};


extern int object; //0 = none, 1 = object, 2 = wall

extern STATE currentSTATE;
extern moveSequence currentMovement;
void systemFSM();
static void toEulerAngle(float qx, float qy, float qz, float qw, float& yaw);
bool within(float value, float compare, float percent);
float control(float input, float feedBack, float kp);
