#include "movement.h"

geometry_msgs::Twist velocityCommand;

#define PI 3.14159265
float conv = 180/PI;
float maxPoint = sqrt((1) + (1));
int count = 0;
/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/

float loopYaw[5] = {0.0, -90.0, 180.0, 90.0, 0};
float loopLinear[4] = {9, -2, 0, 2};

float qx = 0;
float qy = 0;
float qz = 0;
float qw = 0;
float x = 0;
float y = 0;
float yaw = 0;
float refYaw = 0;

int object = 0; //0 = none, 1 = object, 2 = wall

STATE currentSTATE = findWall;
moveSequence currentMovement = ANGLE;

static void toEulerAngle(float qx, float qy, float qz, float qw, float& yaw)
{
	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
	yaw = atan2(siny, cosy);
}


bool within(float value, float compare, float percent) {

	if (compare == 0) {
		//ROS_INFO("ZERO DESIRED");
		if (value <= 0.001 && value >= -0.001) {
			return true;
		} else {
			return false;
		}
	}

	if (value > 0) {
		/*ROS_INFO("Positive");
		ROS_INFO("LOWERBOUND [%f]", compare - (percent * compare/100));
		ROS_INFO("UPPERBOUND [%f]", compare + (percent * compare/100));*/
	  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
		//	ROS_INFO("within [%f]", percent * 100);
	    return true;
	  } else {
	    return false;
	  }
	} else {
		if (value <= (compare - (percent * compare/100)) && value >= (compare + (percent * compare/100))) {
			/*ROS_INFO("Negative");
			ROS_INFO("LOWERBOUND [%f]", compare - (percent * compare/100));
			ROS_INFO("UPPERBOUND [%f]", compare + (percent * compare/100));
			ROS_INFO("within [%f]", percent * 100);*/
			return true;
		} else {
			return false;
		}
	}

}

float control(float input, float feedBack, float kp) {
	float output = kp * (input - feedBack);
	//ROS_INFO("error: [%f]", input - feedBack);
	return fabs(output);

}

int rAngle2Index(float angle, float increment) {
	float index = angle / increment;
	return index;
}

int dAngle2Index(float angle, float increment) {
	float index = angle * (1/conv) / increment;
	return index;
}

bool wallLeft(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float size, float distance) {
	float counter = 0;
	for (int i = dAngle2Index(90, laserScanData->angle_increment); i < size; i++) {
		float cX, cY;

		findXY(laserScanData->ranges[i], laserScanData->angle_increment*i, cX, cY);
		ROS_INFO("cX: [%f]", cX);
		if (fabs(cX) < distance) {

			counter++;
		}
	}

	if (counter > dAngle2Index(180 - 90, laserScanData->angle_increment)) {
		ROS_INFO("WALL Found");
		return true;
	} else {
		return false;
	}
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

		//These two variables are clearances at approx robot width assuming robot witdth = 0.46
	int clearCheck1 = 218;
	int clearCheck2 = 293;
	int startPoint = 0;
	int endPoint = 0;
	if (wallLeft(laserScanData, rangeDataNum, 1)) {
		velocityCommand.angular.z = 0;
		currentSTATE = loop;
	}

}

void positions(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Updates the position of the robot w.r.t. world frame
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
  qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	yaw  = 0;
	toEulerAngle(qx, qy, qz, qw, yaw);
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "movement");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
	ros::Subscriber sub = my_handle.subscribe("/odom", 1000, positions);
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);

	ros::Rate loop_rate(10);// loop 10 Hz

	while(ros::ok()) // publish the velocity set in the call back
	{
		ros::spinOnce();
		loop_rate.sleep();
		systemFSM();
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
