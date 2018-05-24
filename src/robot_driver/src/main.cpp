#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <std_msgs/Int8.h>
#include "sensor_msgs/LaserScan.h"
#include "shapeDetection.cpp"

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

enum state {
	Roam,
	Dodge,
	Scan
};

enum lookState {
	left,
	right,
	done,
	noEscape
};

state currentState = Roam;
lookState currentLook = left;

float qx = 0;
float qy = 0;
float qz = 0;
float qw = 0;
float x = 0;
float y = 0;
float yaw = 0;
float refYaw = 0;
bool rotate = true;



int object = 0; //0 = none, 1 = object, 2 = wall

float idealPathX[12] = {0, 8, 8, 0, 1, 7, 7, 1, 2, 6, 6, 2};
float idealPathY[12] = {2, 2, -2, -2, 1, 1, -1, -1, 0.5, 0.5, -0.5, -0.5};

static void toEulerAngle(float qx, float qy, float qz, float qw, float& yaw) {
	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);

  yaw = atan2(siny, cosy) * conv;

  if (yaw < 0) {
    yaw += 360;
  }
}

float constrain(float value) {
	if (value < 0) {
		if (value < -0.5) {
			return -0.5;
		} else {
			return value;
		}
	} else {
		if (value > 0.5) {
			return 0.5;
		} else {
			return value;
		}
	}
}

void local2Global(float lX, float lY, float theta, float &globeX, float &globeY, float posX, float posY) {
	globeX = (lX * cos(theta/conv)) - (lY * sin(theta/conv)) + posX;
	globeY = (lX* sin(theta/conv)) + (lY * cos(theta/conv)) + posY;
	ROS_INFO("X:[%f], Y:[%f]", globeX, globeY);
}

bool withinLinear(float value, float compare, float percent) {
	if (compare == 0) {
		if (value >= -0.1 && value <= 0.1) {
			ROS_INFO("TRUE");
			return true;
		} else {
			return false;
		}
	}
  ROS_INFO("Value [%f], Compare [%f], Percent [%f]", value, compare, percent);
  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
    ROS_INFO("Normal Check");
    return true;
  } else {
    return false;
  }
}

bool withinRotate(float value, float compare, float percent) {
	ROS_INFO("Angle Error [%f]", value);
	if (compare == 0) {
		if ((value >= -0.3 && value <= 0.3) || (value >= -0.3 && value <= 0.3)) {
			ROS_INFO("TRUE");
			return true;
		} else {
			return false;
		}
	}

  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
    ROS_INFO("TRUE");
    return true;
  } else {
    return false;
  }
}

//Rotate Control
float control(float input, float feedBack, float kp) {
  //ROS_INFO("error [%f] ", input - feedBack);
	float output = kp * (input - feedBack);
	return constrain(output);
}
//Linear Control
float linearControl(float input, float feedBack, float kp) {
	float output = kp * (input - feedBack);
	return constrain(fabs(output));
}

//Rotate to Desired
bool rotateRobot(float x2, float y2) {
	  float angle = atan2(y2 - y,x2 - x)*conv;
		if (angle < 0) (angle += 360);
		ROS_INFO("Desired Angle[%f]", angle );
		if (!withinRotate(angle- yaw, 0, 10)) {
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = control(angle, yaw, 0.02);
			return false;
		} else {
			velocityCommand.linear.x = 0;
			velocityCommand.angular.z = 0;
			return true;
		}
}

//Move to Ideal Linear
bool linearRobot(float x2, float y2) {
	float xSquare = (x2 - x) * (x2 - x);
	float ySquare = (y2 - y) * (y2 - y);
	float distanceTo = fabs(sqrt(xSquare + ySquare));
	ROS_INFO("Distance Left [%f]", distanceTo);
	if (!withinLinear(distanceTo, 0, 10)) {
		velocityCommand.linear.x = linearControl(0, distanceTo, 0.5);
		velocityCommand.angular.z = 0;
		return false;
	} else {
		velocityCommand.linear.x = 0;
		velocityCommand.angular.z = 0;
		return true;
	}
}

bool moveTo(float x2, float y2) {
	if (rotateRobot(x2, y2)) {
		if (linearRobot(x2, y2)) {
			return true;
		}
	}
	return false;
}

bool lookLeft() {
	ROS_INFO("LOOKING LEFT");
	float viewX = 0;
	float viewY = 0;
	local2Global(1, 0, yaw - 90, viewX, viewY, x, y);
	rotateRobot(viewX, viewY);
}

bool lookRight() {
	ROS_INFO("LOOKING LEFT");
	float viewX = 0;
	float viewY = 0;
	local2Global(-1, 0, yaw - 90, viewX, viewY, x, y);
	rotateRobot(viewX, viewY);

}


bool dodgeAlgo() {
	switch (currentLook) {
		case (left):
			lookLeft();
			break;

		case (right):
			lookRight();
			break;

		case (done):
			ROS_INFO("ESCAPE FOUND");
			break;
		case (noEscape):
			ROS_INFO("NO ESCAPE");
			break;


	}
}


bool checkFront(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

	int clearCheck1 = 218;
	int clearCheck2 = 293;
	int startPoint = 0;
	int endPoint = 0;

	for (int i = clearCheck1; i < clearCheck2; i++) {
			if (laserScanData->ranges[i] < 0.6) {
				return true;
			}
		}

	return false;
}



void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

	int clearCheck1 = 218;
	int clearCheck2 = 293;
	int startPoint = 0;
	int endPoint = 0;

	if (currentState == Roam) {
		for (int i = clearCheck1; i < clearCheck2; i++) {
			if (laserScanData->ranges[i] < 0.6) {
				ROS_INFO("Something is too close");
				if (objectDetection(laserScanData, rangeDataNum, startPoint, endPoint)) {
					ROS_INFO("TIME TO DODGE");
					currentState = Dodge;
					velocityCommand.linear.x = 0;
					velocityCommand.angular.z = 0;
				}
			}
		}

	} else if (currentState == Dodge) {
		if (currentLook == left) {

		}
	}



}

void positions(const nav_msgs::Odometry::ConstPtr& msg)
{
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
  qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	yaw  = 0;
	toEulerAngle(qx, qy, qz, qw, yaw);
  //ROS_INFO("YAW [%f]", yaw);
}


state roaming() {
	ROS_INFO("Moving to %f , %f", idealPathX[count], idealPathY[count]);

	if (moveTo(idealPathX[count], idealPathY[count])) {
		count++;
	}

	if (count > 12) {
		count = 0;
	}

	return Roam;
}

state dodging() {
	ROS_INFO("DODGING");

	dodgeAlgo();
	return Dodge;
}

state scanning() {
	return Scan;
}


void loop() {
	ROS_INFO("State [%d]", currentState);
	switch (currentState) {
		case (Roam):
			currentState = roaming();
			break;

		case (Dodge):
			currentState = dodging();
			break;

		case (Scan):
			currentState = scanning();
			break;
	}

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
    loop();
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
