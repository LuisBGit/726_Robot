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
	strafe,
	forward,
	goBack,
	noEscape
};

state currentState = Roam;
lookState currentLook = left;
shape foundShape;

float qx = 0;
float qy = 0;
float qz = 0;
float qw = 0;
float x = 0;
float y = 0;
float yaw = 0;
float refYaw = 0;
bool rotate = true;


bool leftTrigger = false;
bool rightTrigger = false;

bool leftFree = false;
bool rightFree = false;

float lViewX = 0;
float lViewY = 0;
float rViewX = 0;
float rViewY = 0;

float frontDodgeX = 0;
float frontDodgeY = 0;


int object = 0; //0 = none, 1 = object, 2 = wall

int stepCounter = 0;

float idealPathX[12] = {0, 8, 8, 2, 1, 7, 7, 1, 2, 6, 6, 2};
float idealPathY[12] = {2, 2, -2, -2, 1, 1, -1, -1, 0.5, 0.5, -0.5, -0.5};

//float idealPathX[7] = {0, 8, 8, 2, 3, 6, 2};
//float idealPathY[7] = {2, 2, -2, -2, 0, 0, 0};




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
	//ROS_INFO("X:[%f], Y:[%f]", globeX, globeY);
}

void displayDetails(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float rangeDataNum, int &startPoint, int &endPoint, shape object) {
  ROS_INFO("OBJECT FOUND!! - ID: %d", 1);
  int closestPoint = getClosestPoint(laserScanData, rangeDataNum, startPoint, endPoint);
  float closestLength, closestAngle, closestX, closestY;
  float startLength, startAngle, startX, startY;
  float endLength, endAngle, endX, endY;
  float localX, localY;
  float yOffset = 0.05 + (0.5 * 0.46);
  getParams(laserScanData, startPoint, startLength, startAngle, startX, startY);
  getParams(laserScanData, endPoint, endLength, endAngle, endX, endY);
  getParams(laserScanData, closestPoint, closestLength, closestAngle, closestX, closestY);
  if (object == Container) {
    float length, width;
    rectangleDimensions(laserScanData, rangeDataNum, width, length, startPoint, endPoint);
    localX = ((startY + endY) / 2) + yOffset;
    localY = (startX + endX)/ 2;
		float displayX;
		float displayY;

		local2Global(localY, localX, yaw - 90, displayX, displayY, x, y);

    ROS_INFO("Type: Container, Length: %.2fcm, Width: %.2fcm, X: %.2f, Y: %.2f ", length * 100, width * 100, displayX, displayY);
  } else {
    float radius;
    circleDimensions(laserScanData, rangeDataNum, radius, startPoint, endPoint);
    localX = closestY + radius + yOffset;
    localY = (startX + endX)/ 2;
		float displayX;
		float displayY;
		local2Global(localY, localX, yaw - 90, displayX, displayY, x, y);
    ROS_INFO("Type: Barrel, Radius: [%.2f]cm, X: [%.2f], Y: [%.2f]", radius * 100, displayX, displayY);
  }
}

bool withinLinear(float value, float compare, float percent) {
	if (compare == 0) {
		if (value >= -0.1 && value <= 0.1) {
			//ROS_INFO("TRUE");
			return true;
		} else {
			return false;
		}
	}
  //ROS_INFO("Value [%f], Compare [%f], Percent [%f]", value, compare, percent);
  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
    //ROS_INFO("Normal Check");
    return true;
  } else {
    return false;
  }
}

bool withinRotate(float value, float compare, float percent) {
	//ROS_INFO("Angle Error [%f]", value);
	if (compare == 0) {
		if ((value >= -0.3 && value <= 0.3) || (value >= -0.3 && value <= 0.3)) {
			//ROS_INFO("TRUE");
			return true;
		} else {
			return false;
		}
	}

  if (value >= (compare - (percent * compare/100)) && value <= (compare + (percent * compare/100))) {
    //ROS_INFO("TRUE");
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


void linearFreeMove() {
	velocityCommand.linear.x = 0.3;
	velocityCommand.angular.y = 0;
}

void rotateFreeMove() {
	velocityCommand.linear.x = 0;
	velocityCommand.angular.y = 0.3;
}

//Rotate to Desired
bool rotateRobot(float x2, float y2) {
	  float angle = atan2(y2 - y,x2 - x)*conv;
		if (angle < 0) (angle += 360);
		//ROS_INFO("Desired Angle[%f]", angle );
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
	//ROS_INFO("Distance Left [%f]", distanceTo);
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
	if (rotate == true && rotateRobot(x2, y2)) {
		/*if (linearRobot(x2, y2)) {
			return true;
		}*/
		rotate = false;
	}

	if ((rotate == false) && linearRobot(x2, y2) ) {
		rotate = true;
		return true;
	}
	return false;
}

void lookLeft(bool &trigger) {
	if (!trigger) {
		//ROS_INFO("LOOKING LEFT");
		/*float viewX = 0;
		float viewY = 0;
		local2Global(1, 0, yaw - 90, viewX, viewY, x, y);*/
		if (rotateRobot(lViewX, lViewY)) {
			trigger = true;
		}
	}
}

void lookRight(bool &trigger) {
	if (!trigger) {
		//ROS_INFO("LOOKING RIGHT");
		/*float viewX = 0;
		float viewY = 0;
		local2Global(-1, 0, yaw - 90, viewX, viewY, x, y);*/
		if (rotateRobot(rViewX, rViewY)) {
			trigger = true;
		}
	}

}


bool dodgeAlgo() {
	switch (currentLook) {
		case (left):
			lookLeft(leftTrigger);
			break;

		case (right):
			lookRight(rightTrigger);
			break;

		case (strafe):
			//ROS_INFO("ESCAPE FOUND");
			if (moveTo(frontDodgeX, frontDodgeY)) {
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0;
				currentLook = forward;
				if (rightFree) {
					local2Global(-2, 0, yaw - 90, frontDodgeX, frontDodgeY, x, y);
				} else if (leftFree) {
					local2Global(2, 0, yaw - 90, frontDodgeX, frontDodgeY, x, y);
				}

			}
			break;
		case (forward):
			if (moveTo(frontDodgeX, frontDodgeY)) {
				//ROS_INFO("TRAVELLED AFTER DODGE");
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0;
				currentLook = goBack;
				if (rightFree) {
					local2Global(-1.5, 0, yaw - 90, frontDodgeX, frontDodgeY, x, y);
				} else if (leftFree) {
					local2Global(1.5, 0, yaw - 90, frontDodgeX, frontDodgeY, x, y);
				}
			}
			break;

		case (goBack):
			if (moveTo(frontDodgeX, frontDodgeY)) {
				//ROS_INFO("ReturnedtoPath");
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0;
				currentState = Roam;
				leftTrigger = false;
				rightTrigger = false;
				leftFree = false;
				rightFree = false;
				currentLook = left;
				return true;
			}

			break;
		case (noEscape):
			//ROS_INFO("NO ESCAPE");
			break;


	}

	return false;
}


bool checkFront(const sensor_msgs::LaserScan::ConstPtr& laserScanData, float thresh) {
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

	int clearCheck1 = 218;
	int clearCheck2 = 293;


	for (int i = clearCheck1; i < clearCheck2; i++) {
			if (laserScanData->ranges[i] < thresh) {
				return true;
			}
		}

	return false;
}



void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{

	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);


	int startPoint = 0;
	int endPoint = 0;

	if (currentState == Roam) {
		if (checkFront(laserScanData, 0.7)) {
			if (objectDetection(laserScanData, rangeDataNum, startPoint, endPoint)) {
				//ROS_INFO("TIME TO DODGE");
				velocityCommand.linear.x = 0;
				velocityCommand.angular.z = 0;
				foundShape = shapeDetection(laserScanData, rangeDataNum, startPoint, endPoint);
				displayDetails(laserScanData, rangeDataNum, startPoint, endPoint, foundShape);
				currentState = Dodge;

				local2Global(1, 0, yaw - 90, rViewX, rViewY, x, y);
				local2Global(-1, 0, yaw - 90, lViewX, lViewY, x, y);
			}
		}
	} else if (currentState == Dodge) {
		if (currentLook == left) {
				if (leftTrigger) {
					if (checkFront(laserScanData, 2)) {
						//ROS_INFO("LEFT IS BLOCKED");
						velocityCommand.linear.x = 0;
						velocityCommand.angular.z = 0;
						currentLook = right;
					} else {
						//ROS_INFO("LEFT IS FREE");
						velocityCommand.linear.x = 0;
						velocityCommand.angular.z = 0;
						currentLook = strafe;
						leftFree = true;
						local2Global(0, 1.5, yaw - 90, frontDodgeX, frontDodgeY, x, y);
					}
				}
		} else if (currentLook == rightTrigger) {
			if (rightTrigger) {
				if (checkFront(laserScanData, 2)) {
					//ROS_INFO("RIGHT IS BLOCKED");
					currentLook = noEscape;
					velocityCommand.linear.x = 0;
					velocityCommand.angular.z = 0;
				} else {
					//ROS_INFO("RIGHT IS FREE");
					currentLook = strafe;
					velocityCommand.linear.x = 0;
					velocityCommand.angular.z = 0;
					rightFree = true;
					local2Global(0, 1.5, yaw - 90, frontDodgeX, frontDodgeY, x, y);
				}
			}
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
	//ROS_INFO("Moving to %f , %f", idealPathX[count], idealPathY[count]);

	if (moveTo(idealPathX[count], idealPathY[count])) {
		count++;
	}

	if (count > 12) {
		count = 12;
	}

	return Roam;
}

state dodging() {
	//ROS_INFO("DODGING");
	//ROS_INFO("left trigger [%d], rightTrigger [%d]", leftTrigger, rightTrigger);
	if (dodgeAlgo()) return Roam;
	return Dodge;
}

state scanning() {
	return Scan;
}


void loop() {
	//ROS_INFO("State [%d]", currentState);
	switch (currentState) {
		case (Roam):
			currentState = roaming();
			break;

		case (Dodge):https://www.youtube.com/results?search_query=ys+movie
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
