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

float loopX[4] = {9.0, 9.0, .0, 0.0};
float loopY[4] = {0.0, 0.0, 0.0, 0.0};
float loopYaw[4] = {0.0, -90.0, 180.0, 90.0};
float loopLinear[4] = {9, -2, 0, 2};

float qx = 0;
float qy = 0;
float qz = 0;
float qw = 0;
float x = 0;
float y = 0;
float yaw = 0;
/*
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
*/
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

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
	float count = 0;

	int startPoint = 0;
	int endPoint = 0;

	for (int i = 0; i < rangeDataNum; i++) {
		if (laserScanData->ranges[i] < 1) {
			//ROS_INFO("Something is too close");
			velocityCommand.angular.z = 0;
			velocityCommand.linear.x = 0;
		}
	}

}

void positions(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Print the odom data structure
	//Message sequence
	/*ROS_INFO("Seq: [%d]", msg->header.seq);
	//Position
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	//Orientation
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//Velocity
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/

	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
  qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	yaw  = 0;
	toEulerAngle(qx, qy, qz, qw, yaw);
	//ROS_INFO("X: [%f], Y: [%f], Yaw: [%f]", x, y, yaw * conv);
}

/*void systemFSM() {
	switch(currentSTATE) {
		case(findWall):
			velocityCommand.angular.z = control(90, yaw*conv, 0.01);
			if (within(yaw*conv, 90, 2)) {
				velocityCommand.angular.z = 0;
				currentSTATE = findCorner;
			}
			break;
		case(findCorner):
			velocityCommand.linear.x = control(2, y, 0.5);
			velocityCommand.angular.z = control(90, yaw*conv, 0.01);
			if (within(y, 2, 2)) {
				velocityCommand.angular.z = 0;
				velocityCommand.linear.x = 0;
				currentSTATE = loop;
			}
			break;
		case(loop):
			float checks[4] = {x, y, x, y};
		//	ROS_INFO("Count[%d]", count);
			if (count < 4) {
				switch(currentMovement) {
					case(ANGLE):
						//ROS_INFO("ANGULAR MOTION");
						velocityCommand.angular.z = control(loopYaw[count], yaw*conv, 0.01);
						velocityCommand.linear.x = 0;
						if (within(yaw*conv, loopYaw[count], 2)) {
							//ROS_INFO("ANGLE DONE");
							velocityCommand.angular.z = 0;
							currentMovement = LINEAR;
						}
						break;

					case(LINEAR):
						//ROS_INFO("LINEAR MOTION");c file structure
						velocityCommand.angular.z = control(loopYaw[count], yaw*conv, 0.01);
						velocityCommand.linear.x = control(loopLinear[count], checks[count], 0.5);
						if (within(checks[count], loopLinear[count], 2)) {
							//ROS_INFO("LINEAR DONE");
							velocityCommand.angular.z = 0;
							velocityCommand.linear.x = 0;
							currentMovement = ANGLE;
							count++;
						}
						break;
				}
			}


			break;
	}
}*/


int main (int argc, char **argv)
{
	ros::init(argc, argv, "movement");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
	ros::Subscriber sub = my_handle.subscribe("/odom", 1000, positions);
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);

	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	//ros::Subscriber shapeMSG = my_handle.subscribe("/message", 1, shapeProcess);

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
