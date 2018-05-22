#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <std_msgs/Int8.h>
#include "sensor_msgs/LaserScan.h"
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

float idealPathX[12] = {0, 9, 9, 0, 1, 8, 8, 1, 2, 7, 7, 2};
float idealPathY[12] = {2, 2, -2, -2, 1, 1, -1, -1, 0.5, 0.5, -0.5, -0.5};

static void toEulerAngle(float qx, float qy, float qz, float qw, float& yaw)
{
	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);

  yaw = atan2(siny, cosy) * conv;

  if (yaw < 0) {
    yaw += 360;
  }
}

bool within(float value, float compare, float percent, bool rotate) {
  float zeroComp = 0;
  if (rotate) {
    zeroComp = 3;
  } else {
    zeroComp = 0.1;
  }
  ROS_INFO("Value [%f], Compare [%f], Percent [%f]", value, compare, percent);
  if (compare == 0) {
    ROS_INFO("ZERO");
    if (value >= -zeroComp && value <= zeroComp) {
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

float control(float input, float feedBack, float kp) {
  ROS_INFO("error [%f] ", input - feedBack);
	float output = kp * (input - feedBack);
	return fabs(output);

}

bool moveTo(float x2, float y2) {
  float magnitude = fabs(sqrt((x2*x2) + (y2*y2)) - sqrt((x*x) + (y*y)));
  float angle = atan2(y2 - y,x2 - x)*conv;
  ROS_INFO("Desired Angle [%f]", angle);

  if (rotate == true) {
    if (!within(yaw, angle, 2, true)) {
      velocityCommand.linear.x = 0;
      velocityCommand.angular.z = control(angle, yaw, 0.01);
      return false;
    } else {
      velocityCommand.linear.x = 0;
      velocityCommand.angular.z = 0;
      rotate = false;
      return false;
    }
  } else {
    if (!within(magnitude,0,10, false)) {
      velocityCommand.linear.x = control(0, magnitude, 0.5);
      velocityCommand.angular.z = 0;
      return false;
    } else {
      velocityCommand.linear.x = 0;
      velocityCommand.angular.z = 0;
      rotate = true;
      return true;
    }
  }


}




void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

	int clearCheck1 = 218;
	int clearCheck2 = 293;
	int startPoint = 0;
	int endPoint = 0;


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
  ROS_INFO("YAW [%f]", yaw);
}

void loop() {
  ROS_INFO("Moving to %f , %f", idealPathX[count], idealPathY[count]);
  if (moveTo(idealPathX[count], idealPathY[count])) {
    count++;
  }

  if (count > 1) {
    count = 0;
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
