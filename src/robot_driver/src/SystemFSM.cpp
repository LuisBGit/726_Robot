#include "movement.h"

void systemFSM() {
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

}
