#include "movement.h"

void systemFSM() {
	ROS_INFO("currentSTATE [%d]", currentSTATE);
	switch(currentSTATE) {
		case(findWall):
      currentSTATE = findFirstWall();
      break;
		case(findCorner):
      currentSTATE = findFirstCorner();
			break;
		case(loop):
      currentSTATE = idealLoop();
			break;
    case(dodge):
      currentSTATE = dodgeObstacle();
      break;
	}

}



STATE findFirstWall() {
  velocityCommand.angular.z = control(90, yaw*conv, 0.01);
  if (within(yaw*conv, 90, 2)) {
    velocityCommand.angular.z = 0;
    return findCorner;
  } else {
    return findWall;
  }
}

STATE findFirstCorner() {
  velocityCommand.linear.x = control(2, y, 0.5);
  velocityCommand.angular.z = control(90, yaw*conv, 0.01);
  if (within(y, 2, 2)) {
    velocityCommand.angular.z = 0;
    velocityCommand.linear.x = 0;
		refYaw = yaw;
    return loop;
  } else {
    return findCorner;
  }
}

STATE idealLoop() {
  float checks[4] = {x, y, x, y};
//	ROS_INFO("Count[%d]", count);
  if (count < 4) {
    switch(currentMovement) {
      case(ANGLE):
        //ROS_INFO("ANGULAR MOTION");
        velocityCommand.angular.z = control(loopYaw[count % 5], yaw*conv, 0.01);
        velocityCommand.linear.x = 0;
        if (within(yaw*conv, loopYaw[count % 5], 3)) {
          //ROS_INFO("ANGLE DONE");
          velocityCommand.angular.z = 0;
          currentMovement = LINEAR;
        }
        break;

      case(LINEAR):
        //ROS_INFO("LINEAR MOTION");c file structure
        velocityCommand.angular.z = control(loopYaw[count % 5], yaw*conv, 0.01);
        velocityCommand.linear.x = control(loopLinear[count % 4] , checks[count % 4], 0.6);
        if (within(checks[count % 4], loopLinear[count % 4], 3)) {
          //ROS_INFO("LINEAR DONE");
          velocityCommand.angular.z = 0;
          velocityCommand.linear.x = 0;
          currentMovement = ANGLE;
          count++;
        }
        break;
    }
  }
  return loop;
}


STATE dodgeObstacle() {
  ROS_INFO("Starting Dodge");
  velocityCommand.linear.x = 0;
  velocityCommand.angular.z = 0;
  return dodge;
}

STATE suppressMovement() {
	return suppressed;
}
