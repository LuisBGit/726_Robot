#include "movement.h"

void systemFSM() {
	ROS_INFO("currentSTATE [%d]", currentSTATE);
	switch(currentSTATE) {
		case(findWall):
      currentSTATE = findFirstWall();
      break;
		case(findCorner):
      //currentSTATE = findFirstCorner();
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
  velocityCommand.angular.z = 0.5;
	return findWall;
}


STATE idealLoop() {

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
