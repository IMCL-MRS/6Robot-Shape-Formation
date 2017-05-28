#include "vLocationTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>

typeCoordinate location;
float _Y2NorthAngle = 0;
void vLocationTask( void *pvParameters ) {
  halt(1);
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  while(1) {
	location = GetCoordinate();
	_Y2NorthAngle = ReadMagSensorAngle2North();
//	_Y2NorthAngle = CalibrateNorth2_Y();
	vTaskDelay(500);
  }
}

