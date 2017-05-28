#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>

#define GETMAG

#ifdef GETMAGP
float data1[1000] = {0};
float data2[1000] = {0};
void vMagTask( void *pvParameters ) {
  static typeMagSensor nowdata;
  int datan = 0;
  for (datan = 0; datan < 1000; ++datan) {
	nowdata = ReadMagSensor();
	data1[datan] = nowdata.magX;
	data2[datan] = nowdata.magY;
	SetLeftWheelGivenSpeed(10);
	SetRightWheelGivenSpeed(-10);
	vTaskDelay(50);
	//ControlRobotRotate(15, 5);
  } 
  vTaskDelay(100000);
  asm("NOP");
}

#endif 

#ifdef GETMAG
//get magX and magY
typeMagSensor data[100], nowdata;
float minX, maxX, minY, maxY, magX, magY;
int datan;
void vMagTask( void *pvParameters ) {
  minX = minY = 100000;
  maxX = maxY = -100000;
  halt(0.5);
  ControlRobotRotate(15, 5);
  for (datan = 0; datan < 48; ++ datan) {
    nowdata = ReadMagSensor();
    if (minX > nowdata.magX) minX = nowdata.magX;
    if (minY > nowdata.magY) minY = nowdata.magY;
    if (maxX < nowdata.magX) maxX = nowdata.magX;
    if (maxY < nowdata.magY) maxY = nowdata.magY;
    data[datan] = nowdata;
    ControlRobotRotate(15, 5);
    halt(0.5);
  }
  magX = (minX + maxX) / 2;
  magY = (minY + maxY) / 2;
  vTaskDelay(100000);
  asm("NOP");
}
#endif
#ifdef TESTMAG
// test mag
float robotDir;
void vMagTask( void *pvParameters ) {
  while (1) {
    robotDir = ReadMagSensorAngle2North();
    vTaskDelay(1000);
  }
}
#endif
