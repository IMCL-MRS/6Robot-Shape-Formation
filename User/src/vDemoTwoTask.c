#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "fc.h"
#include "snake.h"
#include <math.h>

extern volatile u8 imReady;
extern u8 activeRb[ROBOTS];

const typeCoordinate posS1 = {1.5,0.28};

const typeCoordinate posA6 = {1.0, 0.30};
const typeCoordinate posA5 = {0.85,0.30};
const typeCoordinate posA4 = {0.70,0.30};
const typeCoordinate posA3 = {0.55,0.30};
const typeCoordinate posA2 = {0.40,0.30};
const typeCoordinate posA1 = {0.20,0.30};

const typeCoordinate posR6 = {0.85,0.58};
const typeCoordinate posR5 = {0.85,0.35};
const typeCoordinate posR4 = {0.85,0.12};
const typeCoordinate posR3 = {0.50,0.58};
const typeCoordinate posR2 = {0.50,0.35};
const typeCoordinate posR1 = {0.50,0.12};

const typeCoordinate posT6 = {0.80,0.54};
const typeCoordinate posT5 = {1.05,0.12};
const typeCoordinate posT4 = {0.80,0.12};
const typeCoordinate posT3 = {0.62,0.32};
const typeCoordinate posT2 = {0.92,0.32};
const typeCoordinate posT1 = {0.53,0.12};

const typeCoordinate posH6 = {0.74,0.69};
const typeCoordinate posH5 = {0.93,0.25};
const typeCoordinate posH4 = {0.68,0.14};
const typeCoordinate posH3 = {0.48,0.55};
const typeCoordinate posH2 = {0.95,0.53};
const typeCoordinate posH1 = {0.45,0.26};

const typeCoordinate posD1 = {0.7,0.1};
const typeCoordinate posD2 = {1.2,0.1};
const typeCoordinate posD3 = {1.2,0.7};
const typeCoordinate posD4 = {0.7,0.7};

//#define DEBUG

void bugTest(){
  gotoLeftDelta();
  rotateFastTo(posA1.x,posA1.y,ANGLESPEED,ROTATE_ACCURATE);
  ControlRobotgo2Position(posA1.x,posA1.y,FASTSPEED,0);
  halt(1000);
}

void point2Point(float x, float y){
    rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
	ControlRobotgo2Position(x,y,FASTSPEED,0);
}

/*robot1 formation control*/
void robot1Step1(){
  imReady = 0;
  vTaskDelay(500);
  //point2Point(posA1.x,posA1.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot1Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	if (isReady(activeRb[1]) && isReady(activeRb[2]) && isReady(activeRb[3]) 
		&& isReady(activeRb[4]) && isReady(activeRb[5])) {
	  imReady = 1;	  
	  break;
	}
   vTaskDelay(500);
  }
#endif
  while(!isReady(activeRb[0])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[0])) {
      beepSing();
      halt(1);
      point2Point(posR1.x,posR1.y);
      rotateToNorthAngle(90,FASTSPEED);
  }
  halt(2);
  return;
}

void robot1Step3(){  
  //3 formation	  
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else	
  while (1) {
	if (isReady2(activeRb[1]) && isReady2(activeRb[2]) && isReady2(activeRb[3])
		&& isReady2(activeRb[4]) && isReady2(activeRb[5])) {
	  imReady = 2;
	  break;
	}
   vTaskDelay(500);
  }
#endif
  while(!isReady2(activeRb[0])){
	vTaskDelay(500);
	imReady = 2;
  }
  
  if (isReady2(activeRb[0])) {
	beepSing();
	halt(1);
	point2Point(posT1.x,posT1.y);
        rotateToNorthAngle(90,FASTSPEED);
  }  
  halt(2);
  return;
}

void robot1Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	if (isReady3(activeRb[1]) && isReady3(activeRb[2]) && isReady3(activeRb[3])
		&& isReady3(activeRb[4]) && isReady3(activeRb[5])) {
	  imReady = 3;
	  break;
	}
	vTaskDelay(500);
  }
#endif
  while(!isReady3(activeRb[0])){
	vTaskDelay(500);
	imReady = 3;
  }
  
  if (isReady3(activeRb[0])) {
    beepSing();
	halt(1);
	point2Point(posH1.x,posH1.y);
        rotateToNorthAngle(90,FASTSPEED);
  }   
  halt(2);
  return;
}

void robot1Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	if (isReady(activeRb[1]) && isReady(activeRb[2]) && isReady(activeRb[3])
		&& isReady(activeRb[4]) && isReady(activeRb[5])) {
	  imReady = 1;
	  break;
	}
   vTaskDelay(500);
  }
#endif
  while(!isReady(activeRb[0])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[0])) {
	beepSing();
	halt(1);
	point2Point(posT1.x,posT1.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  return;
}

void robot1Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	if (isReady2(activeRb[1]) && isReady2(activeRb[2]) && isReady2(activeRb[3])
		&& isReady2(activeRb[4]) && isReady2(activeRb[5])) {
	  imReady = 2;
	  break;
	}
   vTaskDelay(500);
  }
#endif
  while(!isReady2(activeRb[0])){
	vTaskDelay(500);
	imReady = 2;
  }
  
  if (isReady2(activeRb[0])) {
	beepSing();
	halt(1);
	point2Point(posR1.x,posR1.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  return;
}

void robot1Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	if (isReady3(activeRb[1]) && isReady3(activeRb[2]) && isReady3(activeRb[3])
		&& isReady3(activeRb[4]) && isReady3(activeRb[5])) {
	  imReady = 3;
	  break;
	}
   vTaskDelay(500);
  }
#endif
  while(!isReady3(activeRb[0])){
	vTaskDelay(500);
	imReady = 3;
  }
  
  if (isReady3(activeRb[0])) {
	beepSing();
	//sundy
	goFor_Axis();
	point2Point(posA1.x,posA1.y);
	rotateToNorthAngle(90,FASTSPEED);
  }
  return;
}

/*robot2 formation control*/
void robot2Step1(){
  imReady = 0;
  vTaskDelay(500);
//  point2Point(posA2.x,posA2.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot2Step2(){  
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;	
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
	 vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[1])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[1])) {
	point2Point(posR2.x,posR2.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  imReady = 2;
  halt(2);
  return;
}

void robot2Step3(){  
  //3 formation
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[1])){
	vTaskDelay(500);
	imReady = 2;
  }
  
  if (isReady2(activeRb[1])) {
	point2Point(posT2.x,posT2.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 3;
  halt(2);
  return;
}

void robot2Step4(){  
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[1])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[1])) {  
	point2Point(posH2.x,posH2.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 1;
  halt(2);
  return;
}

void robot2Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[1])){
	vTaskDelay(500);
	imReady = 1;
  }
  if (isReady(activeRb[1])) {
	halt(1);
	point2Point(posT2.x,posT2.y);
	rotateToNorthAngle(90,FASTSPEED);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 2;
  halt(2);
  return;
}

void robot2Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[1])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[1])) {
	halt(1);
	point2Point(posR2.x,posR2.y);
	rotateToNorthAngle(90,FASTSPEED);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 3;
  halt(2);
  return;
}

void robot2Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[1])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[1])) {
	halt(1);
	point2Point(posA2.x,posA2.y);
	rotateToNorthAngle(90,FASTSPEED);
  }
      imReady = 0;
  halt(2);
  return;
}

/*robot3 formation control*/
void robot3Step1(){
  imReady = 0;
  vTaskDelay(500);
//  point2Point(posA3.x,posA3.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot3Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[2])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[2])) {
         goForward(100);
	 point2Point(posR3.x,posR3.y);
         rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 2;
  halt(2);
  return;
}

void robot3Step3(){
  //3 formation
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[2])){
	vTaskDelay(500);
	imReady = 2;
  }
  
  if (isReady2(activeRb[2])) {
	point2Point(posT3.x,posT3.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 3;
  halt(2);
  return;
}

void robot3Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[2])){
	vTaskDelay(500);
	imReady = 3;
  }
  
  if (isReady3(activeRb[2])) {
	point2Point(posH3.x,posH3.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 1;
  halt(2);
  return;
}

void robot3Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[2])){
	vTaskDelay(500);
	imReady = 1;
  }
  if (isReady(activeRb[2])) {
	halt(1);
	point2Point(posT3.x,posT3.y);
	rotateToNorthAngle(90,FASTSPEED);
  }
  imReady = 2;
  halt(2);
  return;
}

void robot3Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[2])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[2])) {
	halt(1);
	point2Point(posR3.x,posR3.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 3;
  halt(2);
  return;
}

void robot3Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[2])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[2])) {
	halt(1);    
	point2Point(posA3.x,posA3.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 0;
  halt(2);
  return;
}

/*robot4 formation control*/
void robot4Step1(){
  imReady = 0;
//  point2Point(posA4.x,posA4.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot4Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[3])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[3])) {       
	point2Point(posR4.x,posR4.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 2;
  halt(2);
  return;
}

void robot4Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[3])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[3])) {
	point2Point(posT4.x,posT4.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 3;
  halt(2);
  return;
}

void robot4Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[3])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[3])) {
	point2Point(posH4.x,posH4.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 1;
  halt(2);
  return;
}

void robot4Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[3])){
	vTaskDelay(500);
	imReady = 1;
  }
  if (isReady(activeRb[3])) {
	halt(1);
	point2Point(posT4.x,posT4.y);
	rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 2;
  halt(2);
  return;
}

void robot4Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[3])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[3])) {
	halt(1);
	point2Point(posR4.x,posR4.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 3;
  halt(2);
  return;
}

void robot4Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[3])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[3])) {
	halt(1);
	//sundy
	point2Point(posA4.x,posA4.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  imReady = 0;
  halt(2);
  return;
}

/*robot5 formation control*/
void robot5Step1(){
  imReady = 0;
//  point2Point(posA5.x,posA5.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot5Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[4])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[4])) {
	point2Point(posR5.x,posR5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 2;
  halt(2);
  return;
}

void robot5Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[4])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[4])) {
	point2Point(posT5.x,posT5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 3;
  halt(2);
  return;
}

void robot5Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[4])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[4])) {
	point2Point(posH5.x,posH5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 1;
  halt(2);
  return;
}

void robot5Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[4])){
	vTaskDelay(500);
	imReady = 1;
  }
  if (isReady(activeRb[4])) {
	halt(1);
	point2Point(posT5.x,posT5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 2;
  halt(2);
  return;
}

void robot5Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[4])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[4])) {
	halt(1);
	point2Point(posR5.x,posR5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 3;
  halt(2);
  return;
}

void robot5Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[4])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[4])) {
	halt(1);
	point2Point(posA5.x,posA5.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
    imReady = 0;
  halt(2);
  return;
}


/*robot6 formation control*/
void robot6Step1(){
  imReady = 0;
//  point2Point(posA6.x,posA6.y);
  rotateToNorthAngle(90,FASTSPEED);
  halt(2);
  return;
}

void robot6Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[5])){
	vTaskDelay(500);
	imReady = 1;
  }
  
  if (isReady(activeRb[5])) {
	point2Point(posR6.x,posR6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 2;
  halt(2);
  return;
}

void robot6Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[5])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[5])) {
	point2Point(posT6.x,posT6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 3;
  halt(2);
  return;
}

void robot6Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
   vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[5])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[5])) {
	point2Point(posH6.x,posH6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 1;
  halt(2);
  return;
}

void robot6Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  while (1) {
	imReady = 1;
	if (isReady(activeRb[0])) {
	  imReady = 1;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady(activeRb[5])){
	vTaskDelay(500);
	imReady = 1;
  }
  if (isReady(activeRb[5])) {
	halt(1);
	point2Point(posT6.x,posT6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  
  imReady = 2;
  halt(2);
  return;
}

void robot6Step6(){  
  //5 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  while (1) {
	imReady = 2;
	if (isReady2(activeRb[0])) {
	  imReady = 2;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady2(activeRb[5])){
	vTaskDelay(500);
	imReady = 2;
  }
  if (isReady2(activeRb[5])) {
	halt(1);
	point2Point(posR6.x,posR6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  imReady = 3;
  halt(2);
  return;
}

void robot6Step7(){  
  //5 formation	
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  while (1) {
	imReady = 3;
	if (isReady3(activeRb[0])) {
	  imReady = 3;
	  halt(1);
	  break;
	}
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!isReady3(activeRb[5])){
	vTaskDelay(500);
	imReady = 3;
  }
  if (isReady3(activeRb[5])) {
        goForAxis();
	point2Point(posA6.x,posA6.y);
        rotateToNorthAngle(90,FASTSPEED);
  }
  imReady = 0;
  halt(2);
  return;
}

	
void robot1()
{
  robot1Step1();
  robot1Step2();
  halt(2);
  robot1Step3();
  halt(2);
  robot1Step4();
  halt(2);
  robot1Step5();
  halt(2);
  robot1Step6();
  halt(2);
  robot1Step7();
}

void robot2()
{
  robot2Step1();
  robot2Step2();
  halt(2);
  robot2Step3();
  halt(2);
  robot2Step4();
  halt(2);
  robot2Step5();
  halt(2);
  robot2Step6();
  halt(2);
  robot2Step7();
}

void robot3()
{
  robot3Step1();
  robot3Step2();
  halt(2);
  robot3Step3();
  halt(2);
  robot3Step4();
  halt(2);
  robot3Step5();
  halt(2);
  robot3Step6();
  halt(2);
  robot3Step7();
}

void robot4()
{
  robot4Step1();
  robot4Step2();
  halt(2);
  robot4Step3();
  halt(2);
  robot4Step4();
  halt(2);
  robot4Step5();
  halt(2);
  robot4Step6();
  halt(2);
  robot4Step7();
}


void robot5()
{
  robot5Step1();
  robot5Step2();
  halt(2);
  robot5Step3();
  halt(2);
  robot5Step4();
    halt(2);
  robot5Step5();
    halt(2);
  robot5Step6();
    halt(2);
  robot5Step7();
}


void robot6()
{
  robot6Step1();
  robot6Step2();
  halt(2);
  robot6Step3();
  halt(2);
  robot6Step4();
  halt(2);
  robot6Step5();
  halt(2);
  robot6Step6();
  halt(2);
  robot6Step7();
}


void robotsWait(){
  if(rbID == activeRb[0]){
	imReady = 0;
	halt(0.5);
  }else if(rbID == activeRb[1]){
	imReady = 0;
	halt(5);
  }else if(rbID == activeRb[2]){
	imReady = 0;
	halt(7);
  }else if(rbID == activeRb[3]){
	imReady = 0;
	halt(9);
  }else if(rbID == activeRb[4]){
	imReady = 0;
	halt(11);
  }else if(rbID == activeRb[5]){
	imReady = 0;
	halt(13);
  }
}

void shapeForm(){
  if(rbID == activeRb[0]){
	robot1();
  }else if(rbID == activeRb[1]){
	robot2();
  }else if(rbID == activeRb[2]){
	robot3();
  }else if(rbID == activeRb[3]){
	robot4();
  }else if(rbID == activeRb[4]){
	robot5();
  }else if(rbID == activeRb[5]){
	robot6();
  }
}

void standLine(){
  if(rbID == activeRb[0]){
	goFastStraight(posA1.x,posA1.y);
  }else if(rbID == activeRb[1]){
	goFastStraight(posA2.x,posA2.y);
  }else if(rbID == activeRb[2]){
	goFastStraight(posA3.x,posA3.y);
  }else if(rbID == activeRb[3]){
	goFastStraight(posA4.x,posA4.y);
  }else if(rbID == activeRb[4]){
	goFastStraight(posA5.x,posA5.y);
  }else if(rbID == activeRb[5]){
	goFastStraight(posA6.x,posA6.y);
  }
}

void getDataSample(){
  goFastStraight(posD4.x,posD4.y);
  goFastStraight(posD3.x,posD3.y);
  goFastStraight(posD2.x,posD2.y);
  goFastStraight(posD1.x,posD1.y); 
}

void formationControl(){  
  snakeForm(posS1.x, posS1.y);  
  robotsWait();  
  //snakeStart(posS1.x,posS1.y);  
  standLine();
  while(1){
	shapeForm();
	sychronize();
  }  
}

