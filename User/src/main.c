#include "stm32f10x.h"
#include "halDriver.h"
#include "vTasks.h"
#include "apps.h"
#include "FreeRTOS.h"
#include "task.h"

#define SNAKE //LOCATION //MAG //EXAMPLE
int main(void) {
  //////////////////////////////////////////////////////////////////////////////
  //1. hardware init
  //////////////////////////////////////////////////////////////////////////////
  SystemInit();    //STM32 cpu clock init -> 72MHZ
  halRCCInit();    //STM32 PeriphClock init
  //////////////////////////////////////////////////////////////////////////////
  //2. Periph devices init
  /////////////////////    //a. Led init
  WheelSpeedSensorInit();  //b. motor feedback speed sensor init 
  MotorInit();         /////////////////////////////////////////////////////////
  LedInit();               //c. motor driver init
  UartInit();              //d. uart Init, communicate with CC2530. ***it must be initialized, even uart is not used!!!***
  halMCUWaitMs(100);
  MPUSensorInit();         //e. MPU sensor Init -> gyro accel mag sensor
  RFInit();                //f. RF Init. via UART communicate with CC2530, config RF
  LightSensorInit();       //g. Light Sensor Init
  BeepInit();
  
  //建立任务
  xTaskCreate( vMotorControlTask,    ( signed portCHAR * ) "MOTOR",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+10, NULL );
#ifdef DEMO1
  // Demo1
  xTaskCreate( vDemoOneTask,         ( signed portCHAR * ) "DEMO1",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
#elif defined(DEMO2)
  // Demo2  
  xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vLocalizationTask,    ( signed portCHAR * ) "MPU READ", configMINIMAL_STACK_SIZE*5, NULL, tskIDLE_PRIORITY+4, NULL );  
  xTaskCreate( vDemoTwoTask,         ( signed portCHAR * ) "DEMO2",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
#elif defined(DEMO3)  
  // Demo3
  xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vLocalizationTask,    ( signed portCHAR * ) "MPU READ", configMINIMAL_STACK_SIZE*5, NULL, tskIDLE_PRIORITY+4, NULL );  
  xTaskCreate( vDemoThreeTask,       ( signed portCHAR * ) "DEMO3",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
#elif defined(DEMO4)  
  // Demo4
  xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vDemoFourTask,        ( signed portCHAR * ) "DEMO4",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );    
#elif defined(DISTRIBUTED)  
  xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",         configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vDistributedTask,     ( signed portCHAR * ) "DISTRIBUTED",   configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY,   NULL );      
  //Snake formation
#elif defined(SNAKE)  
  xTaskCreate( vBCastInfoTask,       ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE*16, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vSnakeTask,           ( signed portCHAR * ) "SNAKE",    configMINIMAL_STACK_SIZE*5,  NULL, tskIDLE_PRIORITY+1,   NULL );      
  //  xTaskCreate( vStackCheck,          ( signed portCHAR * ) "STACK",    configMINIMAL_STACK_SIZE,  NULL, tskIDLE_PRIORITY,   NULL );      
#elif defined(EXAMPLE)
  xTaskCreate( vExampleTask1,        ( signed portCHAR * ) "Example",    configMINIMAL_STACK_SIZE*16, NULL, tskIDLE_PRIORITY+5, NULL );  
#elif defined(MAG)
  // debug mag
  xTaskCreate( vMagTask,             ( signed portCHAR * ) "MAG",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
#elif defined(LOCATION)
  // debug location
  xTaskCreate( vLocalizationTask,    ( signed portCHAR * ) "MPU READ", configMINIMAL_STACK_SIZE*5, NULL, tskIDLE_PRIORITY+4, NULL );  
  xTaskCreate( vLocationTask,        ( signed portCHAR * ) "LOCATION", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY+3, NULL );
#elif defined(MELD)
  xTaskCreate( vMeldTask,            ( signed portCHAR * ) "MELD",     configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );
#elif defined(MULTIMELD)
  xTaskCreate( interruptMotorTask,    ( signed portCHAR * ) "CONTROL",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( multiRobotVMTask,     ( signed portCHAR * ) "MULTIMELD", configMINIMAL_STACK_SIZE*30, NULL, tskIDLE_PRIORITY+3, NULL );
#endif
  
  //启动OS
  vTaskStartScheduler();  
  return 0;
}

