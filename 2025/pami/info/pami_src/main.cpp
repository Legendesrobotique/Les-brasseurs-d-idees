/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
//#include "customTimer.h"
#include "ihm.h"
#include "led.h"
#include "motor.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"
#include "trajectory_mgr.h"
#include "Wire.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void setup() {
  Serial.begin(SERIAL_SPEED);
  Wire.begin();
  Wire.setClock(400000UL);

  pinMode(SWITCH_START_PIN, INPUT_PULLUP);
  pinMode(SWITCH_MODE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_REED_PIN, INPUT_PULLUP);
  pinMode(SWITCH_GROUND_PIN, INPUT_PULLUP);

  /* Init de tous les modules */
  IhmInit();
  LedInit();
  MatchMgrInit();
  MotorInit();
  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();
  TrajectoryMgrInit();

  /* On attend le bouton le top départ */
  /*while (digitalRead(SWITCH_REED_PIN) == 0)
  {
    LedAnimK2000();
  }
  LedAnimAllOff();
  
  LedAnimStart(); // Blocking 5s before start
  LedAnimAllOff();*/

  //CustomTimerInit();
}

void loop() {
  //MotorTest();
  //OdometryEncoderTest();
  IhmUpdate(false);
  LedUpdate(false);
  MatchMgrUpdate();
  PositionMgrUpdate();
  TrajectoryMgrUpdate(false);
}
