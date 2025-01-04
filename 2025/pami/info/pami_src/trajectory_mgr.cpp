/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "led.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "position_mgr.h"
#include "trajectory_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define TRAJECTORY_DEBUG            false
#define TRAJECTORY_UPDATE_PERIOD_S  0.1

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

/**
   @brief     This function inits the trajectory manager module.


   @param     none

   @result    none

*/
void TrajectoryMgrInit()
{

}

/**
   @brief     This function updates the trajectory manager module.


   @param     none

   @result    none

*/
void TrajectoryMgrUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (TRAJECTORY_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual code */
    switch (MatchMgrGetState())
    {
      case MATCH_STATE_BORDER_ADJUST:
        /* Should adjust to border */
        TrajectoryMgrCalibTrajectory();
        break;

      case MATCH_STATE_ON_MOVING:
        /* Should do the main trajectory */
        TrajectoryMgrMainTrajectory();
        break;

      case MATCH_STATE_ON_WAITING:
        /* Should stay on its position? */
        break;

      default:
        break;
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Trajectory loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
      Serial.println();
    }
  }
}

void TrajectoryMgrCalibTrajectory()
{
  static uint8_t trajectoryIndex_u8 = 0;

  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      /* No state */
      //Serial.println("No state");
      break;
    case POSITION_STATE_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;
    case POSITION_STATE_STOPPED:
      /* Next move */
      //Serial.println("Next move");
      TrajectoryCalibrateBorder(trajectoryIndex_u8);
      trajectoryIndex_u8 ++;
      break;
    case POSITION_STATE_EMERGENCY:
      /* What to do ?*/
      //Serial.println("Emergency");
      break;
    default:
      break;
  }
}

void TrajectoryMgrMainTrajectory()
{
  static uint8_t trajectoryIndex_u8 = 0;

  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      /* No state */
      //Serial.println("No state");
      break;
    case POSITION_STATE_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;
    case POSITION_STATE_STOPPED:
      /* Next move */
      //Serial.println("Next move");
      TrajectoryCalibrateSquare(trajectoryIndex_u8, 1.0, true);
      trajectoryIndex_u8 ++;
      break;
    case POSITION_STATE_EMERGENCY:
      /* What to do ?*/
      //Serial.println("Emergency");
      break;
    default:
      break;
  }
}

/**
   @brief     This function makes the robot do a calibration square.


   @param     trajectoryIndex_u8    Index used by the manager to determine which part of the trajectory it is on.

              squareSizeM_d         Size in meters of the sides of the square.

              direction_b           Direction (true cw, false, ccw).

   @result    none

*/
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d, bool direction_b)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;

  double angleDeg_d = 0;
  if (direction_b == true)
    angleDeg_d = 90.0;
  else
    angleDeg_d = -90.0;

  if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
  {
    //Serial.print("Index : ");
    //Serial.print(trajectoryIndex_u8);

    switch (trajectoryIndex_u8)
    {
      case 0:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 1:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 2:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 3:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 4:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 5:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 6:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 7:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 8:
        // trajectory over
        LedAnimAllOff();
        LedSetAnim(LED3_ID, ANIM_STATE_BREATH);
        trajectoryFinished_b = true;
      default:
        break;
    }
    trajectoryIndexLast_i8 = trajectoryIndex_u8;
  }
}

/**
   @brief     This function makes the robot do a calibration on the borders.


   @param     none

   @result    none

*/
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;


  if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
  {
    ObstacleSensorStop();
    //Serial.print("Index : ");
    //Serial.print(trajectoryIndex_u8);

    switch (trajectoryIndex_u8)
    {
      case 0:
        /* Move backwards until border, with no pids */
        PositionMgrSetOrientationControl(false);
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;
      case 1:
        /* Reset the x coordinate, and the theta orientation */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMeter(0.032);
          OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetXMeter(3.0 - 0.032);
          OdometrySetThetaDeg(180.0);
        }
        /* Move forward X cm, X should be greater than the half width of the robot */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_X, true);
        break;

      case 2:
        /* Rotate Ccw or Cw ? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(-90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(90.0);
        }
        break;

      case 3:
        /* Move backwards until border */
        PositionMgrSetOrientationControl(false);
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;

      case 4:
        /* Reset the y coordinate */
        OdometrySetYMeter(2.0 - 0.032);
        OdometrySetThetaDeg(-90.0);
        /* Move forward 0.075m */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_Y, true);
        break;

      case 5:
        /* Rotate Ccw? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA + 90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA - 90.0);
        }
        trajectoryFinished_b = true;
        ObstacleSensorStart();
        MatchMgrSetState(MATCH_SATE_READY);
        break;

      default:
        break;
    }
  }
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
