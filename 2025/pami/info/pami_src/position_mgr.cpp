/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "led.h"
#include "motor.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define PID_DISTANCE_DEBUG      false
#define PID_ORIENTATION_DEBUG   false
#define POSITION_MGR_DEBUG      false

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  MVT_TYPE_NONE = 0u,         /* No profile selected */
  MVT_TYPE_DISTANCE = 1u,     /* Ramp of triangular profil */
  MVT_TYPE_ORIENTATION = 2u,  /* Ramp of trapezoïdal profil */
} PositionManagerMvtTypeEn;   /* Enumeration used to select the mvt type */

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
//uint8_t positionMgrStatus_u8_g;
int32_t startDistance_i32_g;
int32_t startOrientation_i32_g;
PositionManagerMvtTypeEn positionMgrMvtType_en_g;
PositionManagerStateEn positionMgrState_en_g;

PidControllerSt pidDistance_st_g;
PidControllerSt pidOrientation_st_g;

RampParametersSt rampDistance_st_g;
RampParametersSt rampOrientation_st_g;

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/

/**
   @brief     This function inits the position manager module.


   @param     none

   @result    none

*/
void PositionMgrInit()
{
  positionMgrMvtType_en_g = MVT_TYPE_NONE;
  //positionMgrStatus_u8_g = 1;
  positionMgrState_en_g = POSITION_STATE_NONE;

  /* init pid submodule */
  PidInit(&pidDistance_st_g);
  PidInit(&pidOrientation_st_g);

  PidSetDeltaTime(&pidDistance_st_g, DELTA_TIME_S);
  PidSetDeltaTime(&pidOrientation_st_g, DELTA_TIME_S);

  PidSetCoefficients(&pidDistance_st_g, KP_DISTANCE, KI_DISTANCE, KD_DISTANCE);
  PidSetCoefficients(&pidOrientation_st_g, KP_ORIENTATION, KI_ORIENTATION, KD_ORIENTATION);

  PidStart(&pidDistance_st_g);
  PidStart(&pidOrientation_st_g);

  /* init ramp submodule */
  RampInit(&rampDistance_st_g);
  RampInit(&rampOrientation_st_g);
}

/**
   @brief     This function updates the position manager module, each delta time
                  odometry
                  ramps
                  pids


   @param     none

   @result    none

*/
void PositionMgrUpdate()
{
  uint32_t currentTime_u32 = 0;
  static bool emergencyActivated_b = false;

  currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution

  /* Manages the update loop every pidGetDeltaTime() */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (DELTA_TIME_S * 1000.0) )
  {
    /* issue a warning if more than a 50% increase in loop time */
    //    if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (PidGetDeltaTime() * 1000.0 * 1.5) )
    //    {
    //      Serial.println("Position Manager Overtime");
    //    }

    uint32_t durationMeasureStart_u32 = 0;
    uint32_t durationMeasure_u32 = 0;

    double consigneDistance_d = 0.0;
    double consigneOrientation_d = 0.0;

    if (DEBUG_TIME)
      durationMeasureStart_u32 = micros();

    ObstacleSensorUpdate(DEBUG_TIME);

    if ( (ObstacleSensorDetected() == true) && (emergencyActivated_b == false) )
    {
      emergencyActivated_b = true;
      RampEmergencyStop(&rampDistance_st_g);
      RampEmergencyStop(&rampOrientation_st_g);
      LedSetAnim(LED3_ID, ANIM_STATE_BLINK);
      LedSetBlinkNb(LED3_ID, 2);
    }

    OdometryUpdate(DEBUG_TIME);

    /* Depending on the mouvement type, computes the correct reference signals */
    switch (positionMgrMvtType_en_g)
    {
      case MVT_TYPE_DISTANCE:
        /* Should keep the orientation, and ramps in distance */
        RampUpdate(&rampDistance_st_g, currentTime_u32 - lastExecutionTime_u32, DEBUG_TIME);
        consigneDistance_d = startDistance_i32_g + RampGetDistance(&rampDistance_st_g);
        consigneOrientation_d = startOrientation_i32_g;
        break;

      case MVT_TYPE_ORIENTATION:
        /* Should keep the distance and ramps in orientation */
        RampUpdate(&rampOrientation_st_g, currentTime_u32 - lastExecutionTime_u32, DEBUG_TIME);
        consigneDistance_d = startDistance_i32_g;
        consigneOrientation_d = startOrientation_i32_g + RampGetDistance(&rampOrientation_st_g);
        break;

      default:
        consigneDistance_d = 0.0;
        consigneOrientation_d = 0.0;
        break;
    }
    Serial.println(1.2 * TopToMeter((double)(RampGetDistanceBrake(&rampDistance_st_g)) * 1000.0) );
    ObstacleSensorSetThreshold( (uint16_t)(1.2 * TopToMeter(RampGetDistanceBrake(&rampDistance_st_g)) * 1000) );

    if ( ((RampGetState(&rampDistance_st_g) == RAMP_STATE_FINISHED) || (RampGetState(&rampDistance_st_g) == RAMP_STATE_INIT)) && ((RampGetState(&rampOrientation_st_g) == RAMP_STATE_FINISHED) || (RampGetState(&rampOrientation_st_g) == RAMP_STATE_INIT)) )
    {
      if (emergencyActivated_b == false )
        positionMgrState_en_g = POSITION_STATE_STOPPED;
      else
        positionMgrState_en_g = POSITION_STATE_EMERGENCY;
    }
    else
    {
      positionMgrState_en_g = POSITION_STATE_MOVING;
    }

    /* Sets the new reference on the pids */
    PidSetReference(&pidDistance_st_g, consigneDistance_d);
    PidSetReference(&pidOrientation_st_g, consigneOrientation_d);

    /* Gets the current distance and orientation of the robot */
    double mesureDistance_d = OdometryGetDistanceTop();
    double mesureOrientation_d = OdometryGetOrientationTop();

    /* Sets the current distance and orientation in the pids, they compute the outputs for the motors */
    double commandeDistance_d = PidUpdate(&pidDistance_st_g, mesureDistance_d, DEBUG_TIME);
    double commandeOrientation_d = PidUpdate(&pidOrientation_st_g, mesureOrientation_d, DEBUG_TIME);

    /* Sends the pids outputs to the motors */
    MotorLeftSetSpeed(commandeDistance_d - commandeOrientation_d);
    MotorRightSetSpeed(commandeDistance_d + commandeOrientation_d);

    /* Saves last execution time, for precise loop control */
    lastExecutionTime_u32 = currentTime_u32;

    if (PID_DISTANCE_DEBUG)
    {
      //Serial.print("PidDistance : ");
      Serial.print(pidDistance_st_g.reference_d);
      Serial.print(", ");
      Serial.print(mesureDistance_d);
      Serial.print(", ");
      Serial.print(pidDistance_st_g.error_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.previousError_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.kp_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.ki_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.kd_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.integral_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.derivative_d);
      Serial.print(", ");
      Serial.print(pidDistance_st_g.output_d);
      Serial.println();
    }

    if (PID_ORIENTATION_DEBUG)
    {
      //Serial.print("PidOrientation : ");
      Serial.print(pidOrientation_st_g.reference_d);
      Serial.print(", ");
      Serial.print(mesureOrientation_d);
      Serial.print(", ");
      Serial.print(pidOrientation_st_g.error_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.previousError_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.kp_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.ki_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.kd_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.integral_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.derivative_d);
      Serial.print(", ");
      Serial.print(pidOrientation_st_g.output_d);
      Serial.println();
    }

    if (POSITION_MGR_DEBUG)
    {
      Serial.print("Time [ms] : ");
      Serial.print(currentTime_u32);
      Serial.print(", X [m] : ");
      Serial.print(OdometryGetXMeter());
      Serial.print(", Y {m] : ");
      Serial.print(OdometryGetYMeter());
      Serial.print(", theta [rad] : ");
      Serial.print(OdometryGetThetaRad());
      Serial.print(", ConsDistance [top] : ");
      Serial.print(consigneDistance_d);
      Serial.print(", distance [top] : ");
      Serial.print(mesureDistance_d);
      Serial.print(", ConsOrientation [top] : ");
      Serial.print(consigneOrientation_d);
      Serial.print(", orientation [top] : ");
      Serial.print(mesureOrientation_d);
      Serial.print(", commande Distance : ");
      Serial.print(commandeDistance_d);
      Serial.print(", commande Orientation : ");
      Serial.print(commandeOrientation_d);
      Serial.print(", commande gauche : ");
      Serial.print(commandeDistance_d - commandeOrientation_d);
      Serial.print(", commande droite : ");
      Serial.print(commandeDistance_d + commandeOrientation_d);
      Serial.println();
    }

    if (DEBUG_TIME)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Position loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us");
      Serial.println();
    }
  }
}

/**
   @brief     This function computes a distance and an orientation to reach the specified target


   @param     x_m         : target x coordinate in meter
              y_m         : target y coordinate in meter
              theta_deg   : target theta corrdinate in degrees

*/
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg)
{
  positionMgrState_en_g = POSITION_STATE_MOVING;
}

/**
   @brief     This function computes the mouvement to reach a distance


   @param     distance_m  : distance to go to

   @result    none

*/
void PositionMgrGotoDistanceMeter(double distance_m, bool braking_b)
{
  //positionMgrStatus_u8_g = 0;
  positionMgrState_en_g = POSITION_STATE_MOVING;
  positionMgrMvtType_en_g = MVT_TYPE_DISTANCE;

  startDistance_i32_g = OdometryGetDistanceTop();
  startOrientation_i32_g = OdometryGetOrientationTop();

  /* Test if braking at the end of the ramp is required */
  if (braking_b == true)
    RampNew(&rampDistance_st_g, (int32_t)MeterToTop(distance_m), 0, (int32_t)MeterToTop(VITESSE_SLOW), (int32_t)MeterToTop(ACCELERATION_SLOW));
  else
    RampNew(&rampDistance_st_g, (int32_t)MeterToTop(distance_m), (int32_t)MeterToTop(VITESSE_SLOW), (int32_t)MeterToTop(VITESSE_SLOW), (int32_t)MeterToTop(ACCELERATION_SLOW));
}

/**
   @brief     This function computes the mouvement to reach an orientation


   @param     theta_deg  : orientation to go to

   @result    none

*/
void PositionMgrGotoOrientationDegree(double theta_deg)
{
  //positionMgrStatus_u8_g = 0;
  positionMgrState_en_g = POSITION_STATE_MOVING;
  positionMgrMvtType_en_g = MVT_TYPE_ORIENTATION;

  startDistance_i32_g = OdometryGetDistanceTop();
  startOrientation_i32_g = OdometryGetOrientationTop();

  RampNew(&rampOrientation_st_g, (int32_t)RadToTop(theta_deg * PI / 180.0), 0, (int32_t)MeterToTop(VITESSE_SLOW), (int32_t)MeterToTop(ACCELERATION_SLOW));
}

/**
   @brief     This function returns the status of the position manager


   @param

   @result    positionMgrStatus_u8_g

*/
PositionManagerStateEn PositionMgrGetState()
{
  //return positionMgrStatus_u8_g;
  return positionMgrState_en_g;
}

/**
   @brief     This function returns the position of the robot


   @param

   @result    TODO position?

*/
void PositionMgrGetPosition()
{

}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
