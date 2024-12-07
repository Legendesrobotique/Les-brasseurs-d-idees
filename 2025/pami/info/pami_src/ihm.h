#ifndef ihm_h_
#define ihm_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define IHM_DISPLAY_SCREEN_NUMBER   5u

#define LOGO_HEIGHT   32
#define LOGO_WIDTH    32
static const unsigned char PROGMEM logo_bmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF0, 0x00, 0x00, 0x31, 0x0C, 0x00, 0x00, 0xC2, 0x83, 0x00,
  0x01, 0x01, 0x00, 0x80, 0x02, 0x00, 0x80, 0x40, 0x04, 0x01, 0xC0, 0x20, 0x08, 0x02, 0x20, 0x10,
  0x10, 0x1F, 0xF8, 0x08, 0x10, 0x20, 0x04, 0x08, 0x20, 0x40, 0x02, 0x04, 0x20, 0x40, 0x02, 0x04,
  0x40, 0x44, 0x22, 0x02, 0x40, 0xCA, 0x53, 0x02, 0x41, 0x4A, 0x52, 0x82, 0x42, 0x4A, 0x52, 0x42,
  0x56, 0x4E, 0x72, 0x6A, 0x4A, 0x4E, 0x72, 0x52, 0x52, 0x40, 0x02, 0x4A, 0x41, 0x40, 0x02, 0x82,
  0x20, 0xCF, 0xF3, 0x04, 0x20, 0x54, 0xAA, 0x04, 0x10, 0x4F, 0xF2, 0x08, 0x10, 0x40, 0x02, 0x08,
  0x08, 0x20, 0x04, 0x10, 0x04, 0x1F, 0xF8, 0x20, 0x02, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x80,
  0x00, 0xC0, 0x03, 0x00, 0x00, 0x38, 0x1C, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00
};
/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  IHM_COLOR_NONE = 0u,    /* No color selected */
  IHM_COLOR_BLUE = 1u,    /* Color blue */
  IHM_COLOR_YELLOW = 2u,  /* Color yellow */
} IhmColorEn; /* Enumeration used to select the color */

typedef enum
{
  IHM_DISPLAY_SCREEN_NONE = 0u,                   /* No color selected */
  IHM_DISPLAY_SCREEN_MATCH = 1u,                  /* Match screen */
  IHM_DISPLAY_SCREEN_SENSOR_DEBUG = 2u,           /* Sensor debug screen */
  IHM_DISPLAY_SCREEN_MOTOR_DEBUG = 3u,            /* Motor debug screen */
  IHM_DISPLAY_SCREEN_CONTROL_DEBUG = 4u,          /* Control debug screen */
  IHM_DISPLAY_SCREEN_LOGO = 5u,                   /* Logo screen */
} IhmDisplayScreenEn; /* Enumeration used to select the screen to display */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void IhmInit();
void IhmUpdate(bool timeMeasure_b);

void IhmSetColor(IhmColorEn color_en);
void IhmDrawScreenMatch();
void IhmDrawScreenSensorDebug();
void IhmDrawScreenMotorDebug();
void IhmDrawScreenControlDebug();
void IhmDrawScreenInit();
void IhmDrawScreenNone();
void IhmMode();
void IhmColor();
void IhmDrawLogo();

#endif
