#ifndef odometry_h_
#define odometry_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void OdometryInit();
int32_t OdometryGetXTop();
int32_t OdometryGetYTop();
int32_t OdometryGetDistanceTop();
int32_t OdometryGetOrientationTop();
double OdometryGetXMeter();
double OdometryGetYMeter();
double OdometryGetThetaRad();
void OdometryUpdate(bool timeMeasure_b);
void OdometryEncoderTest();

double MeterToTop(double meter);
double TopToMeter(double top);
double TopToRad(double nTop);
double RadToTop(double radian);

#endif
