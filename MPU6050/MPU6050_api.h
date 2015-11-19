#ifndef MPU6050_API_H
#define MPU6050_API_H
#include "MPU6050.h"
#include "AHRSLib.h"
#include "Sensors.h"
#include "Timer_Ctrl.h"

extern float GyroScale[];
extern float AccScale[];
extern float GyroOffset[];
extern float AccOffset[];
extern int16_t rawGYRO[];
extern int16_t rawACC[];

extern SensorInit_T SensorInitState;
extern SensorInit_T SensorCalState;
extern Sensor_T Sensor;
//volatile uint16_t AcceX, AcceY,AcceZ;
extern volatile uint16_t GyroX,GyroY,GyroZ;

void Init_AHRS(void);
void SensorsDynamicCalibrate(char SensorType);
void GyroCalibrate(void);

void AccCalibrationZ(void);


#endif
