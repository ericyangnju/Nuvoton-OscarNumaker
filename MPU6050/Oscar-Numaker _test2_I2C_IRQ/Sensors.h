/*================================================================================*
 * O     O          __             ______  __   __  ____     __  ___          __  *
 *  \   /      /\  / /_      _    / /___/ / /  / / / __ \   / / /   \    /\  / /  *
 *   [+]      /  \/ / \\    //   / /____ / /  / /  \ \_    / / | | | |  /  \/ /   *
 *  /   \    / /\  /   \\__//   / /----// /__/ /  \ \__ \ / /  | | | | / /\  /    *
 * O     O  /_/  \/     \__/   /_/      \_ ___/    \___ //_/    \___/ /_/  \/     *
 *                                                                                *
 *                                                                                *
 * Nuvoton Sensor Fusion Application Firmware for Cortex M4 Series                *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#ifndef _SENSORS_H
#define _SENSORS_H
#include "Def.h"
#include <stdint.h>

typedef struct {
	int16_t rawACC[3];
	int16_t rawGYRO[3];
	int16_t rawMAG[3];
}Sensor_T;

typedef struct {
	bool ACC_Done;
	bool GYRO_Done;
	bool MAG_Done;
}SensorInit_T;

#if STACK_ACC
#define ACC_OFFSET_X          36
#define ACC_OFFSET_Y          -322
#define ACC_OFFSET_Z          467
#define ACC_SCALE_X           0.000061f
#define ACC_SCALE_Y           0.000061f
#define ACC_SCALE_Z           0.000061f
#ifdef MPU6050
#define IMU_GYRO_FS_CFG       MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6050_ACCEL_FS_2
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_2
#define IMU_1G_RAW           (int16_t)(1.0 / IMU_G_PER_LSB_CFG)
#endif
#else
#define IMU_GYRO_FS_CFG       MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6050_ACCEL_FS_2
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_2
#define IMU_1G_RAW           (int16_t)(1.0 / IMU_G_PER_LSB_CFG)
#endif

#ifdef HMC5883
#define MAG_GAUSS_PER_LSB_CFG    HMC5883L_GAIN_660
#define MAG_GAUSS_PER_LSB        (1/660.0)
#define MAG_RATIO_FACTOR_CALIBRATION
#endif
#ifdef AK8975
#define MAG_GAUSS_PER_LSB        (1/660.0)
#define MAG_RATIO_FACTOR_CALIBRATION
#endif
#ifndef MAG_GAUSS_PER_LSB
#define MAG_GAUSS_PER_LSB        (1/660.0)
#endif
#define MAG_CAL0 18.594749f
#define MAG_CAL1 -67.389771f
#define MAG_CAL2 -12.142605f
#define MAG_CAL3 0.038077f
#define MAG_CAL4 0.036936f
#define MAG_CAL5 0.040339f
#define MAG_CAL6 0.000455f
#define MAG_CAL7 -0.000022f
#define MAG_CAL8 0.000917f
#define MAG_CAL9 11.632515f
#define MAG_QFactor 3
void SensorsRead(char,char);
void SensorsDynamicCalibrate(char SensorType);
void temperatureRead(float *temperatureOut);
void SensorsInit(void);
char GetSensorInitState(void);
char GetSensorCalState(void);
#endif
