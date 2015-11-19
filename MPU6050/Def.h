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
#ifndef DEF_H_
#define DEF_H_
//#include "Config.h"
#include <stdbool.h>
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif
#ifndef bool
#define bool unsigned char
#endif
/****************************************************************/
/*                    Sensor Type definitions                   */
/****************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(NUNCHUCK)
#define STACK_ACC 1
#ifdef MPU6050
//#define ACC_ORIENTATION(X, Y, Z)  {Sensor.rawACC[0] = X; Sensor.rawACC[1] = Y; Sensor.rawACC[2] = Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {Sensor.rawGYRO[0] = X; Sensor.rawGYRO[1] = Y; Sensor.rawGYRO[2] = Z;}
#define ACC_ORIENTATION(X, Y, Z)  {Sensor.rawACC[0] = -X; Sensor.rawACC[1] = -Y; Sensor.rawACC[2] = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {Sensor.rawGYRO[0] = -X; Sensor.rawGYRO[1] = -Y; Sensor.rawGYRO[2] = Z;}
#endif
#else
#define STACK_ACC 0
#define ACC_ORIENTATION(X, Y, Z) 
#define GYRO_ORIENTATION(X, Y, Z)
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110) || defined(AK8975)
#define STACK_MAG 1
#ifdef HMC5883
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = X; Sensor.rawMAG[1] = Y; Sensor.rawMAG[2] = Z;}
#endif
#ifdef AK8975
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = Y; Sensor.rawMAG[1] = X; Sensor.rawMAG[2] = Z;}
#endif
#else
#define STACK_MAG 0
#define MAG_ORIENTATION(X, Y, Z)  {Sensor.rawMAG[0] = X; Sensor.rawMAG[1] = Y; Sensor.rawMAG[2] = Z;}
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(WMP)
#define STACK_GYRO 1
#else
#define STACK_GYRO 0
#endif

#define SYSTEM_CLOCK 72000000

#ifdef USE_I2C_PORT1
#ifdef M451
#define I2C_PORT I2C1
#else
#define I2C_PORT I2C_PORT1
#endif
#else
#ifdef M451
#define I2C_PORT I2C0
#else
#define I2C_PORT I2C_PORT0
#endif
#endif
#ifdef M451
#define IMU_UPDATE_FREQ 560
#define DATA_FLASH_TEST_BASE 0x20000
#else
#define IMU_UPDATE_FRE 300
#endif

#endif //DEF_H_
