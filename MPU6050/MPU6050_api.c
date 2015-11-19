
#include "MPU6050_api.h"
#include "config.h"
float GyroScale[3];
float AccScale[3];
float GyroOffset[3];
float AccOffset[3];
int16_t rawGYRO[3];
int16_t rawACC[3];

SensorInit_T SensorInitState = {true,true,false};
SensorInit_T SensorCalState  = {false,false,false};
Sensor_T Sensor;
//volatile uint16_t AcceX, AcceY,AcceZ;
volatile uint16_t GyroX,GyroY,GyroZ;

void Init_AHRS(void)
{
#ifdef SYSTICK
		TIMER_Init();	
		setup_system_tick(1000);
#else
TIMER_Open( TIMER0, TIMER_PERIODIC_MODE, 1000);
TIMER_Start(TIMER0);
TIMER_EnableInt(TIMER0);
NVIC_EnableIRQ(TMR0_IRQn);
ChronographStart(ChronMain);
#endif
	nvtAHRSInit();

	//SensorsInit();
	AccOffset[0] = 0;//0;
	AccOffset[1] = 0;//0;
	AccOffset[2] = 0;//0;
	AccScale[0] = IMU_G_PER_LSB_CFG;
	AccScale[1] = IMU_G_PER_LSB_CFG;
	AccScale[2] = IMU_G_PER_LSB_CFG;
	nvtSetAccScale(AccScale);
	nvtSetAccOffset(AccOffset);
	nvtSetAccG_PER_LSB(IMU_G_PER_LSB_CFG);
	
	GyroOffset[0] = 0;
	GyroOffset[1] = 0;
	GyroOffset[2] = 0;
	GyroScale[0] = IMU_DEG_PER_LSB_CFG;
	GyroScale[1] = IMU_DEG_PER_LSB_CFG;
	GyroScale[2] = IMU_DEG_PER_LSB_CFG;
	nvtSetGyroScale(GyroScale);
	nvtSetGyroOffset(GyroOffset);
	nvtSetGYRODegPLSB(IMU_DEG_PER_LSB_CFG);		
}

void SensorsDynamicCalibrate(char SensorType)
{
	if(SensorType&SENSOR_GYRO&&SensorInitState.GYRO_Done) 
	{
		if(!SensorCalState.GYRO_Done) 
		{
			DelayMsec(1);
			if(nvtGyroCenterCalibrate()==STATUS_GYRO_CAL_DONE) 
			{
				float GyroMean[3];
				SensorCalState.GYRO_Done = true;
				nvtGetGyroOffset(GyroMean);
			}
		}
	}
}

	//Calibrate Gyro
void GyroCalibrate(void)
{
	while (!SensorCalState.GYRO_Done)
	{	
			DelayMsec(1);
			rawGYRO[0] = Read_MPU6050_GyroX();
			rawGYRO[1] = Read_MPU6050_GyroY();
			rawGYRO[2] = Read_MPU6050_GyroZ();
			nvtInputSensorRawGYRO(rawGYRO);
			SensorsDynamicCalibrate (SENSOR_GYRO);
	}
}

void AccCalibrationZ(void)
{
	signed char status;

	nvtCalACCInit();
	do 
	{
		DelayMsec(1);
		rawACC[0] = Read_MPU6050_AccX();
		rawACC[1] = Read_MPU6050_AccY();
		rawACC[2] = Read_MPU6050_AccZ();
		nvtInputSensorRawACC(rawACC);
		status = nvtCalACCBufferFill(0);
	}while(status==STATUS_BUFFER_NOT_FILLED);
}

