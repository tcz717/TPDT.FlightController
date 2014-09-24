#include "ahrs.h"
#include "stm32f10x.h"                  // Device header
#include "hardtimer.h"
#include "math.h"
struct ahrs_t ahrs;
int16_t mpu_acc_x,mpu_acc_y,mpu_acc_z;
int16_t mpu_gryo_pitch,mpu_gryo_roll,mpu_gryo_yaw;


/*=====================================================================================================*/
/*=====================================================================================================*/
/*=====================================================================================================*/
/*=====================================================================================================*
**: MoveAve_SMA
**: Simple Moving Average
**: NewData, MoveAve_FIFO, SampleNum
**: AveData
**: MoveAve_SMA(NewData, MoveAve_FIFO, SampleNum)
**=====================================================================================================*/
/*=====================================================================================================*/
static s16 MoveAve_SMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)       // ????
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;    // ?????

	for (i = 0; i < SampleNum; i++)           // ??
		MoveAve_Sum += MoveAve_FIFO[i];

	AveData = (s16)(MoveAve_Sum / SampleNum); // ?????

	return AveData;
}
/*=====================================================================================================*/
/*=====================================================================================================*
**: MoveAve_WMA
**: Weighted Moving Average
**: NewData, MoveAve_FIFO, SampleNum
**: AveData
**: MoveAve_WMA(NewData, MoveAve_FIFO, SampleNum)
**=====================================================================================================*/
/*=====================================================================================================*/
static s16 MoveAve_WMA(volatile int16_t NewData, volatile int16_t *MoveAve_FIFO, u8 SampleNum)
{
	u8 i = 0;
	s16 AveData = 0;
	u16 SampleSum = 0;
	s32 MoveAve_Sum = 0;

	for (i = 0; i < SampleNum - 1; i++)         // ????
		MoveAve_FIFO[i] = MoveAve_FIFO[i + 1];

	MoveAve_FIFO[SampleNum - 1] = NewData;      // ?????

	for (i = 0; i < SampleNum; i++)             // ?? & ??
		MoveAve_Sum += MoveAve_FIFO[i] * (i + 1);

	SampleSum = (SampleNum * (SampleNum + 1)) / 2; // ??????
	AveData = (s16)(MoveAve_Sum / SampleSum);   // ?????

	return AveData;
}

void ahrs_update()
{
	double ax,ay;
	const double a = 0.98;
    double dt = Timer4_GetSec();
	const double gyroscale = 1000.0;
	
	ahrs.gryo_pitch		= mpu_gryo_pitch * gyroscale / 32767.0;
	ahrs.gryo_roll		= mpu_gryo_roll * gyroscale / 32767.0;
	ahrs.gryo_yaw		= mpu_gryo_yaw * gyroscale / 32767.0;
	ax					= atan2(
							mpu_acc_y,
							sqrt(mpu_acc_x * mpu_acc_x +mpu_acc_z * mpu_acc_z)
								)* 180.0 / 3.14;
	ay					= -atan2(
							mpu_acc_x,
							sqrt(mpu_acc_y * mpu_acc_y + mpu_acc_z * mpu_acc_z)
								) * 180.0 / 3.14;
	ahrs.degree_pitch	= a * (ahrs.degree_pitch  + ahrs.gryo_pitch * dt) + (1 - a) * ax;
	ahrs.degree_roll	= a * (ahrs.degree_roll + ahrs.gryo_roll * dt) + (1 - a) * ay;
	ahrs.degree_yaw		= ahrs.degree_yaw + ahrs.gryo_yaw * dt;
	ahrs.time_span		= dt;
}


volatile int16_t MPU6050_ACC_FIFO[3][256] = {{0}};
volatile int16_t MPU6050_GYR_FIFO[3][256] = {{0}};
double MPU6050_Diff[6]={0};
void ahrs_put_mpu6050(s16 * data)
{
	mpu_gryo_pitch=(MoveAve_WMA(data[3], MPU6050_GYR_FIFO[0], 8)+MPU6050_Diff[0]);
	mpu_gryo_roll=(MoveAve_WMA(data[4], MPU6050_GYR_FIFO[1], 8)+MPU6050_Diff[1]);
	mpu_gryo_yaw=(MoveAve_WMA(data[5], MPU6050_GYR_FIFO[2], 8)+MPU6050_Diff[2]);
	mpu_acc_x=MoveAve_WMA(data[0], MPU6050_ACC_FIFO[0], 16)+MPU6050_Diff[3];
	mpu_acc_y=MoveAve_WMA(data[1], MPU6050_ACC_FIFO[1], 16)+MPU6050_Diff[4];
	mpu_acc_z=MoveAve_WMA(data[2], MPU6050_ACC_FIFO[2], 16)+MPU6050_Diff[5];
}
