#ifndef __AHRS_H__
#define __AHRS_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f10x.h"   

void ahrs_put_mpu6050(s16 * data);
void ahrs_update(void);
extern struct ahrs_t
{
	double acc_x;
	double acc_y;
	double acc_z;
	double gryo_pitch;
	double gryo_roll;
	double gryo_yaw;
	double degree_pitch;
	double degree_roll;
	double degree_yaw;
	double time_span;
}ahrs;

#endif
