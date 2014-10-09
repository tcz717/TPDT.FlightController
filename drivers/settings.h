#ifndef __SETTING_H__
#define __SETTING_H__

#include <board.h>
#include <rtthread.h>
#include <components.h>
#include <dfs_fs.h>
#include "PID.h"

#define SETTING_MAGIC 0x1234

struct setting_t
{
	u16 magic;
	
	double pitch_p;
	double pitch_i;
	double pitch_d;
	
	double roll_p;
	double roll_i;
	double roll_d;
	
	double yaw_p;
	double yaw_i;
	double yaw_d;
	
	double mpu6050_acc_diff[3];
	
	u16 roll_min;
	u16 roll_mid;
	u16 roll_max;
	
	u16 pitch_min;
	u16 pitch_mid;
	u16 pitch_max;
	
	u16 th_min;
	u16 th_max;
	
	u16 yaw_min;
	u16 yaw_mid;
	u16 yaw_max;
};

extern struct setting_t settings;

void get_pid(void);
rt_err_t load_settings(struct setting_t * s,const char * path,PID * pitch,PID * roll);

#endif
