#include <board.h>
#include <rtthread.h>

#include <components.h>

//#include "i2c1.h"
#include "spi2.h"
#include "bmp085.h"
//#include "MPU6050.h"
#include "ahrs.h"  
#include "LED.h"
#include "Motor.h"
#include "hardtimer.h"
#include "PID.h"
#include "settings.h"
#include "stm32_iic.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include <dfs_posix.h> 
#endif

//#define FC_DEBUG
#ifdef FC_DEBUG
#define debug(fmt, ...)   rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#define AHRS_EVENT_MPU6050	1
#define AHRS_EVENT_Update 	(1 << 7)
#define AHRS_EVENT_WRONG 	(1 << 15)

#define LED1(TIME) led_period[0]=(TIME)
#define LED2(TIME) led_period[1]=(TIME)
#define LED3(TIME) led_period[2]=(TIME)
#define LED4(TIME) led_period[3]=(TIME)

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 256 ];
static struct rt_thread led_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t control_stack[ 512 ];
static struct rt_thread control_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t ahrs_stack[ 512 ];
static struct rt_thread ahrs_thread;
static struct rt_event ahrs_event;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t correct_stack[ 512 ];
static struct rt_thread correct_thread;

u8 led_period[4];
void led_thread_entry(void* parameter)
{
	u8 time[4];
	while(1)
	{
		if(led_period[0])
			time[0]=time[0]%led_period[0];
		LED_set1(!time[0]);
		
		if(led_period[0])
			time[1]=time[1]%led_period[1];
		LED_set2(!time[1]);
		
		if(led_period[0])
			time[2]=time[2]%led_period[2];
		LED_set3(!time[2]);
		
		if(led_period[0])
			time[3]=time[3]%led_period[3];
		LED_set4(!time[3]);
		rt_thread_delay(50);
	}
}


u8 balence=0;
u8 pwmcon=0;
PID pitch_pid,roll_pid,yaw_pid;
PID pout_pid,rout_pid;
s16 pitch_ctl[16],roll_ctl[16],yaw_ctl[16];

rt_bool_t lost_ahrs=RT_FALSE;
rt_tick_t last_ahrs;
void control_thread_entry(void* parameter)
{
	u16 throttle=0;
	double pitch=0;
	double roll=0;
	double yaw=0;
	
	pitch_pid.expect=0;
	roll_pid.expect=0;
	yaw_pid.expect=0;
	pout_pid.expect=0;
	rout_pid.expect=0;
	
	rt_kprintf("start control\n");
	
	while(1)
	{
		LED3(balence*2);
		//rt_kprintf("i3:%d	i5:%d	o1:%d\n",PWM3_Time,PWM5_Time,throttle);
		debug("p:%d	%d	r:%d	%d\n",(s16)pitch,(s16)pitch_pid.expect,(s16)roll,(s16)roll_pid.expect);
		if(pwmcon)
		{
			if(PWM3_Time<=settings.th_max&&PWM3_Time>=settings.th_min)
				throttle=(PWM3_Time-settings.th_min)*1000/(settings.th_max-settings.th_min);
			else 
				throttle=0;
			
			if(PWM1_Time<=settings.roll_max&&PWM1_Time>=settings.roll_min)
			{
				roll	=MoveAve_WMA(PWM1_Time,roll_ctl,16)		-	settings.roll_mid;
				if(roll>5)
					PID_SetTarget(&rout_pid,-roll/(double)(settings.roll_max	-	settings.roll_mid)*45.0);
				else if(roll<-5)
					PID_SetTarget(&rout_pid,-roll
											/(double)(settings.roll_mid	-	settings.roll_min)*45.0);
				else
					PID_SetTarget(&rout_pid,0);
			}
			if(PWM2_Time<=settings.pitch_max&&PWM2_Time>=settings.pitch_min)
			{
				pitch	=MoveAve_WMA(PWM2_Time,pitch_ctl,16)	-	settings.pitch_mid;
				if(pitch>5)
					PID_SetTarget(&pout_pid,-pitch
											/(double)(settings.pitch_max	-	settings.pitch_mid)*30.0);
				else if(pitch<-5)
					PID_SetTarget(&pout_pid,-pitch
										/(double)(settings.pitch_mid	-	settings.pitch_min)*30.0);
				else 
					PID_SetTarget(&pout_pid,0);
			}	
			if(PWM4_Time<=settings.yaw_max&&PWM4_Time>=settings.yaw_min)
			{
				yaw	=MoveAve_WMA(PWM4_Time,yaw_ctl,16)	-	settings.yaw_mid;
				if(yaw>5)
					yaw=-yaw/(double)(settings.yaw_max	-	settings.yaw_mid)*300.0;
				else if(yaw<-5)
					yaw=-yaw/(double)(settings.yaw_mid	-	settings.yaw_min)*300.0;
				else
					yaw=0;
			}	
			if(!balence)
				Motor_Set(throttle,throttle,throttle,throttle);
		}
		else if(PWM3_Time>settings.th_min&&PWM3_Time<settings.th_min+40&&
			PWM5_Time<1500&&PWM5_Time>500)
		{
			//set pwm middle
			if(!pwmcon)
			{
				rt_uint32_t e;
				rt_uint16_t i;
				rt_int16_t * mpu1, * mpu2, * mpu3;
				rt_int16_t m1,m2,m3;
				{
					
					rt_kprintf("start sensors correct\n");
					
					mpu1 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
					mpu2 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
					mpu3 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
					
					for(i=0;i<255;i++)
					{
						if(rt_event_recv(&ahrs_event,AHRS_EVENT_Update,
							RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,
							RT_WAITING_FOREVER,&e)==RT_EOK)
						{
							m1=MoveAve_SMA(mpu_gryo_pitch	, mpu1, 255);
							m2=MoveAve_SMA(mpu_gryo_roll	, mpu2, 255);
							m3=MoveAve_SMA(mpu_gryo_yaw		, mpu3, 255);
						}
					}
					
					MPU6050_Diff[0]-=m1; 
					MPU6050_Diff[1]-=m2;
					MPU6050_Diff[2]-=m3;
					
					rt_free(mpu1);
					rt_free(mpu2);
					rt_free(mpu3);
					
					rt_kprintf("sensor correct finish.\n");
				}
				settings.roll_mid	=PWM1_Time;
				settings.pitch_mid	=PWM2_Time;
				settings.yaw_mid	=PWM4_Time;
			}
			pwmcon=1;
			balence=1;
			pitch_pid.iv=0;
			roll_pid.iv=0;
			yaw_pid.iv=0;
		}
		
		if(balence)
		{
			if(throttle>30&&abs(ahrs.degree_pitch)<30&&abs(ahrs.degree_roll)<30)
			{
				
				PID_xUpdate(&pout_pid	,ahrs.degree_pitch);
				PID_SetTarget(&pitch_pid,-RangeValue(pout_pid.out,-80,80));
				PID_xUpdate(&pitch_pid	,ahrs.gryo_pitch);
				
				PID_xUpdate(&rout_pid	,ahrs.degree_roll);
				PID_SetTarget(&roll_pid,-RangeValue(rout_pid.out,-80,80));
				PID_xUpdate(&roll_pid	,ahrs.gryo_roll);
//				PID_Update(&pitch_pid	,ahrs.degree_pitch	,ahrs.gryo_pitch);
//				PID_Update(&roll_pid	,ahrs.degree_roll	,ahrs.gryo_roll);
				PID_Update(&yaw_pid		,ahrs.degree_yaw	,ahrs.gryo_yaw);
				
				Motor_Set1(throttle - pitch_pid.out - roll_pid.out + yaw_pid.out);
				Motor_Set2(throttle - pitch_pid.out + roll_pid.out - yaw_pid.out);
				Motor_Set3(throttle + pitch_pid.out - roll_pid.out - yaw_pid.out);
				Motor_Set4(throttle + pitch_pid.out + roll_pid.out + yaw_pid.out);
				
				debug( "m1:%d	m2:%d	m3:%d	m4:%d\n",
				(u16)(throttle - pitch_pid.out + roll_pid.out),
				(u16)(throttle - pitch_pid.out - roll_pid.out),
				(u16)(throttle + pitch_pid.out - roll_pid.out),
				(u16)(throttle + pitch_pid.out + roll_pid.out));
			}
			else
				Motor_Set(0,0,0,0);
		}
		
		if(PWM5_Time>1500)
		{
//			settings.roll_min= min(settings.roll_min, PWM1_Time);
//			settings.roll_max= max(settings.roll_max, PWM1_Time);	
//			
//			settings.pitch_min= min(settings.pitch_min, PWM2_Time);
//			settings.pitch_max= max(settings.pitch_max, PWM2_Time);
//			
//			settings.yaw_min= min(settings.yaw_min, PWM4_Time);
//			settings.yaw_max= max(settings.yaw_max, PWM4_Time);
//			if(PWM3_Time<settings.th_min)
//				settings.th_min=PWM3_Time;
//			if(PWM3_Time>settings.th_max)
//				settings.th_max=PWM3_Time;
			Motor_Set(0,0,0,0);
			pitch_pid.iv=0;
			roll_pid.iv=0;
			yaw_pid.iv=0;
			balence=0;
			pwmcon=0;
			LED_set2(1);
		}
		else
			LED_set2(0);
		
		rt_thread_delay(4);
	}
}

void correct_thread_entry(void* parameter)
{
	rt_uint32_t e;
	rt_uint16_t i;
	rt_int16_t * mpu1, * mpu2, * mpu3;
	rt_int16_t m1,m2,m3;
	{
		
		rt_kprintf("start sensors correct\n");
		
		mpu1 = (rt_int16_t *)rt_calloc(200,sizeof(rt_int16_t));
		mpu2 = (rt_int16_t *)rt_calloc(200,sizeof(rt_int16_t));
		mpu3 = (rt_int16_t *)rt_calloc(200,sizeof(rt_int16_t));
		
		for(i=0;i<200;i++)
		{
			if(rt_event_recv(&ahrs_event,AHRS_EVENT_Update,
				RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,
				RT_WAITING_FOREVER,&e)==RT_EOK)
			{
				m1=MoveAve_SMA(mpu_gryo_pitch	, mpu1, 200);
				m2=MoveAve_SMA(mpu_gryo_roll	, mpu2, 200);
				m3=MoveAve_SMA(mpu_gryo_yaw		, mpu3, 200);
			}
		}
		
		MPU6050_Diff[0]=-m1;
		MPU6050_Diff[1]=-m2;
		MPU6050_Diff[2]=-m3;
		
		rt_free(mpu1);
		rt_free(mpu2);
		rt_free(mpu3);
		
		rt_kprintf("sensor correct finish.\n");
	}
	rt_sem_init(&acc_fix_sem,"acc_fix",0,RT_IPC_FLAG_FIFO);
	
	while(1)
	{
		rt_sem_take(&acc_fix_sem,RT_WAITING_FOREVER);
		rt_kprintf("start acc fix\n");
	
		MPU6050_Diff[3]=0;
		MPU6050_Diff[4]=0;
		MPU6050_Diff[5]=0;
		
		mpu1 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
		mpu2 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
		mpu3 = (rt_int16_t *)rt_calloc(255,sizeof(rt_int16_t));
		
		for(i=0;i<255;i++)
		{
			if(rt_event_recv(&ahrs_event,AHRS_EVENT_Update,
				RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,
				RT_WAITING_FOREVER,&e)==RT_EOK)
			{
				m1=MoveAve_SMA(mpu_acc_x	, mpu1, 255);
				m2=MoveAve_SMA(mpu_acc_y	, mpu2, 255);
				m3=MoveAve_SMA(mpu_acc_z	, mpu3, 255);
			}
		}
		
		settings.mpu6050_acc_diffx=MPU6050_Diff[3]=-m1;
		settings.mpu6050_acc_diffy=MPU6050_Diff[4]=-m2;
		settings.mpu6050_acc_diffz=MPU6050_Diff[5]=-m3 + HALF_SIGNED16 / MPU6050_ACC_SCALE;
		
		rt_free(mpu1);
		rt_free(mpu2);
		rt_free(mpu3);
		
		rt_kprintf( "ax:%d	ay:%d	az:%d\n",
				(s16)(MPU6050_Diff[3]),
				(s16)(MPU6050_Diff[4]),
				(s16)(MPU6050_Diff[5]));
		
		rt_kprintf("acc fix finish.\n");
	}
}

u8 en_out_ahrs=0;
short gyro[3], accel[3], sensors;   
static volatile Quaternion curq={1.0,0.0,0.0,0.0};
#define q0 curq.q0
#define q1 curq.q1
#define q2 curq.q2
#define q3 curq.q3
#define DEFAULT_MPU_HZ  (200)
#define q30  1073741824.0f
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;
    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}
static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) 
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}
unsigned long sensor_timestamp;
unsigned char more;
long quat[4];
#define PITCH_D -1.25
#define ROLL_D 5
void ahrs_thread_entry(void* parameter)
{
	const double gyroscale = 2000;	
	
	while(1 == mpu_init());
	//mpu_set_sensor
	while(1 == mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));  

	//mpu_configure_fifo
	while(1 == mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));

	//mpu_set_sample_rate
	while(1 == mpu_set_sample_rate(DEFAULT_MPU_HZ));

	//dmp_load_motion_driver_firmvare
	while(1 == dmp_load_motion_driver_firmware());

	//dmp_set_orientation
	while(1 == dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));

	//dmp_enable_feature
	while(1 == dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL));
	
	//dmp_set_fifo_rate
	while(1 == dmp_set_fifo_rate(DEFAULT_MPU_HZ));

	run_self_test();
	
	while(1 == mpu_set_dmp_state(1));
	
	rt_kprintf("start mpu6050\n");
	
	rt_kprintf("start ahrs\n");
	
	last_ahrs=rt_tick_get();
	
	while(1)
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
        if (sensors & INV_WXYZ_QUAT )
        {
            q0 = (double)quat[0] / q30;
            q1 = (double)quat[1] / q30;
            q2 = (double)quat[2] / q30;
            q3 = (double)quat[3] / q30;
			
			ahrs.gryo_pitch		= -gyro[0] 	* gyroscale / 32767.0;
			ahrs.gryo_roll		= -gyro[1] 	* gyroscale / 32767.0;
			ahrs.gryo_yaw		= +gyro[2] 	* gyroscale / 32767.0;
			
			ahrs.degree_roll  = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 +ROLL_D;   //+ Pitch_error; // pitch
            ahrs.degree_pitch = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3+PITCH_D ;  //+ Roll_error; // roll
            ahrs.degree_yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 ;  //+ Yaw_error;
			
			last_ahrs=rt_tick_get();
			
			if(en_out_ahrs)
				rt_kprintf("%d,%d,%d		%d\n",
				(s32)(ahrs.degree_pitch),
				(s32)(ahrs.degree_roll),
				(s32)(ahrs.degree_yaw	),
				(u32)(1.0/ahrs.time_span));
			rt_event_send(&ahrs_event,AHRS_EVENT_Update);
		}
//		else
//		{
//			LED4(0);
//			Timer4_GetSec();
//			debug("ahrs timeout!\n");
//			if(rt_tick_get()-last_ahrs>1000)
//			{
//				lost_ahrs=RT_TRUE;
//				debug("ahrs connection wrong!\n");
//				Motor_Set(0,0,0,0);
//				rt_thread_detach(&control_thread);
//				return;
//			}
//		}
		rt_thread_delay(2);
	}
}

void rt_init_thread_entry(void* parameter)
{
    rt_components_init();
	
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
	
	LED4(5);
	rt_kprintf("start device init\n");
	
	//rt_hw_i2c1_init();
    i2cInit();  
	rt_hw_spi2_init();
	
	rt_thread_init(&led_thread,
					"led",
					led_thread_entry,
					RT_NULL,
                    led_stack,
					256, 16, 1);
    rt_thread_startup(&led_thread);
	
	spi_flash_init();
	
//	mpu6050_init("i2c1");
//	bmp085_init("i2c1");
	
	rt_kprintf("device init succeed\n");
	
    if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("flash0 mount to /.\n");
    }
    else
    {
        rt_kprintf("flash0 mount to / failed.\n");
    }

	//default settings
	PID_Init(&pitch_pid,3.2,0,1.2);
	PID_Init(&roll_pid,3.2,0,1.2);
	PID_Init(&yaw_pid,0,0,3);
	PID_Init(&pout_pid,2.5,0,0);
	PID_Init(&rout_pid,2.5,0,0);
//	PID_Init(&pout_pid,0,0,0);
//	PID_Init(&rout_pid,0,0,0);
	
	load_settings(&settings,"/setting",&pitch_pid,&roll_pid);
	
	settings.roll_min	=settings.pitch_min	=settings.yaw_min	=1000;
	settings.th_min	=1000;
	settings.roll_max	=settings.pitch_max	=settings.yaw_max	=2000;
	settings.th_max	=2000;
	
	if(settings.pwm_init_mode)
	{
		Motor_Set(1000,1000,1000,1000);
		
		rt_thread_delay(RT_TICK_PER_SECOND*5);
		
		Motor_Set(0,0,0,0);
		
		settings.pwm_init_mode=0;
		save_settings(&settings,"/setting");
		
		rt_kprintf("pwm init finished!\n");
	}
	
	get_pid();
	PID_Set_Filt_Alpha(&pitch_pid,1.0/125.0,20.0);
	PID_Set_Filt_Alpha(&roll_pid,1.0/125.0,20.0);
	PID_Set_Filt_Alpha(&yaw_pid,1.0/125.0,20.0);
	PID_Set_Filt_Alpha(&pout_pid,1.0/125.0,20.0);
	PID_Set_Filt_Alpha(&rout_pid,1.0/125.0,20.0);
	pitch_pid.d/=10;
	roll_pid.d/=10;
	
	LED4(0);
	
	rt_event_init(&ahrs_event,"ahrs_e",RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&ahrs_thread,
					"ahrs",
					ahrs_thread_entry,
					RT_NULL,
                    ahrs_stack,
					512, 5, 10);
    rt_thread_startup(&ahrs_thread);
	
	rt_thread_init(&control_thread,
					"control",
					control_thread_entry,
					RT_NULL,
                    control_stack,
					512, 3, 2);
    rt_thread_startup(&control_thread);
	
	rt_thread_init(&correct_thread,
					"correct",
					correct_thread_entry,
					RT_NULL,
                    correct_stack,
					512, 12, 1);
    rt_thread_startup(&correct_thread);
	
	LED1(1);
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 1, 20);

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
