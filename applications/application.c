#include <board.h>
#include <rtthread.h>

#include <components.h>

#include "i2c1.h"
#include "spi2.h"
#include "bmp085.h"
#include "MPU6050.h"
#include "l3g4200d.h"
#include "ahrs.h"  
#include "LED.h"
#include "Motor.h"
#include "hardtimer.h"
#include "PID.h"
#include "settings.h"

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
static rt_uint8_t correct_stack[ 256 ];
static struct rt_thread correct_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t mpu6050_stack[ 512 ];
static struct rt_thread mpu6050_thread;
static struct rt_semaphore mpu6050_sem;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t l3g4200d_stack[ 512 ];
static struct rt_thread l3g4200d_thread;
static struct rt_semaphore l3g4200d_sem;

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
void control_thread_entry(void* parameter)
{
	u16 throttle=0;
	
	pitch_pid.expect=0;
	roll_pid.expect=0;
	yaw_pid.expect=0;
	
	rt_kprintf("start control\n");
	
	while(1)
	{
		LED3(balence*2);
		debug("i3:%d	i5:%d	o1:%d\n",PWM3_Time,PWM5_Time,throttle);
		if(pwmcon)
		{
			if(PWM3_Time<=settings.th_max&&PWM3_Time>=settings.th_min)
				throttle=(PWM3_Time-settings.th_min)*1000/(settings.th_max-settings.th_min);
			else 
				throttle=0;
			if(!balence)
				Motor_Set(throttle,throttle,throttle,throttle);
		}
		else if(PWM3_Time>settings.th_min&&PWM3_Time<settings.th_min+40&&
			PWM5_Time<1500&&PWM5_Time>500)
		{
			//set pwm middle
			if(!pwmcon)
			{
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
			if(throttle>30)
			{
				PID_Update(&pitch_pid	,ahrs.degree_pitch	,ahrs.gryo_pitch);
				PID_Update(&roll_pid	,ahrs.degree_roll	,ahrs.gryo_roll);
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
			settings.roll_min= min(settings.roll_min, PWM1_Time);
			settings.roll_max= max(settings.roll_max, PWM1_Time);	
			
			settings.pitch_min= min(settings.pitch_min, PWM2_Time);
			settings.pitch_max= max(settings.pitch_max, PWM2_Time);
			
			settings.yaw_min= min(settings.yaw_min, PWM4_Time);
			settings.yaw_max= max(settings.yaw_max, PWM4_Time);
			if(PWM3_Time<settings.th_min)
				settings.th_min=PWM3_Time;
			if(PWM3_Time>settings.th_max)
				settings.th_max=PWM3_Time;
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
		
		rt_thread_delay(10);
	}
}

void correct_thread_entry(void* parameter)
{
	rt_uint32_t e;
	{
		rt_uint16_t i;
		rt_int16_t * mpu1, * mpu2, * mpu3;
		rt_int16_t m1,m2,m3;
		
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
	while(1)
	{
		rt_thread_suspend(rt_thread_self());
	}
}

void ahrs_thread_entry(void* parameter)
{
	rt_uint32_t e;
	
	rt_kprintf("start ahrs\n");
	
	while(1)
	{
		rt_sem_release(&mpu6050_sem);
//		rt_sem_release(&l3g4200d_sem);
		
		if(rt_event_recv(&ahrs_event,1,
						RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,
						2,&e)==RT_EOK)
		{
			LED4(2);
//			debug("AHRS Received OK\n");
			ahrs_update();
			debug("%d,%d,%d		%d\n",
			(s32)(ahrs.degree_pitch),
			(s32)(ahrs.degree_roll),
			(s32)(ahrs.degree_yaw	),
			(u32)(1.0/ahrs.time_span));
			rt_event_send(&ahrs_event,AHRS_EVENT_Update);
		}
		else
		{
			LED4(0);
			debug("ahrs timeout!\n");
		}
		rt_thread_delay(2);
	}
}

void mpu6050_thread_entry(void* parameter)
{
	s16 AccelGyro[6];
	rt_sem_init(&mpu6050_sem,"mpu_t",0,RT_IPC_FLAG_FIFO);		
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);
	
	rt_kprintf("start mpu6050\n");
	
	while(1)
	{
		rt_sem_take(&mpu6050_sem,RT_WAITING_FOREVER);
		if( MPU6050_TestConnection())
		{
			MPU6050_GetRawAccelGyro(AccelGyro);
			
			//rt_kprintf("%d,%d,%d,%d,%d,%d\n",
//			AccelGyro[0],AccelGyro[1],AccelGyro[2],AccelGyro[3],AccelGyro[4],AccelGyro[5]);
			ahrs_put_mpu6050(AccelGyro);
			
			rt_event_send(&ahrs_event,1);
		}
		else
		{
			debug("error:lost mpu6050.\n");
		}
	}
}

void l3g4200d_thread_entry(void* parameter)
{
	struct l3g4200d_data data;
	rt_sem_init(&l3g4200d_sem,"l3g_t",0,RT_IPC_FLAG_FIFO);		
	
	rt_kprintf("start l3g4200d\n");
	
	while(1)
	{
		rt_sem_take(&l3g4200d_sem,RT_WAITING_FOREVER);
		if( l3g4200d_TestConnection())
		{
			l3g4200d_read(&data);
			debug("\t%d\t%d\t%d\n",data.x,data.y,data.z);
		}
		else
		{
			debug("error:lost l3g4200d.\n");
		}
	}
}

void rt_init_thread_entry(void* parameter)
{
    rt_components_init();
	
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
	
	LED4(5);
	rt_kprintf("start device init\n");
	
	rt_hw_i2c1_init();
	rt_hw_spi2_init();
	
	rt_thread_init(&led_thread,
					"led",
					led_thread_entry,
					RT_NULL,
                    led_stack,
					256, 16, 1);
    rt_thread_startup(&led_thread);
	
	spi_flash_init();
	
	mpu6050_init("i2c1");
//	bmp085_init("i2c1");
//	l3g4200d_init("i2c1");
//	while(1)
//		if( l3g4200d_TestConnection())
//		{
//			struct l3g4200d_data data;
//			l3g4200d_read(&data);
//			debug("\t%d\t%d\t%d\n",data.x,data.y,data.z);
//		}
//		else
//			debug("retry\n");
	
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
	PID_Init(&yaw_pid,0,0,1.5);
	settings.th_min	=2000;
	settings.roll_min	=settings.pitch_min	=settings.yaw_min	=1600;
	settings.th_max	=1000;
	settings.roll_max	=settings.pitch_max	=settings.yaw_max	=1400;
	load_settings(&settings,"/setting",&pitch_pid,&roll_pid);
	
	get_pid();
	
	LED4(0);
	
	rt_event_init(&ahrs_event,"ahrs_e",RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&mpu6050_thread,
					"mpu6050",
					mpu6050_thread_entry,
					RT_NULL,
                    mpu6050_stack,
					512, 8, 10);
    rt_thread_startup(&mpu6050_thread);
	
	
	rt_thread_init(&l3g4200d_thread,
					"l3g4200d",
					l3g4200d_thread_entry,
					RT_NULL,
                    l3g4200d_stack,
					512, 8, 10);
    //rt_thread_startup(&l3g4200d_thread);
	
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
					256, 12, 1);
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
