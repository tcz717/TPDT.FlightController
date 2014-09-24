#include <board.h>
#include <rtthread.h>

#include <components.h>

#include "i2c1.h"
#include "bmp085.h"
#include "MPU6050.h"
#include "ahrs.h"  
#include "bluetooth.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#include "led.h"

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t control_stack[ 512 ];
static struct rt_thread control_thread;


ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t ahrs_stack[ 256 ];
static struct rt_thread ahrs_thread;
static struct rt_event ahrs_event;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t mpu6050_stack[ 512 ];
static struct rt_thread mpu6050_thread;
static struct rt_semaphore mpu6050_sem;

void control_thread_entry(void* parameter)
{
	
}


void ahrs_thread_entry(void* parameter)
{
	rt_uint32_t e;
	while(1)
	{
		rt_sem_release(&mpu6050_sem);
		
		if(rt_event_recv(&ahrs_event,1,
						RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,
						10,&e)==RT_EOK)
		{
			
		}
	}
}

void mpu6050_thread_entry(void* parameter)
{
	s16 AccelGyro[6];
	rt_sem_init(&mpu6050_sem,"mpu_t",0,RT_IPC_FLAG_FIFO);		
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);

	while(1)
	{
//		rt_sem_take(&mpu6050_sem,RT_WAITING_FOREVER);
		if( MPU6050_TestConnection())
		{
			MPU6050_GetRawAccelGyro(AccelGyro);
			rt_kprintf("%d,%d,%d,%d,%d,%d\n",AccelGyro[0],AccelGyro[1],
			AccelGyro[2],AccelGyro[3],
			AccelGyro[4],AccelGyro[5]);
		}
		else
			return;
		rt_thread_delay(5);
	}
}

void rt_init_thread_entry(void* parameter)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    rt_components_init();
	
//	rt_hw_bluetooth_init();
//	rt_console_set_device("bt1");
//    finsh_set_device("bt1");
	
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
	
	//while(1)rt_kprintf("%d\n", GPIOB->IDR&GPIO_Pin_7);
	
	rt_hw_i2c1_init();
//	bmp085_init("i2c1");

	mpu6050_init("i2c1");
	
	rt_event_init(&ahrs_event,"ahrs_e",RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&mpu6050_thread,
					"mpu6050",
					mpu6050_thread_entry,
					RT_NULL,
                    mpu6050_stack,
					512, 8, 20);
    rt_thread_startup(&mpu6050_thread);
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
