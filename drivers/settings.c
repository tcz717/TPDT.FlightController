#include "settings.h"
#include <dfs_posix.h> 
#include "ahrs.h"

struct setting_t settings;

rt_err_t load_settings(struct setting_t * s,const char * path,PID * pitch,PID * roll)
{
	int fd;
	rt_kprintf("start load settings\n");
	
	fd=open(path,O_RDWR | O_CREAT,0);
	
	if(fd>=0)
	{
		if(read(fd,&settings,sizeof(settings)) != sizeof(settings) ||
			settings.magic != SETTING_MAGIC)
		{
			rt_kprintf("read error,use default settings and save it.\n");
			s->magic	= SETTING_MAGIC;
			
			s->pitch_p	= pitch->p;
			s->pitch_i	= pitch->i;
			s->pitch_d	= pitch->d;
			
			s->roll_p	= roll->p;
			s->roll_i	= roll->i;
			s->roll_d	= roll->d;
			
//			s->yaw_p	= yaw->p;
//			s->yaw_i	= yaw->i;
//			s->yaw_d	= yaw->d;
			write(fd,s,sizeof(struct setting_t));
			close(fd);
			return -RT_EIO;
		}
		else
		{
			pitch->p	= s->pitch_p;
			pitch->i	= s->pitch_i;
			pitch->d	= s->pitch_d;
			
			roll->p		= s->roll_p;
			roll->i		= s->roll_i;
			roll->d		= s->roll_d;
			
			
			MPU6050_Diff[3]=settings.mpu6050_acc_diffx;
			MPU6050_Diff[4]=settings.mpu6050_acc_diffy;
			MPU6050_Diff[5]=settings.mpu6050_acc_diffz;
			
			rt_kprintf("settings load succeed.\n");
		}
		close(fd);
	}
	else
	{
		rt_kprintf("load error,use default settings.\n");
		return -RT_EIO;
	}
	return RT_EOK;
}

rt_err_t save_settings(struct setting_t * s,const char * path)
{
	int fd;
	
	fd=open(path,O_WRONLY | O_TRUNC,0);
	
	if(fd>=0)
	{
		write(fd,s,sizeof(struct setting_t));
		close(fd);
		return RT_EOK;
	}
	else
	{
		return -RT_EIO;
	}
}

void save()
{
	save_settings(&settings,"/setting");
	
	get_pid();
	
	rt_kprintf("pitch:		%d	%d	%d.\n",	settings.pitch_min,
											settings.pitch_mid,
											settings.pitch_max);
	rt_kprintf("roll:		%d	%d	%d.\n",	settings.roll_min,
											settings.roll_mid,
											settings.roll_max);
	rt_kprintf("yaw:		%d	%d	%d.\n",	settings.yaw_min,
											settings.yaw_mid,
											settings.yaw_max);
	rt_kprintf("throttle:	%d	%d.\n",		settings.th_min,
											settings.th_max);
}
FINSH_FUNCTION_EXPORT(save, save all config);

void get_pid()
{
	rt_kprintf("pitch:		%d.%d	%d.%2d	%d.%d.\n",	(s32)pitch_pid.p,(s32)(pitch_pid.p*10.0)%10,
														(s32)pitch_pid.i,(s32)(pitch_pid.i*100.0)%100,
														(s32)pitch_pid.d,(s32)(pitch_pid.d*10.0)%10);
	rt_kprintf("roll:		%d.%d	%d.%2d	%d.%d.\n",	(s32)roll_pid.p,(s32)(roll_pid.p*10.0)%10,
														(s32)roll_pid.i,(s32)(roll_pid.i*100.0)%100,
														(s32)roll_pid.d,(s32)(roll_pid.d*10.0)%10);
	rt_kprintf("yaw:		%d.%d	%d.%2d	%d.%d.\n",	(s32)yaw_pid.p,(s32)(yaw_pid.p*10.0)%10,
														(s32)yaw_pid.i,(s32)(yaw_pid.i*100.0)%100,
														(s32)yaw_pid.d,(s32)(yaw_pid.d*10.0)%10);
}
FINSH_FUNCTION_EXPORT(get_pid, show the value of pid);

void set_pitch(s16 p,s16 i,s16 d)
{
	PID_Init(&pitch_pid,p/10.0,i/100.0,d/10.0);
	
	settings.pitch_p	= pitch_pid.p;
	settings.pitch_i	= pitch_pid.i;
	settings.pitch_d	= pitch_pid.d;
	
	save_settings(&settings,"/setting");
	
	get_pid();
}
FINSH_FUNCTION_EXPORT(set_pitch, set the value of pid in pitch);

void set_roll(s16 p,s16 i,s16 d)
{
	PID_Init(&roll_pid,p/10.0,i/100.0,d/10.0);
	
	settings.roll_p	= roll_pid.p;
	settings.roll_i	= roll_pid.i;
	settings.roll_d	= roll_pid.d;
	
	save_settings(&settings,"/setting");
	
	get_pid();
}
FINSH_FUNCTION_EXPORT(set_roll, set the value of pid in roll);

void set_pwm()
{
	settings.pwm_init_mode=!settings.pwm_init_mode;
	
	save_settings(&settings,"/setting");
	
	if(settings.pwm_init_mode)
		rt_kprintf("warning:after reset, we will set moter out full!\n");
}
FINSH_FUNCTION_EXPORT(set_pwm, set pwm to the same);

extern u8 en_out_ahrs;
void out_ahrs()
{
	en_out_ahrs=!en_out_ahrs;
}
FINSH_FUNCTION_EXPORT(out_ahrs, output the ahrs);

void set_yaw(s16 p,s16 i,s16 d)
{
	PID_Init(&yaw_pid,p/10.0,i/100.0,d/10.0);
	
	settings.yaw_p	= yaw_pid.p;
	settings.yaw_i	= yaw_pid.i;
	settings.yaw_d	= yaw_pid.d;
	
	save_settings(&settings,"/setting");
	
	get_pid();
}
FINSH_FUNCTION_EXPORT(set_yaw, set the value of pid in yaw);

static void calculate_speed_print(rt_uint32_t speed)
{
    rt_uint32_t k,m;

    k = speed/1024UL;
    if( k )
    {
        m = k/1024UL;
        if( m )
        {
            rt_kprintf("%d.%dMbyte/s",m,k%1024UL*100/1024UL);
        }
        else
        {
            rt_kprintf("%d.%dKbyte/s",k,speed%1024UL*100/1024UL);
        }
    }
    else
    {
        rt_kprintf("%dbyte/s",speed);
    }
}

static rt_err_t _block_device_test(rt_device_t device)
{
    rt_err_t result;
    struct rt_device_blk_geometry geometry;
    rt_uint8_t * read_buffer  = RT_NULL;
    rt_uint8_t * write_buffer = RT_NULL;

    rt_kprintf("\r\n");

    if( (device->flag & RT_DEVICE_FLAG_RDWR) == RT_DEVICE_FLAG_RDWR )
    {
        // device can read and write.
        // step 1: open device
        result = rt_device_open(device,RT_DEVICE_FLAG_RDWR);
        if( result != RT_EOK )
        {
            return result;
        }

        // step 2: get device info
        rt_memset(&geometry, 0, sizeof(geometry));
        result = rt_device_control(device,
                                   RT_DEVICE_CTRL_BLK_GETGEOME,
                                   &geometry);
        if( result != RT_EOK )
        {
            rt_kprintf("device : %s cmd RT_DEVICE_CTRL_BLK_GETGEOME failed.\r\n");
            return result;
        }
        rt_kprintf("device info:\r\n");
        rt_kprintf("sector  size : %d byte\r\n", geometry.bytes_per_sector);
        rt_kprintf("sector count : %d \r\n", geometry.sector_count);
        rt_kprintf("block   size : %d byte\r\n", geometry.block_size);

        rt_kprintf("\r\n");
        read_buffer = rt_malloc(geometry.bytes_per_sector);
        if( read_buffer == RT_NULL )
        {
            rt_kprintf("no memory for read_buffer!\r\n");
            goto __return;
        }
        write_buffer = rt_malloc(geometry.bytes_per_sector);
        if( write_buffer == RT_NULL )
        {
            rt_kprintf("no memory for write_buffer!\r\n");
            goto __return;
        }

        /* step 3:  R/W test */
        {
            rt_uint32_t i,err_count, sector_no;
            rt_uint8_t * data_point;

            i = rt_device_read(device, 0, read_buffer, 1);
            if(i != 1)
            {
                rt_kprintf("read device :%s ", device->parent.name);
                rt_kprintf("the first sector failed.\r\n");
                goto __return;
            }

            data_point = write_buffer;
            for(i=0; i<geometry.bytes_per_sector; i++)
            {
                *data_point++ = (rt_uint8_t)i;
            }

            /* write first sector */
            sector_no = 0;
            data_point = write_buffer;
            *data_point++ = (rt_uint8_t)sector_no;
            i = rt_device_write(device, sector_no, write_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("read the first sector success!\r\n");
                rt_kprintf("but write device :%s ", device->parent.name);
                rt_kprintf("the first sector failed.\r\n");
                rt_kprintf("maybe readonly!\r\n");
                goto __return;
            }

            /* write the second sector */
            sector_no = 1;
            data_point = write_buffer;
            *data_point++ = (rt_uint8_t)sector_no;
            i = rt_device_write(device,sector_no,write_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("write device :%s ",device->parent.name);
                rt_kprintf("the second sector failed.\r\n");
                goto __return;
            }

            /* write the end sector */
            sector_no = geometry.sector_count-1;
            data_point = write_buffer;
            *data_point++ = (rt_uint8_t)sector_no;
            i = rt_device_write(device,sector_no,write_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("write device :%s ",device->parent.name);
                rt_kprintf("the end sector failed.\r\n");
                goto __return;
            }

            /* verify first sector */
            sector_no = 0;
            i = rt_device_read(device,sector_no,read_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("read device :%s ",device->parent.name);
                rt_kprintf("the first sector failed.\r\n");
                goto __return;
            }
            err_count = 0;
            data_point = read_buffer;
            if( (*data_point++) != (rt_uint8_t)sector_no)
            {
                err_count++;
            }
            for(i=1; i<geometry.bytes_per_sector; i++)
            {
                if( (*data_point++) != (rt_uint8_t)i )
                {
                    err_count++;
                }
            }
            if( err_count > 0 )
            {
                rt_kprintf("verify device :%s ",device->parent.name);
                rt_kprintf("the first sector failed.\r\n");
                goto __return;
            }

            /* verify sector sector */
            sector_no = 1;
            i = rt_device_read(device,sector_no,read_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("read device :%s ",device->parent.name);
                rt_kprintf("the second sector failed.\r\n");
                goto __return;
            }
            err_count = 0;
            data_point = read_buffer;
            if( (*data_point++) != (rt_uint8_t)sector_no)
            {
                err_count++;
            }
            for(i=1; i<geometry.bytes_per_sector; i++)
            {
                if( (*data_point++) != (rt_uint8_t)i )
                {
                    err_count++;
                }
            }
            if( err_count > 0 )
            {
                rt_kprintf("verify device :%s ",device->parent.name);
                rt_kprintf("the second sector failed.\r\n");
                goto __return;
            }

            /* verify the end sector */
            sector_no = geometry.sector_count-1;
            i = rt_device_read(device,sector_no,read_buffer,1);
            if( i != 1 )
            {
                rt_kprintf("read device :%s ",device->parent.name);
                rt_kprintf("the end sector failed.\r\n");
                goto __return;
            }
            err_count = 0;
            data_point = read_buffer;
            if( (*data_point++) != (rt_uint8_t)sector_no)
            {
                err_count++;
            }
            for(i=1; i<geometry.bytes_per_sector; i++)
            {
                if( (*data_point++) != (rt_uint8_t)i )
                {
                    err_count++;
                }
            }
            if( err_count > 0 )
            {
                rt_kprintf("verify device :%s ",device->parent.name);
                rt_kprintf("the end sector failed.\r\n");
                goto __return;
            }
            rt_kprintf("device R/W test pass!\r\n");

        } /* step 3: I/O R/W test */

        rt_kprintf("\r\nRT_TICK_PER_SECOND:%d\r\n", RT_TICK_PER_SECOND);

        // step 4: continuous single sector speed test
        {
            rt_uint32_t tick_start,tick_end;
            rt_uint32_t i;

            rt_kprintf("\r\ncontinuous single sector speed test:\r\n");

            if( geometry.sector_count < 10 )
            {
                rt_kprintf("device sector_count < 10, speed test abort!\r\n");
            }
            else
            {
                unsigned int sector;

                // sign sector write
                rt_kprintf("write: ");
                sector = 0;
                tick_start = rt_tick_get();
                for(i=0; i<200; i++)
                {
                    sector += rt_device_write(device, i, read_buffer, 1);
                    if((i != 0) && ((i%4) == 0) )
                    {
                        if(sector < 4)
                        {
                            rt_kprintf("#");
                        }
                        else
                        {
                            rt_kprintf("<");
                        }
                        sector = 0;
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\nwrite 200 sector from %d to %d, ",tick_start,tick_end);
                calculate_speed_print( (geometry.bytes_per_sector*200UL*RT_TICK_PER_SECOND)/(tick_end-tick_start) );
                rt_kprintf("\r\n");

                // sign sector read
                rt_kprintf("read : ");
                sector = 0;
                tick_start = rt_tick_get();
                for(i=0; i<200; i++)
                {
                    sector += rt_device_read(device, i, read_buffer, 1);
                    if((i != 0) && ((i%4) == 0) )
                    {
                        if(sector < 4)
                        {
                            rt_kprintf("#");
                        }
                        else
                        {
                            rt_kprintf(">");
                        }
                        sector = 0;
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\nread 200 sector from %d to %d, ",tick_start,tick_end);
                calculate_speed_print( (geometry.bytes_per_sector*200UL*RT_TICK_PER_SECOND)/(tick_end-tick_start) );
                rt_kprintf("\r\n");
            }
        }// step 4: speed test

        // step 5: random single sector speed test
        {
            rt_uint32_t tick_start,tick_end;
            rt_uint32_t i;

            rt_kprintf("\r\nrandom single sector speed test:\r\n");

            if( geometry.sector_count < 10 )
            {
                rt_kprintf("device sector_count < 10, speed test abort!\r\n");
            }
            else
            {
                unsigned int sector;

                // sign sector write
                rt_kprintf("write: ");
                sector = 0;
                tick_start = rt_tick_get();
                for(i=0; i<200; i++)
                {
                    sector += rt_device_write(device, (geometry.sector_count / 10) * (i%10) + (i%10), read_buffer, 1);
                    if((i != 0) && ((i%4) == 0) )
                    {
                        if(sector < 4)
                        {
                            rt_kprintf("#");
                        }
                        else
                        {
                            rt_kprintf("<");
                        }
                        sector = 0;
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\nwrite 200 sector from %d to %d, ",tick_start,tick_end);
                calculate_speed_print( (geometry.bytes_per_sector*200UL*RT_TICK_PER_SECOND)/(tick_end-tick_start) );
                rt_kprintf("\r\n");

                // sign sector read
                rt_kprintf("read : ");
                sector = 0;
                tick_start = rt_tick_get();
                for(i=0; i<200; i++)
                {
                    sector += rt_device_read(device, (geometry.sector_count / 10) * (i%10) + (i%10), read_buffer, 1);
                    if((i != 0) && ((i%4) == 0) )
                    {
                        if(sector < 4)
                        {
                            rt_kprintf("#");
                        }
                        else
                        {
                            rt_kprintf(">");
                        }
                        sector = 0;
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\nread 200 sector from %d to %d, ",tick_start,tick_end);
                calculate_speed_print( (geometry.bytes_per_sector*200UL*RT_TICK_PER_SECOND)/(tick_end-tick_start) );
                rt_kprintf("\r\n");
            }
        }// step 4: speed test

        /* step 6: multiple sector speed test */
        {
            rt_uint8_t * multiple_buffer;
            rt_uint8_t * ptr;
            rt_uint32_t tick_start,tick_end;
            rt_uint32_t sector,i;

            rt_kprintf("\r\nmultiple sector speed test\r\n");

            for(sector=2; sector<256; sector=sector*2)
            {
                multiple_buffer = rt_malloc(geometry.bytes_per_sector * sector);

                if(multiple_buffer == RT_NULL)
                {
                    rt_kprintf("no memory for %d sector! multiple sector speed test abort!\r\n", sector);
                    break;
                }

                rt_memset(multiple_buffer, sector, geometry.bytes_per_sector * sector);
                rt_kprintf("write: ");
                tick_start = rt_tick_get();
                for(i=0; i<10; i++)
                {
                    rt_size_t n;
                    n = rt_device_write(device, 50, multiple_buffer, sector);
                    if(n == sector)
                    {
                        rt_kprintf("<");
                    }
                    else
                    {
                        rt_kprintf("#");
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\n");
                rt_kprintf("multiple write %d sector speed : ", sector);
                calculate_speed_print( (geometry.bytes_per_sector * sector * 10 * RT_TICK_PER_SECOND)/(tick_end-tick_start) );
                rt_kprintf("\r\n");

                rt_memset(multiple_buffer, ~sector, geometry.bytes_per_sector * sector);
                rt_kprintf("read : ");
                tick_start = rt_tick_get();
                for(i=0; i<10; i++)
                {
                    rt_size_t n;
                    n = rt_device_read(device, 50, multiple_buffer, sector);
                    if(n == sector)
                    {
                        rt_kprintf(">");
                    }
                    else
                    {
                        rt_kprintf("#");
                    }
                }
                tick_end = rt_tick_get();
                rt_kprintf("\r\n");
                rt_kprintf("multiple read %d sector speed : ", sector);
                calculate_speed_print( (geometry.bytes_per_sector * sector * 10 * RT_TICK_PER_SECOND)/(tick_end-tick_start) );

                ptr = multiple_buffer;
                for(i=0; i<geometry.bytes_per_sector * sector; i++)
                {
                    if(*ptr != sector)
                    {
                        rt_kprintf(" but data verify fail!");
                        break;
                    }
                    ptr++;
                }
                rt_kprintf("\r\n\r\n");

                rt_free(multiple_buffer);
            }
        } /* step 5: multiple sector speed test */

        return RT_EOK;
    }// device can read and write.
    else
    {
        // device read only
        return RT_EOK;
    }// device read only

__return:
    if( read_buffer != RT_NULL )
    {
        rt_free(read_buffer);
    }
    if( write_buffer != RT_NULL )
    {
        rt_free(write_buffer);
    }
    return RT_ERROR;
}

int device_test(const char * device_name)
{
    rt_device_t device = RT_NULL;

    // step 1:find device
    device = rt_device_find(device_name);
    if( device == RT_NULL)
    {
        rt_kprintf("device %s: not found!\r\n");
        return RT_ERROR;
    }

    // step 2:init device
    if (!(device->flag & RT_DEVICE_FLAG_ACTIVATED))
    {
        rt_err_t result;
        result = rt_device_init(device);
        if (result != RT_EOK)
        {
            rt_kprintf("To initialize device:%s failed. The error code is %d\r\n",
                       device->parent.name, result);
            return result;
        }
        else
        {
            device->flag |= RT_DEVICE_FLAG_ACTIVATED;
        }
    }

    // step 3: device test
    switch( device->type )
    {
    case RT_Device_Class_Block :
        rt_kprintf("block device!\r\n");
        return _block_device_test(device);
    default:
        rt_kprintf("unkown device type : %02X",device->type);
        return RT_ERROR;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(device_test, e.g: device_test("sd0"));
#endif
