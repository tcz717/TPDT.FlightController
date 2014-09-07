#include <board.h>
#include <rtthread.h>

#include <components.h>

#include "i2c1.h"
#include "bmp085.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#include "led.h"

//ALIGN(RT_ALIGN_SIZE)
//static rt_uint8_t led_stack[ 512 ];
//static struct rt_thread led_thread;

void rt_init_thread_entry(void* parameter)
{
    rt_components_init();
	
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
	
	rt_hw_i2c1_init();
	bmp085_init("i2c1");
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
