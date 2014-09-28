//l3g4200d
#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

struct l3g4200d_data
{
	rt_int16_t x;
	rt_int16_t y;
	rt_int16_t z;
};

rt_err_t l3g4200d_init(const char * i2c_bus_device_name);
rt_err_t l3g4200d_read(struct l3g4200d_data *data);
rt_bool_t l3g4200d_TestConnection(void);

#endif
