//l3g4200d
#include "stm32f10x.h"
#include "l3g4200d.h"
#include "board.h"
#include "i2c1.h"
#include "math.h"
#include <components.h>

#define	L3G4200_Addr   0xD0
//**********L3G4200D内部寄存器地址*********
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

#define L3G4200D_INT2_PIN          GPIO_Pin_3
#define L3G4200D_INT2_GPIO_PORT    GPIOC
#define L3G4200D_INT2_GPIO_CLK     RCC_APB2Periph_GPIOC

#define L3G4200D_INT2_EXIT_IRQn			EXTI1_IRQn
#define L3G4200D_INT2_GPIO_PortSource		GPIO_PortSourceGPIOC
#define L3G4200D_INT2_GPIO_PinSource     	GPIO_PinSource3
#define L3G4200D_INT2_EXTI_Line       		EXTI_Line3

static struct rt_i2c_bus_device * i2c_device;
static struct rt_semaphore l3g4200d_int2;

void get_l3g4200d()
{
	struct l3g4200d_data data;
	if(l3g4200d_TestConnection() )
	{
		if(l3g4200d_read(&data)==RT_EOK)
		{
			rt_kprintf("p:%d t:%d h:%d\n",
						data.x,data.y,data.z);
			return;
		}
		rt_kprintf("read time out!\n");
	}
	else
		rt_kprintf("l3g4200d connection error.");
}
FINSH_FUNCTION_EXPORT(get_l3g4200d, get l3g4200d data)

static u16 Multiple_read(u8 ST_Address)
{
	u8 tmp[2];
	if(i2c_register_read(i2c_device,L3G4200_Addr,ST_Address,tmp,2)!=RT_EOK)
		return 0;
	return tmp[0]<<8 | tmp[1];
}
void Single_Write(u8 ST_Address,u8 data)
{
	i2c_register_write(i2c_device,L3G4200_Addr,ST_Address,&data,1);
}
static void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(L3G4200D_INT2_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
}
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = L3G4200D_INT2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(L3G4200D_INT2_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(L3G4200D_INT2_GPIO_PortSource,L3G4200D_INT2_GPIO_PinSource);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	
	EXTI_Init(&EXTI_InitStructure);
}
static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = L3G4200D_INT2_EXIT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t l3g4200d_GetDeviceID()
{
    uint8_t tmp;
	i2c_register_read(i2c_device,L3G4200_Addr,WHO_AM_I,&tmp,1);
    return tmp; 
}
rt_bool_t l3g4200d_TestConnection() 
{
    return l3g4200d_GetDeviceID() == 0xD3;
}

rt_err_t l3g4200d_read(struct l3g4200d_data *data)
{
	u8 tmp[6];
//	i2c_register_read(i2c_device,L3G4200_Addr,OUT_X_L,tmp,6);
	
	i2c_register_read(i2c_device,L3G4200_Addr,OUT_X_L,tmp,0);
	i2c_register_read(i2c_device,L3G4200_Addr,OUT_X_H,tmp+1,0);
	data->x=	(tmp[1]<<8)|tmp[0];


	i2c_register_read(i2c_device,L3G4200_Addr,OUT_Y_L,tmp+2,0);
	i2c_register_read(i2c_device,L3G4200_Addr,OUT_Y_H,tmp+3,0);
	data->y=	(tmp[3]<<8)|tmp[2];

	i2c_register_read(i2c_device,L3G4200_Addr,OUT_Y_L,tmp+4,0);
	i2c_register_read(i2c_device,L3G4200_Addr,OUT_Y_H,tmp+5,0);
	data->z=	(tmp[5]<<8)|tmp[4];
	
	return RT_EOK;
}

rt_err_t l3g4200d_init(const char * i2c_bus_device_name)
{
    i2c_device = rt_i2c_bus_device_find(i2c_bus_device_name);
    if(i2c_device == RT_NULL)
    {
        rt_kprintf("i2c bus device %s not found!\r\n", i2c_bus_device_name);
        return -RT_ENOSYS;
    }
	
//	RCC_Configuration();
//	NVIC_Configuration();
//	GPIO_Configuration();
	
	Single_Write(CTRL_REG1, 0x0f);
	Single_Write(CTRL_REG2, 0x00);
	Single_Write(CTRL_REG3, 0x08);
	Single_Write(CTRL_REG4, 0x30);	//+-2000dps
	Single_Write(CTRL_REG5, 0x00);
	
//	rt_sem_init(&l3g4200d_int2,"l3g_INT2",0,RT_IPC_FLAG_FIFO);
	
	return RT_EOK;
}

void EXTI3_IRQHandler()
{
	if(EXTI_GetITStatus(L3G4200D_INT2_EXTI_Line)==SET)
	{
		rt_interrupt_enter();
		
		rt_sem_release(&l3g4200d_int2);
		EXTI_ClearITPendingBit(L3G4200D_INT2_EXTI_Line);
		
		rt_interrupt_leave();
	}
}
