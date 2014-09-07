#include "stm32f10x.h"
#include "i2c1.h"
#include "board.h"

#define I2Cx                  I2C1
#define I2Cx_CLK              RCC_APB1Periph_I2C1

#define I2Cx_SCL_PIN          GPIO_Pin_6
#define I2Cx_SCL_GPIO_PORT    GPIOB
#define I2Cx_SCL_GPIO_CLK     RCC_APB2Periph_GPIOB

#define I2Cx_SDA_PIN          GPIO_Pin_7
#define I2Cx_SDA_GPIO_PORT    GPIOB
#define I2Cx_SDA_GPIO_CLK     RCC_APB2Periph_GPIOB

#define I2Cx_DR_ADDR          ((u32)&I2C1->DR)
#define I2Cx_SPEED            ((u32)400000)

#define DMAx                  DMA1
#define DMAx_CLK              RCC_AHB1Periph_DMA1

//TODO: Use Right Channel
#define DMAx_TX_CHANNEL       DMA1_Channel6
#define DMAx_TX_FLAG_GLIF     DMA1_IT_GL6
#define DMAx_TX_FLAG_TEIF     DMA1_IT_TE6
#define DMAx_TX_FLAG_HTIF     DMA1_IT_HT6
#define DMAx_TX_FLAG_TCIF     DMA1_IT_TC6
#define DMAx_TX_IRQn          DMA1_Channel6_IRQn
#define I2C_TX_DMA_IRQ        DMA1_Channel6_IRQHandler

#define DMAx_RX_CHANNEL       DMA1_Channel7
#define DMAx_RX_FLAG_GLIF     DMA1_IT_GL7
#define DMAx_RX_FLAG_TEIF     DMA1_IT_TE7
#define DMAx_RX_FLAG_HTIF     DMA1_IT_HT7
#define DMAx_RX_FLAG_TCIF     DMA1_IT_TC7
#define DMAx_RX_IRQn          DMA1_Channel7_IRQn
#define I2C_RX_DMA_IRQ        DMA1_Channel7_IRQHandler

#define CHECKTIME(TIME)		if(rt_tick_get()>(TIME)+1){ret=-RT_ETIMEOUT;goto out;}

static struct rt_i2c_bus_device i2c_bus1;
//TODO:Add TX and RX DMA Semaphore
static struct rt_semaphore DMA_RX_Sem;
static struct rt_semaphore DMA_TX_Sem;
static rt_size_t i2c1_recv_bytes(struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;
	DMA_InitTypeDef DMA_InitStructure;
	
	if (len < 2) 
	{
//		if ((msg->flags & RT_I2C_NO_READ_ACK))
//		{
//			I2C_AcknowledgeConfig(I2Cx,DISABLE);
//			(void)I2Cx->SR2;
//		}
	   
		while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) == RESET);
		msg->buf[bytes++] = I2C_ReceiveData(I2Cx);
		
//		if ((msg->flags & RT_I2C_NO_READ_ACK))
//			I2C_AcknowledgeConfig(I2Cx,ENABLE);
	} 
	else
	{
		DMA_Cmd(DMAx_RX_CHANNEL, DISABLE);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2Cx_DR_ADDR;
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(msg->buf);
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = msg->len;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
		DMA_Init(DMAx_RX_CHANNEL, &DMA_InitStructure);

		I2C_DMALastTransferCmd(I2Cx, ENABLE);
		I2C_DMACmd(I2Cx,ENABLE);
		
		DMA_ITConfig(DMAx_RX_CHANNEL, DMA_IT_TC|DMA_IT_TE, ENABLE);
		
		DMA_Cmd(DMAx_RX_CHANNEL, ENABLE);
	}
	//TODO: Check finshed
	if(rt_sem_take(&DMA_RX_Sem,20)!=RT_EOK)
		return 0;
    return len;
}

static rt_size_t stm32_i2c_send_bytes(struct rt_i2c_msg *msg)
{
    rt_size_t len = msg->len;
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2Cx_DR_ADDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(msg->buf);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = msg->len;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMAx_TX_CHANNEL, &DMA_InitStructure);

	I2C_DMACmd(I2Cx,ENABLE);
	DMA_Cmd(DMAx_TX_CHANNEL, ENABLE);

	//TODO: Check finshed
	if(rt_sem_take(&DMA_TX_Sem,20)!=RT_EOK)
		return 0;
    return len;
}

rt_size_t i2c1_master_xfer(struct rt_i2c_bus_device *bus,
						 struct rt_i2c_msg msgs[],
						 rt_uint32_t num)
{
	struct rt_i2c_msg *msg;
    rt_int32_t i, ret;
	rt_tick_t tick=rt_tick_get();
	
	I2C_GenerateSTART(I2Cx, ENABLE);
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		CHECKTIME(tick);

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                I2C_GenerateSTART(I2Cx, ENABLE);
				while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
					CHECKTIME(tick);
            }
            if (msg->flags & RT_I2C_RD)
			{
				I2C_Send7bitAddress(I2Cx, msg->addr, I2C_Direction_Receiver);
                while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
					CHECKTIME(tick);
			}
            else
			{
				I2C_Send7bitAddress(I2Cx, msg->addr, I2C_Direction_Transmitter);
                while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
					CHECKTIME(tick);
			}

        }
        if (msg->flags & RT_I2C_RD)
        {
            ret = i2c1_recv_bytes(msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = stm32_i2c_send_bytes(msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    I2C_GenerateSTOP(I2Cx, ENABLE);
	
    return ret;
}

rt_size_t i2c1_slave_xfer(struct rt_i2c_bus_device *bus,
                            struct rt_i2c_msg msgs[],
                            rt_uint32_t num)
{
    return RT_EOK;
}

rt_err_t i2c1_i2c_bus_control(struct rt_i2c_bus_device *bus,
                                rt_uint32_t a,
                                rt_uint32_t b)
{
    return RT_EOK;
}

rt_err_t i2c_register_read(struct rt_i2c_bus_device *bus,
						rt_uint16_t daddr,
						rt_uint8_t raddr,
						void *buffer,
						rt_size_t count)
{
	struct rt_i2c_msg msgs[2];
	rt_err_t err;
    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
	
	msgs[0].addr=daddr;
	msgs[0].buf=&raddr;
	msgs[0].len=1;
	msgs[0].flags=RT_I2C_WR;
	
	msgs[1].addr=daddr;
	msgs[1].buf=buffer;
	msgs[1].len=count;
	msgs[1].flags=RT_I2C_RD|RT_I2C_NO_READ_ACK;
	
	err=rt_i2c_transfer(bus,msgs,2);
	
	if(err<0)
		return -err;
	else
		return RT_EOK;
}

rt_err_t i2c_register_write(struct rt_i2c_bus_device *bus,
						rt_uint16_t daddr,
						rt_uint8_t raddr,
						void *buffer,
						rt_size_t count)
{
	struct rt_i2c_msg msgs[2];
	rt_err_t err;
    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
	
	msgs[0].addr=daddr;
	msgs[0].buf=&raddr;
	msgs[0].len=1;
	msgs[0].flags=RT_I2C_WR;
	
	msgs[1].addr=daddr;
	msgs[1].buf=buffer;
	msgs[1].len=count;
	msgs[1].flags=RT_I2C_WR|RT_I2C_NO_START;
	
	err=rt_i2c_transfer(bus,msgs,2);
	
	if(err<0)
		return -err;
	else
		return RT_EOK;
}

static void RCC_Configuration(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(I2Cx_SCL_GPIO_CLK |
						   I2Cx_SDA_GPIO_CLK  , ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;

	GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = I2Cx_SPEED;
	I2C_Cmd(I2Cx, ENABLE);
	I2C_Init(I2Cx, &I2C_InitStruct);
	
	DMA_ClearFlag(DMAx_TX_FLAG_GLIF | DMAx_TX_FLAG_TEIF |
		      DMAx_TX_FLAG_HTIF | DMAx_TX_FLAG_TCIF);
	DMA_Cmd(DMAx_TX_CHANNEL, DISABLE);
	DMA_DeInit(DMAx_TX_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2Cx_DR_ADDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0xFFFF;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMAx_TX_CHANNEL, &DMA_InitStructure);
	
	DMA_ClearFlag(DMAx_RX_FLAG_GLIF | DMAx_RX_FLAG_TEIF |
		      DMAx_RX_FLAG_HTIF | DMAx_RX_FLAG_TCIF);
	DMA_Cmd(DMAx_RX_CHANNEL, DISABLE);
	DMA_DeInit(DMAx_RX_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)I2Cx_DR_ADDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 0xFFFF;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMAx_RX_CHANNEL, &DMA_InitStructure);
	
	DMA_ITConfig(DMAx_TX_CHANNEL, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMAx_RX_CHANNEL, DMA_IT_TC|DMA_IT_TE, ENABLE);
}

static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMAx_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMAx_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
static const struct rt_i2c_bus_device_ops i2c1_ops=
{
	i2c1_master_xfer,
	i2c1_slave_xfer,
	i2c1_i2c_bus_control,
};
void rt_hw_i2c1_init(void)
{
	struct rt_i2c_bus_device *bus;
	
	RCC_Configuration();
	NVIC_Configuration();
    GPIO_Configuration();
	
	bus=&i2c_bus1;
    rt_memset(bus, 0, sizeof(struct rt_i2c_bus_device));
	bus->ops = &i2c1_ops;
	
	rt_sem_init(&DMA_TX_Sem,"i2c_tx",0,RT_IPC_FLAG_FIFO);
	rt_sem_init(&DMA_RX_Sem,"i2c_rx",0,RT_IPC_FLAG_FIFO);
	
	rt_i2c_bus_device_register(bus,"i2c1");
}
//TODO:Add DMA IRQ Handler
void DMA1_Channel6_IRQHandler(void)
{
	if (DMA_GetITStatus(DMAx_TX_FLAG_TCIF) != RESET) {
		rt_interrupt_enter();
		
		DMA_Cmd(DMAx_TX_CHANNEL, DISABLE);
		
		DMA_ClearITPendingBit(DMAx_TX_FLAG_TCIF);
		
		rt_sem_release(&DMA_TX_Sem);
		
		rt_interrupt_leave();
	}
}

void DMA1_Channel7_IRQHandler(void)
{
	if (DMA_GetITStatus(DMAx_RX_FLAG_TCIF) != RESET) {
		rt_interrupt_enter();
		
		DMA_Cmd(DMAx_RX_CHANNEL, DISABLE);
		DMA_ClearITPendingBit(DMAx_RX_FLAG_TCIF);
		rt_sem_release(&DMA_RX_Sem);
		
		rt_interrupt_leave();
	}
	else
	{
		DMA_ClearITPendingBit(DMAx_RX_FLAG_TEIF);
		rt_kprintf("DMA RX ERROR!\nSR1=0x%04x\nSR1=0x%04x\nDR=0x%04x\nCCR=0x%04x\nCR1=0x%04x\nCR2=0x%04x\n",
					I2Cx->SR1,
					I2Cx->SR2,
					I2Cx->DR,
					I2Cx->CCR,
					I2Cx->CR1,
					I2Cx->CR2);
	}
}
