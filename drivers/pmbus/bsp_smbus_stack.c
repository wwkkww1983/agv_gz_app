/**
  ******************************************************************************
  * @file    stm32_SMBUS_stack.c
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    28-June-2017
  * @brief   This file provides a set of functions needed to manage the SMBUS STACK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_smbus_stack.h"

/** @addtogroup STM32_SMBUS_STACK
  * @{
  */

#define		TIME_OUT_30MS	240000  //30ms
#define		TIME_OUT_28MS	220000  //28ms
#define		TIME_OUT_25MS	200000  //25ms 待测试

#define		TIME_OUT	TIME_OUT_25MS

#define		TRUE	1
#define		FALSE	0

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */


/* 定义I2C总线连接的GPIO端口 */
#define GPIO_PORT_I2C	GPIOB						/* GPIO端口 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_Pin_6					/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7					/* 连接到SDA数据线的GPIO */


/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
#if 0	
	#define I2C_SCL_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* 读SDA口线状态 */

/* 这个分支选择直接寄存器操作实现IO读写 */
/*注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
#else
void 		I2C_SCL_1(void); // 加入clock low extending 检测机制	 
//	#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
	#define I2C_SCL_READ()  ((GPIO_PORT_I2C->IDR & I2C_SCL_PIN) != 0)	/* 读SCL口线状态 */
#endif

void i2c_Start(void);
//uint8_t i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);


void clear_i2c_timeout(void);
void set_i2c_timeout(void);
uint8_t get_i2c_timeout(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
void i2c_gpio_init(void);


/****************************************************************************************
basic 
*****************************************************************************************/

/* 标示i2c通讯是否超时 */
static uint8_t timeout_flag = FALSE;

void clear_i2c_timeout(void)
{
	timeout_flag = FALSE;	
}
void set_i2c_timeout(void)
{
	timeout_flag = TRUE;
}
uint8_t get_i2c_timeout(void)
{
	return timeout_flag;
}

/* 简单的延时 */
static void smbus_delay_us( __IO uint32_t _T )
{
	uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 11; i++);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过逻辑分析仪测试得到的。
    工作条件：CPU主频72MHz ，MDK编译环境，1级优化
  
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
	  30 -> scl = 95khz
	  40 -> scl = 73.5  - 75 khz
	  50 -> scl = 60.97 - 62.5 khz
	  60 -> scl = 52 khz
	  100 -> scl =  32.7khz
	  150 -> scl =  22.7khz 
	  200 -> scl =  17khz
	*/
	for (i = 0; i < 40; i++);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;
	
	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();

	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	函 数 名: i2c_CfgGpio
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_CfgGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* 开漏输出 */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	i2c_Stop();
}



/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	
	i2c_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_Address | I2C_WR);
	ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

	i2c_Stop();			/* 发送停止信号 */

	// ucAck = 0，表示设备应答
	if (ucAck == 0)
		return 1;
	else
		return 0;
}

void I2C_SCL_1(void) 
{
	uint32_t time_out = TIME_OUT;
	uint8_t num = 0;
	
	for (num = 0; num < 3; num ++)
	{
//		time_out = TIME_OUT;
		do
		{
			if ( time_out-- == 0 )
				goto cmd_fail;
			
			GPIO_PORT_I2C->BSRR = I2C_SCL_PIN; // 拉高SCL
			smbus_delay_us(5);
		} while ( !I2C_SCL_READ() ); // 判断SCL是否被拉高
	}
	
	return;

cmd_fail: 
//	i2c_Stop();  // 不能加，不然可能会导致死循环
	set_i2c_timeout();
}

void i2c_gpio_init(void)
{
	i2c_CfgGpio();
}


/****************************************************************************************
api
*****************************************************************************************/

static StatusTypeDef SMBUS_Master_Read_Multi_Byte(uint8_t _addr, uint8_t _cmd,uint8_t *_buf, uint8_t _size)
{
	uint8_t idx = 0;
	
	/* 发送启动信号 */
	i2c_Start();		

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_addr | I2C_WR);
 	if ( i2c_WaitAck() != 0 )
 		goto cmd_fail;
	
 	/* 发送pmbus命令 */
 	i2c_SendByte(_cmd);
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
			
	/* 重新发送启动信号 */
	i2c_Start();	
	/* 发送地址加读命令 */
	i2c_SendByte(_addr | I2C_RD);	
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
	/* 连续读取字节 */
	for ( idx = 0; idx < _size; idx++ )
	{
		_buf[idx] = i2c_ReadByte();
		
		if ( idx != (_size - 1) )
			i2c_Ack();
		else
			i2c_NAck();
	}
	
	i2c_Stop();
	
	/* 停止 */
	if ( get_i2c_timeout() )
		return SMBUS_TIMEOUT;
	else
		return SMBUS_OK;

cmd_fail: 
	i2c_Stop();
    smbus_delay_us( 1000 );
	
	if ( get_i2c_timeout() )
		return SMBUS_TIMEOUT;
	else
		return SMBUS_ERROR;		
}

static StatusTypeDef SMBUS_Master_Write_Multi_Byte(uint8_t _addr, uint8_t _cmd,uint8_t *_buf, uint8_t _size)
{
	uint8_t idx = 0;
	
	/* 发送启动信号 */
	i2c_Start();
	
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_addr | I2C_WR);
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
 	/* 发送pmbus命令 */
 	i2c_SendByte(_cmd);
	if (i2c_WaitAck() != 0)
		goto cmd_fail;
	
	// low byte first send!
	for ( idx = 0; idx < _size; idx ++ )
	{
		i2c_SendByte(_buf[idx]);
		if ( i2c_WaitAck() != 0 )
			goto cmd_fail;
	}
	
	i2c_Stop();

	if ( get_i2c_timeout() )
		return SMBUS_TIMEOUT;
	else
		return SMBUS_OK;

cmd_fail: 
	i2c_Stop();
    smbus_delay_us( 1000 );

	if ( get_i2c_timeout() )
		return SMBUS_TIMEOUT;
	else
		return SMBUS_ERROR;		
}

static StatusTypeDef SMBUS_Master_Chk_Addr_Dev(uint8_t _addr)
{
	uint8_t ucAck;
	
	/* 发送启动信号 */
	i2c_Start();
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_addr | I2C_WR);
	/* 检测设备的ACK应答 */
	ucAck = i2c_WaitAck();	
	
	/* 发送停止信号 */
	i2c_Stop();			
    smbus_delay_us( 1000 );
	
	if (ucAck == 0)
		return SMBUS_OK;
	else
		return SMBUS_ERROR;
}

StatusTypeDef SMBUS_Master_Transmit(SMBUS_StackHandleTypeDef *pStackContext)
{
	if ( (pStackContext->OpMode & READ) == READ)
		return SMBUS_Master_Read_Multi_Byte(pStackContext->SlaveAddress, \
   											pStackContext->CMD_table->cmnd_code,\
											pStackContext->Buffer, \
											pStackContext->Byte_count);

	else if ( (pStackContext->OpMode & WRITE) == WRITE)
		return SMBUS_Master_Write_Multi_Byte(pStackContext->SlaveAddress, \
   											pStackContext->CMD_table->cmnd_code, \
											pStackContext->Buffer, \
											pStackContext->Byte_count);
	else if ( (pStackContext->OpMode & CHK_ADDR_DEVICE) == CHK_ADDR_DEVICE)
	{
	   return SMBUS_Master_Chk_Addr_Dev( pStackContext->SlaveAddress );
	}
//	else if ( (pStackContext->OpMode & WRITE) == BLOCK_WRITE)
//	{
//		;
//	}
	return SMBUS_ERROR;
}

void Init_SMBus(void)
{
	i2c_gpio_init();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
