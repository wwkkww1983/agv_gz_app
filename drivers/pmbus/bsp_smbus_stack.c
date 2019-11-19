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
#define		TIME_OUT_25MS	200000  //25ms ������

#define		TIME_OUT	TIME_OUT_25MS

#define		TRUE	1
#define		FALSE	0

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */


/* ����I2C�������ӵ�GPIO�˿� */
#define GPIO_PORT_I2C	GPIOB						/* GPIO�˿� */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define I2C_SCL_PIN		GPIO_Pin_6					/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7					/* ���ӵ�SDA�����ߵ�GPIO */


/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */
#if 0	
	#define I2C_SCL_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* ��SDA����״̬ */

/* �����֧ѡ��ֱ�ӼĴ�������ʵ��IO��д */
/*ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
#else
void 		I2C_SCL_1(void); // ����clock low extending ������	 
//	#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
	#define I2C_SCL_READ()  ((GPIO_PORT_I2C->IDR & I2C_SCL_PIN) != 0)	/* ��SCL����״̬ */
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

/* ��ʾi2cͨѶ�Ƿ�ʱ */
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

/* �򵥵���ʱ */
static void smbus_delay_us( __IO uint32_t _T )
{
	uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 11; i++);
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Delay
*	����˵��: I2C����λ�ӳ٣����400KHz
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
    ����������CPU��Ƶ72MHz ��MDK���뻷����1���Ż�
  
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
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
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C���������ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
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
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C����ֹͣ�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_SendByte
*	����˵��: CPU��I2C�����豸����8bit����
*	��    �Σ�_ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
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
			 I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_ReadByte
*	����˵��: CPU��I2C�����豸��ȡ8bit����
*	��    �Σ���
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
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
*	�� �� ��: i2c_WaitAck
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    �Σ���
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;
	
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay();

	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
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
*	�� �� ��: i2c_Ack
*	����˵��: CPU����һ��ACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_CfgGpio
*	����˵��: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_CfgGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* ��GPIOʱ�� */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* ��©��� */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	/* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
	i2c_Stop();
}



/*
*********************************************************************************************************
*	�� �� ��: i2c_CheckDevice
*	����˵��: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
*	��    �Σ�_Address���豸��I2C���ߵ�ַ
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;
	
	i2c_Start();		/* ���������ź� */

	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(_Address | I2C_WR);
	ucAck = i2c_WaitAck();	/* ����豸��ACKӦ�� */

	i2c_Stop();			/* ����ֹͣ�ź� */

	// ucAck = 0����ʾ�豸Ӧ��
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
			
			GPIO_PORT_I2C->BSRR = I2C_SCL_PIN; // ����SCL
			smbus_delay_us(5);
		} while ( !I2C_SCL_READ() ); // �ж�SCL�Ƿ�����
	}
	
	return;

cmd_fail: 
//	i2c_Stop();  // ���ܼӣ���Ȼ���ܻᵼ����ѭ��
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
	
	/* ���������ź� */
	i2c_Start();		

	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(_addr | I2C_WR);
 	if ( i2c_WaitAck() != 0 )
 		goto cmd_fail;
	
 	/* ����pmbus���� */
 	i2c_SendByte(_cmd);
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
			
	/* ���·��������ź� */
	i2c_Start();	
	/* ���͵�ַ�Ӷ����� */
	i2c_SendByte(_addr | I2C_RD);	
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
	/* ������ȡ�ֽ� */
	for ( idx = 0; idx < _size; idx++ )
	{
		_buf[idx] = i2c_ReadByte();
		
		if ( idx != (_size - 1) )
			i2c_Ack();
		else
			i2c_NAck();
	}
	
	i2c_Stop();
	
	/* ֹͣ */
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
	
	/* ���������ź� */
	i2c_Start();
	
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(_addr | I2C_WR);
	if ( i2c_WaitAck() != 0 )
		goto cmd_fail;
	
 	/* ����pmbus���� */
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
	
	/* ���������ź� */
	i2c_Start();
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(_addr | I2C_WR);
	/* ����豸��ACKӦ�� */
	ucAck = i2c_WaitAck();	
	
	/* ����ֹͣ�ź� */
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
