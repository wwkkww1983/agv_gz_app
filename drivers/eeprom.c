
#include "eeprom.h"
#include "stdio.h"


#define EEPROM_I2C_WR	0		/* д����bit */
#define EEPROM_I2C_RD	1		/* ������bit */



/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define EEPROM_GPIO_PORT_I2C	GPIOB			/* GPIO�˿� */
#define EEPROM_RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define EEPROM_I2C_SCL_PIN		GPIO_Pin_10			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define EEPROM_I2C_SDA_PIN		GPIO_Pin_11			/* ���ӵ�SDA�����ߵ�GPIO */

/* �����дSCL��SDA�ĺ꣬�����Ӵ���Ŀ���ֲ�ԺͿ��Ķ��� */
#if 0	/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */
	#define EEPROM_I2C_SCL_1()  GPIO_SetBits(EEPROM_GPIO_PORT_I2C, EEPROM_I2C_SCL_PIN)		/* SCL = 1 */
	#define EEPROM_I2C_SCL_0()  GPIO_ResetBits(EEPROM_GPIO_PORT_I2C, EEPROM_I2C_SCL_PIN)		/* SCL = 0 */
	
	#define EEPROM_I2C_SDA_1()  GPIO_SetBits(EEPROM_GPIO_PORT_I2C, EEPROM_I2C_SDA_PIN)		/* SDA = 1 */
	#define EEPROM_I2C_SDA_0()  GPIO_ResetBits(EEPROM_GPIO_PORT_I2C, EEPROM_I2C_SDA_PIN)		/* SDA = 0 */
	
	#define EEPROM_I2C_SDA_READ()  GPIO_ReadInputDataBit(EEPROM_GPIO_PORT_I2C, EEPROM_I2C_SDA_PIN)	/* ��SDA����״̬ */
#else	/* �����֧ѡ��ֱ�ӼĴ�������ʵ��IO��д */
    /*��ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
	#define EEPROM_I2C_SCL_1()  EEPROM_GPIO_PORT_I2C->BSRR = EEPROM_I2C_SCL_PIN				/* SCL = 1 */
	#define EEPROM_I2C_SCL_0()  EEPROM_GPIO_PORT_I2C->BRR = EEPROM_I2C_SCL_PIN				/* SCL = 0 */
	
	#define EEPROM_I2C_SDA_1()  EEPROM_GPIO_PORT_I2C->BSRR = EEPROM_I2C_SDA_PIN				/* SDA = 1 */
	#define EEPROM_I2C_SDA_0()  EEPROM_GPIO_PORT_I2C->BRR = EEPROM_I2C_SDA_PIN				/* SDA = 0 */
	
	#define EEPROM_I2C_SDA_READ()  ((EEPROM_GPIO_PORT_I2C->IDR & EEPROM_I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#endif


#define EEPROM_DEV_ADDR			0xA0			/* 24LC128T���豸��ַ */
#define EEPROM_PAGE_SIZE		64			 	/* 24LC128T��ҳ���С */
#define EEPROM_SIZE				128		 		/* 24LC128T������,16K x 8 (128Kbit)��ʵ���ò�����ô�࣬��ʱȡ128��byte */

/* Private functions ---------------------------------------------------------*/
static uint8_t ee_i2c_chk_device(uint8_t _Address);

/* Public functions ---------------------------------------------------------*/
static void i2c_config_gpio(void);

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_delay
*	����˵��: I2C����λ�ӳ٣����400KHz
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ee_i2c_delay(void)
{
	uint8_t i;

	/*��
	 	�����ʱ����ͨ���߼������ǲ��Եõ��ġ�
    ����������CPU��Ƶ72MHz ��MDK���뻷����1���Ż�
  
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
	*/
	for (i = 0; i < 10; i++);
}

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_start
*	����˵��: CPU����I2C���������ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ee_i2c_start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
	EEPROM_I2C_SDA_1();
	EEPROM_I2C_SCL_1();
	ee_i2c_delay();
	EEPROM_I2C_SDA_0();
	ee_i2c_delay();
	EEPROM_I2C_SCL_0();
	ee_i2c_delay();
}

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_start
*	����˵��: CPU����I2C����ֹͣ�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ee_i2c_stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	EEPROM_I2C_SDA_0();
	EEPROM_I2C_SCL_1();
	ee_i2c_delay();
	EEPROM_I2C_SDA_1();
}

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_send_byte
*	����˵��: CPU��I2C�����豸����8bit����
*	��    �Σ�_ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ee_i2c_send_byte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			EEPROM_I2C_SDA_1();
		}
		else
		{
			EEPROM_I2C_SDA_0();
		}
		ee_i2c_delay();
		EEPROM_I2C_SCL_1();
		ee_i2c_delay();	
		EEPROM_I2C_SCL_0();
		if (i == 7)
		{
			 EEPROM_I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */
		ee_i2c_delay();
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
static uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		EEPROM_I2C_SCL_1();
		ee_i2c_delay();
		if (EEPROM_I2C_SDA_READ())
		{
			value++;
		}
		EEPROM_I2C_SCL_0();
		ee_i2c_delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_wait_ack
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    �Σ���
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
static uint8_t ee_i2c_wait_ack(void)
{
	uint8_t re;

	EEPROM_I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	ee_i2c_delay();
	EEPROM_I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	ee_i2c_delay();
	if (EEPROM_I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	EEPROM_I2C_SCL_0();
	ee_i2c_delay();
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
static void i2c_Ack(void)
{
	EEPROM_I2C_SDA_0();	/* CPU����SDA = 0 */
	ee_i2c_delay();
	EEPROM_I2C_SCL_1();	/* CPU����1��ʱ�� */
	ee_i2c_delay();
	EEPROM_I2C_SCL_0();
	ee_i2c_delay();
	EEPROM_I2C_SDA_1();	/* CPU�ͷ�SDA���� */
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_NAck(void)
{
	EEPROM_I2C_SDA_1();	/* CPU����SDA = 1 */
	ee_i2c_delay();
	EEPROM_I2C_SCL_1();	/* CPU����1��ʱ�� */
	ee_i2c_delay();
	EEPROM_I2C_SCL_0();
	ee_i2c_delay();	
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_CfgGpio
*	����˵��: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_config_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(EEPROM_RCC_I2C_PORT, ENABLE);	/* ��GPIOʱ�� */

	GPIO_InitStructure.GPIO_Pin = EEPROM_I2C_SCL_PIN | EEPROM_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* ��©��� */
	GPIO_Init(EEPROM_GPIO_PORT_I2C, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: ee_i2c_chk_device
*	����˵��: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
*	��    �Σ�_Address���豸��I2C���ߵ�ַ
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
*********************************************************************************************************
*/
static uint8_t ee_i2c_chk_device(uint8_t _Address)
{
	uint8_t ucAck;
	
	ee_i2c_start();		/* ���������ź� */

	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	ee_i2c_send_byte(_Address | EEPROM_I2C_WR);
	ucAck = ee_i2c_wait_ack();	/* ����豸��ACKӦ�� */

	ee_i2c_stop();			/* ����ֹͣ�ź� */

	return ucAck;
}

void eeprom_init(void)
{
	i2c_config_gpio();
	
	/* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
	ee_i2c_stop();
}


/* Public functions ---------------------------------------------------------*/

/*
*********************************************************************************************************
*	�� �� ��: ee_check_ok
*	����˵��: �жϴ���EERPOM�Ƿ�����
*	��    �Σ���
*	�� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
*********************************************************************************************************
*/
uint8_t ee_check_ok(void)
{
	if (ee_i2c_chk_device(EEPROM_DEV_ADDR) == 0)
	{
		return 1;
	}
	else
	{
		/* ʧ�ܺ��мǷ���I2C����ֹͣ�ź� */
		ee_i2c_stop();		
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ee_ReadBytes
*	����˵��: �Ӵ���EEPROMָ����ַ����ʼ��ȡ��������
*	��    �Σ�_usAddress : ��ʼ��ַ
*			 _usSize : ���ݳ��ȣ���λΪ�ֽ�
*			 _pReadBuf : ��Ŷ��������ݵĻ�����ָ��
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;
	
	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */
	/* ��1��������I2C���������ź� */
	ee_i2c_start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	ee_i2c_send_byte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* �˴���дָ�� */
	if (ee_i2c_wait_ack() != 0)
		goto cmd_fail;	/* EEPROM������Ӧ�� */

	/* ��3���������ֽڵ�ַ */
	/* address high byte */
	ee_i2c_send_byte((uint8_t)(_usAddress>>8));
	if (ee_i2c_wait_ack() != 0)
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	/* address low byte */
	ee_i2c_send_byte((uint8_t)_usAddress);
	if (ee_i2c_wait_ack() != 0)
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	
	
	/* ��4������������I2C���ߡ�ǰ��Ĵ����Ŀ����EEPROM���͵�ַ�����濪ʼ��ȡ���� */
	ee_i2c_start();
	
	/* ��5������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	ee_i2c_send_byte(EEPROM_DEV_ADDR | EEPROM_I2C_RD);	/* �˴��Ƕ�ָ�� */
	if (ee_i2c_wait_ack() != 0)
		goto cmd_fail;	/* EEPROM������Ӧ�� */	
	
	/* ��6����ѭ����ȡ���� */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = i2c_ReadByte();	/* ��1���ֽ� */
		
		/* ÿ����1���ֽں���Ҫ����Ack�� ���һ���ֽڲ���ҪAck����Nack */
		if (i != _usSize - 1)
			i2c_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */
		else
			i2c_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
	}
	
	/* ����I2C����ֹͣ�ź� */
	ee_i2c_stop();
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	ee_i2c_stop();
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: ee_WriteBytes
*	����˵��: ����EEPROMָ����ַд���������ݣ�����ҳд�������д��Ч��
*	��    �Σ�_usAddress : ��ʼ��ַ
*			 _usSize : ���ݳ��ȣ���λΪ�ֽ�
*			 _pWriteBuf : ����Ҫд�����ݵĻ�����ָ��
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i,m;
	uint16_t usAddr;
	/* 
		д����EEPROM�������������������ȡ�ܶ��ֽڣ�ÿ��д����ֻ����ͬһ��page��
		����24xx02��page size = 8
		24LC128T, page size = 64
		�򵥵Ĵ�����Ϊ�����ֽ�д����ģʽ��ûд1���ֽڣ������͵�ַ
		Ϊ���������д��Ч��: ����������page wirte������
	*/

	usAddr = _usAddress;	
	for (i = 0; i < _usSize; i++)
	{
		/* �����͵�1���ֽڻ���ҳ���׵�ַʱ����Ҫ���·��������źź͵�ַ */
		if ((i == 0) || (usAddr & (EEPROM_PAGE_SIZE - 1)) == 0)
		{
			/*���ڣ�������ֹͣ�źţ������ڲ�д������*/
			ee_i2c_stop();
			
			/* ͨ���������Ӧ��ķ�ʽ���ж��ڲ�д�����Ƿ����, һ��С�� 10ms 			
				CLKƵ��Ϊ200KHzʱ����ѯ����Ϊ30������
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* ��1��������I2C���������ź� */
				ee_i2c_start();
				
				/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
				ee_i2c_send_byte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* �˴���дָ�� */
				
				/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
				if (ee_i2c_wait_ack() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM����д��ʱ */
			}
		
			/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
			/* address high byte */
			ee_i2c_send_byte((uint8_t)(usAddr>>8));
			if (ee_i2c_wait_ack() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
			/* address low byte */
			ee_i2c_send_byte((uint8_t)usAddr);
			if (ee_i2c_wait_ack() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
		}
	
		/* ��5������ʼд������ */
		ee_i2c_send_byte(_pWriteBuf[i]);
	
		/* ��6��������ACK */
		if (ee_i2c_wait_ack() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}

		usAddr++;	/* ��ַ��1 */		
	}
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	ee_i2c_stop();
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	ee_i2c_stop();
	return 0;
}


void ee_Erase(void)
{
	uint16_t i;
	uint8_t buf[EEPROM_SIZE];
	
	/* ��仺���� */
	for (i = 0; i < EEPROM_SIZE; i++)
	{
		buf[i] = 0xFF;
	}
	
	/* дEEPROM, ��ʼ��ַ = 0�����ݳ���Ϊ 256 */
	if (ee_WriteBytes(buf, 0, EEPROM_SIZE) == 0)
	{
		printf("����eeprom����\r\n");
		return;
	}
	else
	{
		printf("����eeprom�ɹ���\r\n");
	}
}


/*--------------------------------------------------------------------------------------------------*/
static void ee_Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}


/*
 * eeprom AT24C02 ��д����
 * ��������1���쳣����0
 */
uint8_t ee_Test(void) 
{
  uint32_t i;
	uint8_t write_buf[EEPROM_SIZE];
  uint8_t read_buf[EEPROM_SIZE];
  
/*-----------------------------------------------------------------------------------*/  
  if (ee_check_ok() == 0)
	{
		/* û�м�⵽EEPROM */
		printf("û�м�⵽����EEPROM!\r\n");
				
		return 0;
	}
/*------------------------------------------------------------------------------------*/  
  /* �����Ի����� */
	for (i = 0; i < EEPROM_SIZE; i++)
	{		
		write_buf[i] = i;
	}
/*------------------------------------------------------------------------------------*/  
  if (ee_WriteBytes(write_buf, 0, EEPROM_SIZE) == 0)
	{
		printf("дeeprom����\r\n");
		return 0;
	}
	else
	{		
		printf("дeeprom�ɹ���\r\n");
	}
  
  /*д��֮����Ҫ�ʵ�����ʱ��ȥ������Ȼ�����*/
  ee_Delay(0x0FFFFF);
/*-----------------------------------------------------------------------------------*/
  if (ee_ReadBytes(read_buf, 0, EEPROM_SIZE) == 0)
	{
		printf("��eeprom����\r\n");
		return 0;
	}
	else
	{		
		printf("��eeprom�ɹ����������£�\r\n");
	}
/*-----------------------------------------------------------------------------------*/  
  for (i = 0; i < EEPROM_SIZE; i++)
	{
		if(read_buf[i] != write_buf[i])
		{
			printf("0x%02X ", read_buf[i]);
			printf("����:EEPROM������д������ݲ�һ��");
			return 0;
		}
    printf(" %02X", read_buf[i]);
		
		if ((i & 15) == 15)
		{
			printf("\r\n");	
		}		
	}
  printf("eeprom��д���Գɹ�\r\n");
  return 1;
}
/*********************************************END OF FILE**********************/



