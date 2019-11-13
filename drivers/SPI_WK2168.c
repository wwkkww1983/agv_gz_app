/**
  ******************************************************************************
  * @file    SPI_WK2168.c
  * @author  June_Zu
  * @version V1.0.0
  * @date    25-Jan-2019
  * @brief   SPI_WK2168 file
  ******************************************************************************/
#include "SPI_WK2168.h"
#include "gpio.h"

SPI_InitTypeDef  SPI_InitStructure;

static void delay_ms_wk( __IO uint32_t _T )
{
	uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 10280; i++);
}


void SPI1_Init(SPI_TypeDef* SPIx)
{
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //ÉèÖÃSPIµ¥Ïò»òÕßË«ÏòµÄÊý¾ÝÄ£Ê½:SPIÉèÖÃÎªË«ÏßË«ÏòÈ«Ë«¹¤
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//ÉèÖÃSPI¹¤×÷Ä£Ê½:ÉèÖÃÎªÖ÷SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//ÉèÖÃSPIµÄÊý¾Ý´óÐ¡:SPI·¢ËÍ½ÓÊÕ8Î»Ö¡½á¹¹
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//Ñ¡ÔñÁË´®ÐÐÊ±ÖÓµÄÎÈÌ¬:Ê±ÖÓÐü¿ÕµÍ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//Êý¾Ý²¶»ñÓÚµÚ¶þ¸öÊ±ÖÓÑØ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSSÐÅºÅÓÉÓ²¼þ£¨NSS¹Ü½Å£©»¹ÊÇÈí¼þ£¨Ê¹ÓÃSSIÎ»£©¹ÜÀí:ÄÚ²¿NSSÐÅºÅÓÐSSIÎ»¿ØÖÆ
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//¶¨Òå²¨ÌØÂÊÔ¤·ÖÆµµÄÖµ:²¨ÌØÂÊÔ¤·ÖÆµÖµÎª256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//Ö¸¶¨Êý¾Ý´«Êä´ÓMSBÎ»»¹ÊÇLSBÎ»¿ªÊ¼:Êý¾Ý´«Êä´ÓMSBÎ»¿ªÊ¼
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCÖµ¼ÆËãµÄ¶àÏîÊ½
	SPI_Init(SPIx, &SPI_InitStructure);  //¸ù¾ÝSPI_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèSPIx¼Ä´æÆ÷
 
	SPI_Cmd(SPIx, ENABLE); //Ê¹ÄÜSPIÍâÉè
	
	SPI_ReadWriteByte(SPIx,0xff);//Æô¶¯´«Êä	
}   

void SPI1_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet)
{
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx,ENABLE);
} 

u8 SPI_ReadWriteByte(SPI_TypeDef* SPIx,u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) //¼ì²éÖ¸¶¨µÄSPI±êÖ¾Î»ÉèÖÃÓë·ñ:·¢ËÍ»º´æ¿Õ±êÖ¾Î»
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPIx, TxData); //Í¨¹ýÍâÉèSPIx·¢ËÍÒ»¸öÊý¾Ý
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)//¼ì²éÖ¸¶¨µÄSPI±êÖ¾Î»ÉèÖÃÓë·ñ:½ÓÊÜ»º´æ·Ç¿Õ±êÖ¾Î»
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPIx); //·µ»ØÍ¨¹ýSPIx×î½ü½ÓÊÕµÄÊý¾Ý					    
}

void SPI_CS_Init(SPI_TypeDef* SPIx)
{
	if(SPIx == SPI1)
   GPIO_SetBits(GPIOA,GPIO_Pin_4);						 //PA.4 Êä³ö¸ß	
	else
	 GPIO_SetBits(GPIOB,GPIO_Pin_12);						 //PA.4 Êä³ö¸ß	
}


void SPI_BUS_Init(SPI_TypeDef* SPIx)
{
	SPI1_Init(SPIx);		   //³õÊ¼»¯SPI
	SPI1_SetSpeed(SPIx,SPI_BaudRatePrescaler_8);	//ÉèÖÃÎª10MÊ±ÖÓ,¸ßËÙÄ£Ê½
}

void SPI_CS_H(SPI_TypeDef* SPIx)
{
	if(SPIx == SPI1)
   GPIO_SetBits(GPIOA,GPIO_Pin_4);						 //PA.4 Êä³ö¸ß	
	else
	 GPIO_SetBits(GPIOB,GPIO_Pin_12);						 //PA.4 Êä³ö¸ß	
}


void SPI_CS_L(SPI_TypeDef* SPIx)
{
	if(SPIx == SPI1)
   GPIO_ResetBits(GPIOA,GPIO_Pin_4);						 //PA.4 Êä³ö¸ß	
	else
	 GPIO_ResetBits(GPIOB,GPIO_Pin_12);						 //PA.4 Êä³ö¸ß	
}

void WK2XXX_SPI_Init(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

	SPI_CS_Init(SPI1);
	SPI_BUS_Init(SPI1);
	SPI_CS_Init(SPI2);
	SPI_BUS_Init(SPI2);
}

void Wk_Init(SPI_TypeDef* SPIx,unsigned char port)
{
	unsigned char gena,grst,gier,sier,scr;
	
	// 使能子串口时钟
	gena=WkReadGReg(SPIx,WK2XXX_GENA); 
	gena=gena|(1<<(port-1));
	WkWriteGReg(SPIx, WK2XXX_GENA,gena);
	//软件复位子串口
	grst=WkReadGReg(SPIx,WK2XXX_GRST); 
	grst=grst|(1<<(port-1));
	WkWriteGReg(SPIx, WK2XXX_GRST,grst);
	// 使能串口总中断
	gier=WkReadGReg(SPIx, WK2XXX_GIER);
	gier=gier|(1<<(port-1));
	WkWriteGReg(SPIx, WK2XXX_GIER,gier); 

	//使能子串口接收触点中断和超时中断
	sier=WkReadSReg(SPIx, port,WK2XXX_SIER); 
	sier |= WK2XXX_RFTRIG_IEN|WK2XXX_RXOVT_IEN;
	WkWriteSReg(SPIx, port,WK2XXX_SIER,sier);

	// 初始化fifo和设置固定中断触点
	WkWriteSReg(SPIx, port,WK2XXX_FCR,0XFF); 
	// 设置任意中断触点，如果下面的设置有效，
	// 那么上面FCR寄存器中断的固定中断触点将失效
	WkWriteSReg(SPIx,port,WK2XXX_SPAGE,1);// 切换到page1
	WkWriteSReg(SPIx,port,WK2XXX_RFTL,0x64);// 设置接收触点为100字节
	WkWriteSReg(SPIx,port,WK2XXX_TFTL,0X10);//ÉèÖÃ·¢ËÍ´¥µãÎª16¸ö×Ö½Ú
	WkWriteSReg(SPIx,port,WK2XXX_SPAGE,0);//切换到page0 

	// 使能子串口的发送和接收使能
	scr=WkReadSReg(SPIx,port,WK2XXX_SCR); 
	scr|=WK2XXX_TXEN|WK2XXX_RXEN;
	WkWriteSReg(SPIx,port,WK2XXX_SCR,scr);
}

/**************************************WK_RstInit***********************************/
//函数功能:wk芯片需要用MCU的GPIO去控制RST引脚
//*************************************************************************/
void WK_RstInit(void)
{
	GPIO_SetBits(SPI_EXT_1_RST_GPIO_PORT, SPI_EXT_1_RST_GPIO_PIN);    //SPI1连接的RST引脚输出高	
	delay_ms_wk(50);
	GPIO_ResetBits(SPI_EXT_1_RST_GPIO_PORT, SPI_EXT_1_RST_GPIO_PIN);  //SPI1连接的RST引脚输出低
	delay_ms_wk(10);
	GPIO_SetBits(SPI_EXT_1_RST_GPIO_PORT, SPI_EXT_1_RST_GPIO_PIN);    //SPI1连接的RST引脚输出高	
	delay_ms_wk(10);


	GPIO_SetBits(SPI_EXT_2_RST_GPIO_PORT, SPI_EXT_2_RST_GPIO_PIN);    //SPI2连接的RST引脚输出高	
	delay_ms_wk(50);
	GPIO_ResetBits(SPI_EXT_2_RST_GPIO_PORT, SPI_EXT_2_RST_GPIO_PIN);  //SPI2连接的RST引脚输出低
	delay_ms_wk(10);
	GPIO_SetBits(SPI_EXT_2_RST_GPIO_PORT, SPI_EXT_2_RST_GPIO_PIN);    //SPI2连接的RST引脚输出高	
	delay_ms_wk(10);
}

void WK_RS485_CS(SPI_TypeDef* SPIx,u8 port,u8 RTXFlag)
{
	if(RTXFlag == 1)
	{
		WkWriteSReg(SPIx,port,WK2XXX_RS485,0x02);//½ÓÊÕ
	}
	else
	{
		WkWriteSReg(SPIx,port,WK2XXX_RS485,0x03);//·¢ËÍ
	}
}

void WK2168_Sys_Init(void)
{
//	u8 dat1,dat2;
	// 初始化SPI接口
	WK2XXX_SPI_Init();
//	dat1=WkReadGReg(SPI1, WK2XXX_GENA);
//	dat2=WkReadGReg(SPI2, WK2XXX_GENA);
//	printf("\r\nspi1, gena=0x%x, spi2, gena=0x%x\n",dat1, dat2);
	
	// 硬件复位芯片
	WK_RstInit();
//	dat1=WkReadGReg(SPI1, WK2XXX_GENA);
//	dat2=WkReadGReg(SPI2, WK2XXX_GENA);
//	printf("\r\nspi1, gena=0x%x, spi2, gena=0x%x\n",dat1, dat2);
	
	// 初始化子串口
	Wk_Init(SPI1,1);
	Wk_Init(SPI1,2);
	Wk_Init(SPI1,3);
	Wk_Init(SPI1,4);
	
	Wk_Init(SPI2,1);
	Wk_Init(SPI2,2);
	Wk_Init(SPI2,3);
	Wk_Init(SPI2,4);	
	
	// 设置子串口波特率
	Wk_SetBaud(SPI1, 1, B9600);
	Wk_SetBaud(SPI1, 2, B9600);
	Wk_SetBaud(SPI1, 3, B9600);
	Wk_SetBaud(SPI1, 4, B9600);
	
	Wk_SetBaud(SPI2, 1, B9600);
	Wk_SetBaud(SPI2, 2, B9600);
	Wk_SetBaud(SPI2, 3, B9600);
	Wk_SetBaud(SPI2, 4, B9600);	
	
	// 初始化485
	// 注：485初始化必须放在最后（因为必须先使能GENA，才能操作子串口相关的寄存器）
	WK_RS485(SPI1,1);
	WK_RS485(SPI1,2);
	WK_RS485(SPI1,3);
	WK_RS485(SPI1,4);
	
	WK_RS485(SPI2,1);
	WK_RS485(SPI2,2);
	WK_RS485(SPI2,3);
	WK_RS485(SPI2,4);
}


/***************************WkWriteGReg***********************************/
//函数功能：写全局寄存器函数（前提是该寄存器可写，
//某些寄存器如果你写1，可能会自动置1，具体见数据手册)
//参数：
//      SPIx:芯片连接的SPI接口
//      greg:为全局寄存器的地址
//      dat:为写入寄存器的数据
//***********************************************************************/
void WkWriteGReg(SPI_TypeDef* SPIx, unsigned char greg,unsigned char dat)
{	 u8 cmd;
	 cmd=0|greg;
	 SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
	 SPI_ReadWriteByte(SPIx, cmd);	//Ð´Ö¸Áî£¬¶ÔÓÚÖ¸ÁîµÄ¹¹³É¼ûÊý¾ÝÊÖ²á
	 SPI_ReadWriteByte(SPIx, dat);//Ð´Êý¾Ý
   SPI_CS_H(SPIx);//À­¸ßcsÐÅºÅ
}
/****************************WkReadGReg***********************************/
//函数功能：读全局寄存器
//参数：
//      SPIx:芯片连接的SPI接口
//      greg:为全局寄存器的地址
//      rec:返回的寄存器值
//***********************************************************************/
u8 WkReadGReg(SPI_TypeDef* SPIx, unsigned char greg)
{	 u8 cmd,rec;
	 cmd=0x40|greg;
	 SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
	 SPI_ReadWriteByte(SPIx, cmd);	//Ð´Ö¸Áî£¬¶ÔÓÚÖ¸ÁîµÄ¹¹³É¼ûÊý¾ÝÊÖ²á
	 rec=SPI_ReadWriteByte(SPIx, 0);//Ð´Êý¾Ý
     SPI_CS_H( SPIx );//À­¸ßcsÐÅºÅ							
	 return rec;
}

/**************************WkWriteSReg***********************************/
//函数功能:
//参数：
//      SPIx:芯片连接的SPI接口
//      port:为子串口
//      sreg:为子串口寄存器
//      dat:为写入寄存器的数据
//注意：在子串口被打通的情况下，向FDAT写入的数据会通过TX引脚输出
//**********************************************************************/
void WkWriteSReg(SPI_TypeDef* SPIx, u8 port,u8 sreg,u8 dat)
{	 u8 cmd;
	 cmd=0x0|((port-1)<<4)|sreg;
	 SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
	 SPI_ReadWriteByte(SPIx, cmd);	//Ð´Ö¸Áî£¬¶ÔÓÚÖ¸ÁîµÄ¹¹³É¼ûÊý¾ÝÊÖ²á
	 SPI_ReadWriteByte(SPIx, dat);//Ð´Êý¾Ý
   SPI_CS_H( SPIx );//À­¸ßcsÐÅºÅ
}

/**************************WkReadSReg***********************************/
//函数功能：读子串口寄存器
//参数：
//      SPIx:芯片连接的SPI接口
//      sreg:为子串口寄存器地址
//      rec:返回的寄存器值
//**********************************************************************/
u8 WkReadSReg(SPI_TypeDef* SPIx, u8 port,u8 sreg)
{	 u8 cmd,rec;
	 cmd=0x40|((port-1)<<4)|sreg;
	 SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
	 SPI_ReadWriteByte(SPIx, cmd);	//Ð´Ö¸Áî£¬¶ÔÓÚÖ¸ÁîµÄ¹¹³É¼ûÊý¾ÝÊÖ²á
	 rec=SPI_ReadWriteByte(SPIx, 0);//Ð´Êý¾Ý
   SPI_CS_H( SPIx );	//À­¸ßcsÐÅºÅ
	 return rec;
}
/************************WkWriteSFifo***********************************/
//函数功能:向子串口fifo写入需要发送的数据
//参数：
//      SPIx:芯片连接的SPI接口
//      port:为子串口
//      *dat:写入数据
//      num：为写入数据的个数，单次不超过256
//注意：通过该方式写入的数据，被直接写入子串口的缓存FIFO，然后被发送
//*********************************************************************/
void WkWriteSFifo(SPI_TypeDef* SPIx, u8 port,u8 *dat,int num)
{
	 u8 cmd;
	 int i;
	 cmd=0x80|((port-1)<<4);
	 if(num>0)
	 {
			 SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
			 SPI_ReadWriteByte(SPIx, cmd); //Ð´Ö¸Áî,¶ÔÓÚÖ¸Áî¹¹³É¼ûÊý¾ÝÊÖ²á
			 for(i=0;i<num;i++)
			 {
				 SPI_ReadWriteByte(SPIx,  *(dat+i));//Ð´Êý¾Ý
			 }
			 SPI_CS_H( SPIx );//À­¸ßcsÐÅºÅ
   }
}

/************************WkReadSFifo***********************************/
//函数功能:从子串口的fifo中读出接收到的数据
//参数：
//      SPIx:芯片连接的SPI接口
//      port:为子串口
//      *rec：接收到的数据
//      num：读出的数据个数。
//注意：通过该方式读出子串口缓存中的数据。单次不能超过256
//*********************************************************************/
void WkReadSFifo(SPI_TypeDef* SPIx, u8 port,u8 *rec,int num)
{
		u8 cmd;
		int n;
	  cmd=0xc0|((port-1)<<4);
		if(num>0)
		{
				SPI_CS_L( SPIx );//À­µÍcsÐÅºÅ
				SPI_ReadWriteByte(SPIx, cmd);
				for(n=0;n<num;n++)
				{
					*(rec+n)=SPI_ReadWriteByte(SPIx, 0);
				}
				SPI_CS_H( SPIx );//À­¸ßcsÐÅºÅ
	  }
}

/**************************Wk_SetBaud*******************************************************/
//函数功能：设置子串口波特率函数、此函数中波特率的匹配值是根据11.0592Mhz下的外部晶振计算的
// SPIx:芯片连接的SPI接口
// port:子串口号
// baud:波特率大小.波特率表示方式，
/**************************Wk2114SetBaud*******************************************************/
void Wk_SetBaud(SPI_TypeDef* SPIx, u8 port,enum WKBaud baud)
{  
	unsigned char baud1,baud0,pres,scr;
	//如下波特率相应的寄存器值，是在外部时钟为11.0592mhz的情况下计算所得，如果使用其他晶振，需要重新计算
	switch (baud) 
		{
				case B600:
					baud1=0x4;
					baud0=0x7f;
					pres=0;
					break;
				case B1200:
					baud1=0x2;
					baud0=0x3F;
					pres=0;
					break;
				case B2400:
					baud1=0x1;
					baud0=0x1f;
					pres=0;
					break;
				case B4800:
					baud1=0x00;
					baud0=0x8f;
					pres=0;
					break;
				case B9600:
					baud1=0x00;
					baud0=0x47;
					pres=0;
					break;
				case B19200:
					baud1=0x00;
					baud0=0x23;
					pres=0;
					break;
				case B38400:
					baud1=0x00;
					baud0=0x11;
					pres=0;
					break;
				case B76800:
					baud1=0x00;
					baud0=0x08;
					pres=0;
					break;        
				case B1800:
					baud1=0x01;
					baud0=0x7f;
					pres=0;
					break;
				case B3600:
					baud1=0x00;
					baud0=0xbf;
					pres=0;
					break;
				case B7200:
					baud1=0x00;
					baud0=0x5f;
					pres=0;
					break;
				case B14400:
					baud1=0x00;
					baud0=0x2f;
					pres=0;
					break;
				case B28800:
					baud1=0x00;
					baud0=0x17;
					pres=0;
					break;
				case B57600:
					baud1=0x00;
					baud0=0x0b;
					pres=0;
					break;
				case B115200:
					baud1=0x00;
					baud0=0x05;
					pres=0;
					break;
				case B230400:
					baud1=0x00;
					baud0=0x02;
					pres=0;
					break;
				default:
					baud1=0x00;
					baud0=0x00;
					pres=0;
    }
		//关掉子串口收发使能
		scr=WkReadSReg(SPIx, port,WK2XXX_SCR); 
		WkWriteSReg(SPIx, port,WK2XXX_SCR,0);
		//设置波特率相关寄存器
		WkWriteSReg(SPIx, port,WK2XXX_SPAGE,1);//切换到page1
		WkWriteSReg(SPIx, port,WK2XXX_BAUD1,baud1);
		WkWriteSReg(SPIx, port,WK2XXX_BAUD0,baud0);
		WkWriteSReg(SPIx, port,WK2XXX_PRES,pres);
		WkWriteSReg(SPIx, port,WK2XXX_SPAGE,0);//切换到page0 
		//使能子串口收发使能
		WkWriteSReg(SPIx, port,WK2XXX_SCR,scr);
}

/**************************WK_TxLen*******************************************/
//函数功能:获取子串口发送FIFO剩余空间长度
// SPIx:芯片连接的SPI接口
// port:端口号
// 返回值：发送FIFO剩余空间长度
/**************************WK_Len********************************************/
int wk_TxLen(SPI_TypeDef* SPIx, u8 port)
{
	u8 fsr,tfcnt;
	int len=0;
	fsr  =WkReadSReg(SPIx, port,WK2XXX_FSR);
	tfcnt=WkReadSReg(SPIx, port,WK2XXX_TFCNT);
	if(fsr& WK2XXX_TFULL)
		{ len=0;}
	else
	  {len=256-tfcnt;}
  return len;
}

/**************************WK_TxChars*******************************************/
//函数功能:通过子串口发送固定长度数据
// SPIx:芯片连接的SPI接口
// port:端口号
// len:单次发送长度不超过256
// 
/**************************WK_TxChars********************************************/
int wk_TxChars(SPI_TypeDef* SPIx, u8 port,int len,u8 *sendbuf)
{

#if 1
	WkWriteSFifo(SPIx, port,sendbuf,len);//Í¨¹ýfifo·½Ê½·¢ËÍÊý¾Ý
#else
	int num=len;
	for(num=0;num<len;num++)
	{
		WkWriteSReg(SPIx, port,WK2XXX_FDAT,*(sendbuf+num));
	}
#endif
}


/**************************WK_RxChars*******************************************/
//函数功能:读取子串口fifo中的数据
// SPIx:芯片连接的SPI接口
// port:端口号
// recbuf:接收到的数据
// 返回值：接收数据的长度
/**************************WK_RxChars********************************************/
int wk_RxChars(SPI_TypeDef* SPIx, u8 port,u8 *recbuf)
{
	u8  fsr=0,rfcnt=0,sifr=0;
	int len=0;
	sifr=WkReadSReg(SPIx, port,WK2XXX_SIFR);
	
	if((sifr&WK2XXX_RFTRIG_INT)||(sifr&WK2XXX_RXOVT_INT))//有接收中断和接收超时中断
	{ 
	fsr  =WkReadSReg(SPIx, port,WK2XXX_FSR);
	rfcnt=WkReadSReg(SPIx, port,WK2XXX_RFCNT);
	//printf("rfcnt=0x%x.\n",rfcnt);
	/*判断fifo中数据个数*/
	if(fsr& WK2XXX_RDAT)
	{ 
		len=(rfcnt==0)?256:rfcnt;
	}
#if 1
  WkReadSFifo(SPIx, port,recbuf,len);
#else
	for(n=0;n<len;n++)
	 *(recbuf+n)=WkReadSReg(SPIx, port,WK2XXX_FDAT);
#endif	
	return len;
  }
	else
	{
		len=0;
		return len;
	}
}

/**************************WK_RS485*******************************************************/
//函数功能:设置子串口RS485的收发转换函数，使用RTS引脚控制485电平转换芯片的收发
// SPIx:芯片连接的SPI接口
// port:子串口号
//注意：只有WK2168/WK2204支持该功能
/**************************WK_RS485*******************************************************/
void WK_RS485(SPI_TypeDef* SPIx, u8 port)
{   
//	unsigned char reg = 0;

	WkWriteSReg(SPIx, port,WK2XXX_RS485,0x02); // 默认rts输出高电平
//	WkWriteSReg(port,WK2XXX_RS485,0x03);
//	reg = WkReadSReg( SPIx, port, WK2XXX_RS485 );
//	printf("\r\nport:%d, 0x%02x", port, reg);	

	WkWriteSReg(SPIx, port,WK2XXX_SPAGE,1);//切换到page1
	WkWriteSReg(SPIx, port,WK2XXX_RTSDLY,0X10);
	WkWriteSReg(SPIx, port,WK2XXX_SPAGE,0);//切换到page0 
}

/**************************WK_RTSCTS*******************************************************/
//函数功能:硬件自动流量控制，需要子设备的支持
// SPIx:芯片连接的SPI接口
// port:子串口号
// 
//注意：只有WK2168/WK2204支持该功能
/**************************WK_RTSCTS*******************************************************/
void WK_RTSCTS(SPI_TypeDef* SPIx, u8 port)
{   
	  WkWriteSReg(SPIx, port,WK2XXX_FWCR,0x30);//
		WkWriteSReg(SPIx, port,WK2XXX_SPAGE,1);//切换到page1
		WkWriteSReg(SPIx, port,WK2XXX_FWTH,0XF0);//停止接收触点
		WkWriteSReg(SPIx, port,WK2XXX_FWTL,0X20);//继续接收触点
		WkWriteSReg(SPIx, port,WK2XXX_SPAGE,0);//切换到page0 
}

/**************************WK_IrqApp*******************************************/
//函数功能:中断处理的一般流程，可以根据需要修改
// 
// 
// 
/**************************WK_IrqApp********************************************/
//u8 WK_IrqApp(void)
//{
//	u8 gifr,sier,sifr;
//	gifr=WkReadGReg(WK2XXX_GIFR);
//	if(gifr&WK2XXX_UT1INT)//判断子串口1是否有中断
//	{ 
//		sifr =WkReadSReg(1,WK2XXX_SIFR);
//		if()
//	}
//	if(gifr&WK2XXX_UT2INT)//判断子串口2是否有中断
//	{
//		
//	}
//	if(gifr&WK2XXX_UT3INT)//判断子串口3是否有中断
//	{
//		
//	}
//	if(gifr&WK2XXX_UT4INT)//判断子串口4是否有中断
//	{
//		
//	}
//	
//	
//}




/*********************************************END OF FILE**********************/
