//头文件
#include "stm32f10x.h" 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"


#define DATALEN 200

char USART1_RX_BUF[DATALEN];
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART1_RX_STA=0;       //接收状态标记



char USART2_RX_BUF[DATALEN];
u16 USART2_RX_STA=0;

char WIFI_DATA[16];

//仿原子延时，不进入systic中断
void delay_us(u32 nus)
{
 u32 temp;
 SysTick->LOAD = 9*nus;
 SysTick->VAL=0X00;//清空计数器
 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
 do
 {
  temp=SysTick->CTRL;//读取当前倒计数值
 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
     SysTick->CTRL=0x00; //关闭计数器
    SysTick->VAL =0X00; //清空计数器
}
void delay_ms(u16 nms)
{
 u32 temp;
 SysTick->LOAD = 9000*nms;
 SysTick->VAL=0X00;//清空计数器
 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
 do
 {
  temp=SysTick->CTRL;//读取当前倒计数值
 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
    SysTick->CTRL=0x00; //关闭计数器
    SysTick->VAL =0X00; //清空计数器
}



void Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB 
		| RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	

	//LED灯初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//USART相关初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1,ENABLE);
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //设置NVIC中断分组2:2位抢占优先级，2位响应优先级   0-3;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	//esp8266接     ch_pd接vcc 3.3v
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    //PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //设置NVIC中断分组2:2位抢占优先级，2位响应优先级   0-3;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
	
	
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 199;
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_Cmd(TIM3,ENABLE);
	
}


void USART1_Send_Data(char *buf,u16 len)
{
	u16 t;
	for(t=0; t<len ;++t)
	{
		USART_SendData(USART1,buf[t]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET );
	}
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET );
}

void USART2_Send_Data(char *buf,u16 len)
{
	u16 t;
	for(t=0; t<len ;++t)
	{
		USART_SendData(USART2,buf[t]);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET );
	}
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET );
}

void InitWifi()
{
	char str[] = "AT+CWJAP=\"chan\",\"chenbbb833\"\r\n";
	char str2[] =  "AT+CIPMUX=1\r\n";
	char str3[] = "AT+CIPSERVER=1,8080\r\n";
	int len = sizeof(str)/sizeof(str[0]);
	int len2 = sizeof(str2)/sizeof(str2[0]);
	int len3 = sizeof(str3)/sizeof(str3[0]);
	USART2_Send_Data(str,len);
	delay_ms(1000);
	USART2_Send_Data(str2,len2);
	delay_ms(1000);
	USART2_Send_Data(str3,len3);
	
}


void getWifiData()
{
	u16 len=USART2_RX_STA&0x3fff;
	int i,j,k,x;
	for(i=0; i<len; ++i)
	{
		//判断是否+IPD开头
		if( USART2_RX_BUF[i] == 0x2B && USART2_RX_BUF[i+1] == 0x49
			&& USART2_RX_BUF[i+2] == 0x50 && USART2_RX_BUF[i+3] == 0x44)
		{
			for(j=i+4; j<len; ++j)
			{
				//找出冒号
				if(USART2_RX_BUF[j] == 0x3A)
				{
					x=0;
					for(k=j+1; k<len; ++k)
					{
						WIFI_DATA[x++] = USART2_RX_BUF[k];						
					}
					WIFI_DATA[x++] = 0;
					return;
				}
			}
			WIFI_DATA[0] = 0x22;WIFI_DATA[1] = 0x22;WIFI_DATA[2] = 0x22;WIFI_DATA[3]=0;
		}
		else{
			WIFI_DATA[0] = 0x11;WIFI_DATA[1] = 0x11;WIFI_DATA[2] = 0x11;WIFI_DATA[3]=0;
		}
	}
	
	WIFI_DATA[0] = 0x44;WIFI_DATA[1] = 0x44;WIFI_DATA[2] = 0x44;WIFI_DATA[3]=0;
	
}

int getLen(char *data)
{
	int i=0;
	while(data[i] != 0)
	{
		++i;
	}
	return i;
}


int main()
{
	u16 len;
	int wifi_angle = 0;
	float adc_angle = 0;
	
	unsigned char s;
	
	Init();
	
	InitWifi();
	
	while(1)
	{
		
		if(USART1_RX_STA&0x8000)
		{
			len=USART1_RX_STA&0x3fff;
			USART1_RX_BUF[len] = 0x0d;
			USART1_RX_BUF[len+1] = 0x0a;
			USART2_Send_Data(USART1_RX_BUF,len+2);//通过串口1将接收到的固定长度字符发送出去
			USART1_RX_STA = 0;
		}
		
		if(USART2_RX_STA&0x8000)
		{
			len=USART2_RX_STA&0x3fff;
	//		USART1_Send_Data(USART2_RX_BUF,len);//通过串口2将接收到的固定长度字符发送出去
			
			
			getWifiData();
			
	//		USART1_Send_Data(WIFI_DATA,getLen(WIFI_DATA));
			
		//	sscanf(WIFI_DATA,"%d",&wifi_angle);
			
			wifi_angle = atoi(WIFI_DATA); //字符串强转int型
			
			USART2_RX_STA = 0; //重置
			
			// 175~195 实际176~193 对应 0度~180度
			adc_angle = wifi_angle/9.0f + 175;
			
		
			if(adc_angle < 176) adc_angle = 176;
			if(adc_angle >193 ) adc_angle = 193;

			TIM_SetCompare2(TIM3,adc_angle); 	
			
		}
		

	}
	return 0;
}

void USART1_IRQHandler(void)      
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);    //读取接收到的数据 
		
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART1_RX_STA=0;//接收错误,重新开始
				else USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{
				if(Res==0x0d)USART1_RX_STA|=0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA & 0x3fff]=Res;        //记录接收到的值
					USART1_RX_STA++;                      //接收数据增加1 
					if(USART1_RX_STA>(DATALEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收
				}
			}	
		}
	}

	 USART_ClearFlag(USART1,USART_IT_RXNE); //一定要清除接收中断
}


//"OK" 是帧尾
void USART2_IRQHandler(void)      
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是 OK 结尾)
	{
		Res =USART_ReceiveData(USART2);    //读取接收到的数据 
		
		if((USART2_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART2_RX_STA&0x4000)//接收到了O
			{
				if(Res!='K')USART2_RX_STA=0;//接收错误,重新开始
				else USART2_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到O
			{
				if(Res=='O')USART2_RX_STA|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_STA & 0x3fff]=Res;        //记录接收到的值
					USART2_RX_STA++;                      //接收数据增加1 
					if(USART2_RX_STA>(DATALEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收
				}
			}	
		}
	}

	 USART_ClearFlag(USART2,USART_IT_RXNE); //一定要清除接收中断
}
