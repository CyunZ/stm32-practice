//ͷ�ļ�
#include "stm32f10x.h" 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "string.h"

#define DATALEN 200

char USART1_RX_BUF[DATALEN];
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART1_RX_STA=0;       //����״̬���



char USART2_RX_BUF[DATALEN];
u16 USART2_RX_STA=0;



//��ԭ����ʱ��������systic�ж�
void delay_us(u32 nus)
{
 u32 temp;
 SysTick->LOAD = 9*nus;
 SysTick->VAL=0X00;//��ռ�����
 SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
 do
 {
  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
 }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
     SysTick->CTRL=0x00; //�رռ�����
    SysTick->VAL =0X00; //��ռ�����
}
void delay_ms(u16 nms)
{
 u32 temp;
 SysTick->LOAD = 9000*nms;
 SysTick->VAL=0X00;//��ռ�����
 SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
 do
 {
  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
 }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
    SysTick->CTRL=0x00; //�رռ�����
    SysTick->VAL =0X00; //��ռ�����
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
	

	//LED�Ƴ�ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//USART��س�ʼ��
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
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1,ENABLE);
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�   0-3;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    //PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//ֹͣ��λ
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�   0-3;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
	
	
	
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

int main()
{
	u16 len;
	Init();
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	
	InitWifi();
	
	while(1)
	{
		if(USART1_RX_STA&0x8000)
		{
			len=USART1_RX_STA&0x3fff;
			USART1_RX_BUF[len] = 0x0d;
			USART1_RX_BUF[len+1] = 0x0a;
			USART2_Send_Data(USART1_RX_BUF,len+2);//ͨ������1�����յ��Ĺ̶������ַ����ͳ�ȥ
			USART1_RX_STA = 0;
		}
		
		if(USART2_RX_STA&0x8000)
		{
			len=USART2_RX_STA&0x3fff;
			USART1_Send_Data(USART2_RX_BUF,len);//ͨ������2�����յ��Ĺ̶������ַ����ͳ�ȥ
			USART2_RX_STA = 0;
			
			if( USART2_RX_BUF[len-1] == '1' )
			{
				TIM_SetCompare2(TIM3,176);
				GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			}
			if( USART2_RX_BUF[len-1] == '2')
			{
				TIM_SetCompare2(TIM3,180);
				GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			}
			if(  USART2_RX_BUF[len-1] == '3' )
			{
				TIM_SetCompare2(TIM3,185);
				GPIO_SetBits(GPIOC,GPIO_Pin_13);
			}
			if(  USART2_RX_BUF[len-1] == '4' )
			{
				TIM_SetCompare2(TIM3,190);
				GPIO_SetBits(GPIOC,GPIO_Pin_13);
			}
			if(  USART2_RX_BUF[len-1] == '5' )
			{
				TIM_SetCompare2(TIM3,193);
				GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			}
			
		}
		

	}
	/*
		for(j=0; j<99; ++j)
			for(i=0; i<99999; ++i)
				;
	*/
	
	return 0;
}

void USART1_IRQHandler(void)      
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);    //��ȡ���յ������� 
		
		if((USART1_RX_STA&0x8000)==0)//����δ���
		{
			if(USART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART1_RX_STA=0;//���մ���,���¿�ʼ
				else USART1_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{
				if(Res==0x0d)USART1_RX_STA|=0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA & 0x3fff]=Res;        //��¼���յ���ֵ
					USART1_RX_STA++;                      //������������1 
					if(USART1_RX_STA>(DATALEN-1))USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����
				}
			}	
		}
	}

	 USART_ClearFlag(USART1,USART_IT_RXNE); //һ��Ҫ��������ж�
}


void USART2_IRQHandler(void)      
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�����ж�(���յ������ݱ����� OK ��β)
	{
		Res =USART_ReceiveData(USART2);    //��ȡ���յ������� 
		
		if((USART2_RX_STA&0x8000)==0)//����δ���
		{
			if(USART2_RX_STA&0x4000)//���յ���O
			{
				if(Res!='K')USART2_RX_STA=0;//���մ���,���¿�ʼ
				else USART2_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�O
			{
				if(Res=='O')USART2_RX_STA|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_STA & 0x3fff]=Res;        //��¼���յ���ֵ
					USART2_RX_STA++;                      //������������1 
					if(USART2_RX_STA>(DATALEN-1))USART2_RX_STA=0;//�������ݴ���,���¿�ʼ����
				}
			}	
		}
	}

	 USART_ClearFlag(USART2,USART_IT_RXNE); //һ��Ҫ��������ж�
}
