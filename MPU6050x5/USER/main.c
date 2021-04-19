
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050iic_1.h"
#include "mpu6050iic_2.h"
//#include "mpu6050iic_3.h"
#include "mpu6050iic_4.h"
//#include "mpu6050iic_5.h"

#include "math.h"
#include "time.h"

#include "kalman.h"
#include "kalman2.h"
//#include "kalman3.h"
#include "kalman4.h"
//#include "kalman5.h"


//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 

 	
//初始化IIC
void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10|GPIO_Pin_11 
									| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 
									| GPIO_Pin_14 | GPIO_Pin_15 ;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_SetBits( GPIOB,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10|GPIO_Pin_11 
						| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13
						| GPIO_Pin_14 | GPIO_Pin_15);				

}

 int main(void)
 {	 
	
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
//	short temp;					//温度	
	 int i;
	 u8 res;
	float angle_ax;
	float gy;
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为500000
	delay_init();	//延时初始化 
	delay_ms(1000);
	MPU_IIC_Init();
	res = MPU_Init();					//初始化MPU6050
 	res = MPU_Init2();
	//res = MPU_Init3();
	res = MPU_Init4();
	
	
 	while(1)
	{	
					
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //陀螺仪得到的角速度
			Kalman_Filter(angle_ax,gy);
	
			MPU_Get_Accelerometer2(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope2(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //陀螺仪得到的角速度
			Kalman_Filter2(angle_ax,gy);
			
		/**	
			MPU_Get_Accelerometer3(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope3(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //陀螺仪得到的角速度
			Kalman_Filter3(angle_ax,gy);
		**/
		
		
			MPU_Get_Accelerometer4(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope4(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //陀螺仪得到的角速度
			Kalman_Filter4(angle_ax,gy);
	
			
			usart1_send_char(0x99);
			usart1_send_char(0xAA);			
			for(i=0;i<4;i++)
				usart1_send_char(r1.data[3-i]); //部分软件识别需要对数据倒过来	
				
			usart1_send_char(0x99);
			usart1_send_char(0xBB);			
			for(i=0;i<4;i++)
				usart1_send_char(r12.data[3-i]);
			/**
			usart1_send_char(0x99);
			usart1_send_char(0xCC);			
			for(i=0;i<4;i++)
				usart1_send_char(r13.data[3-i]); 
				**/
				
			usart1_send_char(0x99);
			usart1_send_char(0xDD);			
			for(i=0;i<4;i++)
				usart1_send_char(r14.data[3-i]); 
		
			
	
	} 	
}
 


