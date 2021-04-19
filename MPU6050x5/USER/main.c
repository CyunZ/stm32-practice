
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


//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1,c);  
} 

 	
//��ʼ��IIC
void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTBʱ�� 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10|GPIO_Pin_11 
									| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 
									| GPIO_Pin_14 | GPIO_Pin_15 ;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
  GPIO_SetBits( GPIOB,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_10|GPIO_Pin_11 
						| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13
						| GPIO_Pin_14 | GPIO_Pin_15);				

}

 int main(void)
 {	 
	
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
//	short temp;					//�¶�	
	 int i;
	 u8 res;
	float angle_ax;
	float gy;
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ�� 
	delay_ms(1000);
	MPU_IIC_Init();
	res = MPU_Init();					//��ʼ��MPU6050
 	res = MPU_Init2();
	//res = MPU_Init3();
	res = MPU_Init4();
	
	
 	while(1)
	{	
					
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //�����ǵõ��Ľ��ٶ�
			Kalman_Filter(angle_ax,gy);
	
			MPU_Get_Accelerometer2(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope2(&gyrox,&gyroy,&gyroz);	//�õ�����������
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //�����ǵõ��Ľ��ٶ�
			Kalman_Filter2(angle_ax,gy);
			
		/**	
			MPU_Get_Accelerometer3(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope3(&gyrox,&gyroy,&gyroz);	//�õ�����������
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //�����ǵõ��Ľ��ٶ�
			Kalman_Filter3(angle_ax,gy);
		**/
		
		
			MPU_Get_Accelerometer4(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope4(&gyrox,&gyroy,&gyroz);	//�õ�����������
			angle_ax=atan2(aacz,aacx)*180/3.14;
			gy=(float)gyroy/16.4;       //�����ǵõ��Ľ��ٶ�
			Kalman_Filter4(angle_ax,gy);
	
			
			usart1_send_char(0x99);
			usart1_send_char(0xAA);			
			for(i=0;i<4;i++)
				usart1_send_char(r1.data[3-i]); //�������ʶ����Ҫ�����ݵ�����	
				
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
 


