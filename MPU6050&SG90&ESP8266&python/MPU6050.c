
#include "delay.h"

#include "sys.h"

#include "usart.h"
#include "mpu6050.h"

#include "inv_mpu.h"
#include "math.h"



//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1,c);  
} 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+4]=0X00;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	

//�������˲������뺯��
float dt=0.001;//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��
float angle=0.0, angle_dot=0.0;//�ǶȺͽ��ٶ�
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

union result
 {
		 float d;
		 unsigned char data[4];
 }r1,t1;


void Kalman_Filter(double angle_m,double gyro_m)
{
angle+=(gyro_m-q_bias) * dt;
angle_err = angle_m - angle;
Pdot[0]=Q_angle - P[0][1] - P[1][0];
Pdot[1]= - P[1][1];
Pdot[2]= - P[1][1];
Pdot[3]=Q_gyro;
P[0][0] += Pdot[0] * dt;
P[0][1] += Pdot[1] * dt;
P[1][0] += Pdot[2] * dt;
P[1][1] += Pdot[3] * dt;
PCt_0 = C_0 * P[0][0];
PCt_1 = C_0 * P[1][0];
E = R_angle + C_0 * PCt_0;
K_0 = PCt_0 / E;
K_1 = PCt_1 / E;
t_0 = PCt_0;
t_1 = C_0 * P[0][1];
P[0][0] -= K_0 * t_0;
P[0][1] -= K_0 * t_1;
P[1][0] -= K_1 * t_0;
P[1][1] -= K_1 * t_1;
angle += K_0 * angle_err; //���ŽǶ�
q_bias += K_1 * angle_err;
angle_dot = gyro_m-q_bias;//���Ž��ٶ�

r1.d = angle;
}
 	
 int main(void)
 {	 
	u8 report=1;			//Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	
	 int i;
	 u8 res;
	
	 
	 
	 float K1 =0.1; // �Լ��ٶȼ�ȡֵ��Ȩ��
	float dt=0.001;//ע�⣺dt��ȡֵΪ�˲�������ʱ��
	float angle;

	float angle_ax;
	 float gy;
	float Pitch;
	 
	 unsigned char buf[4]; 
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ�� 
	delay_ms(1000);
	res = MPU_Init();					//��ʼ��MPU6050
 	
	
	res = 1;
	 
	
 	while(1)
	{	
		
			
	//		temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
		
			
		
		//	if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			angle_ax=atan(aacx/aacz)*57.3;     //���ٶȵõ��ĽǶ�
			gy=(float)gyroy/16.4;       //�����ǵõ��Ľ��ٶ�
			Kalman_Filter(angle_ax,gy);
		
			//�Զ���֡ͷ
			usart1_send_char(0x99);
			usart1_send_char(0x11);			
			for(i=0;i<4;i++)
				usart1_send_char(r1.data[3-i]); //�������ʶ����Ҫ�����ݵ�����
			
		
			
		
		
	//	}
	} 	
}
 


