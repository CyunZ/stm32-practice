
#include "delay.h"

#include "sys.h"

#include "usart.h"
#include "mpu6050.h"

#include "inv_mpu.h"
#include "math.h"



//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+4]=0X00;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
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
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	

//卡尔曼滤波参数与函数
float dt=0.001;//注意：dt的取值为kalman滤波器采样时间
float angle=0.0, angle_dot=0.0;//角度和角速度
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
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
angle += K_0 * angle_err; //最优角度
q_bias += K_1 * angle_err;
angle_dot = gyro_m-q_bias;//最优角速度

r1.d = angle;
}
 	
 int main(void)
 {	 
	u8 report=1;			//默认开启上报
	u8 key;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
	 int i;
	 u8 res;
	
	 
	 
	 float K1 =0.1; // 对加速度计取值的权重
	float dt=0.001;//注意：dt的取值为滤波器采样时间
	float angle;

	float angle_ax;
	 float gy;
	float Pitch;
	 
	 unsigned char buf[4]; 
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为500000
	delay_init();	//延时初始化 
	delay_ms(1000);
	res = MPU_Init();					//初始化MPU6050
 	
	
	res = 1;
	 
	
 	while(1)
	{	
		
			
	//		temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		
			
		
		//	if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			angle_ax=atan(aacx/aacz)*57.3;     //加速度得到的角度
			gy=(float)gyroy/16.4;       //陀螺仪得到的角速度
			Kalman_Filter(angle_ax,gy);
		
			//自定义帧头
			usart1_send_char(0x99);
			usart1_send_char(0x11);			
			for(i=0;i<4;i++)
				usart1_send_char(r1.data[3-i]); //部分软件识别需要对数据倒过来
			
		
			
		
		
	//	}
	} 	
}
 


