
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //TIM2 不重映射 PA0、PA1、PA2、PA3
	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	 
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);//TIM3部分重映像  PC6 PC7 PC8 PC9
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);//TIM3部分重映像  PB4 PB5 PB0 PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_4 | GPIO_Pin_0| GPIO_Pin_1; //TIM3_CH2 1 3 4
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7 | GPIO_Pin_8| GPIO_Pin_9; //TIM3_CH2 1 3 4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1; //TIM2_CH1 2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM3,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	
}




void vPWMTask(void * pvParameters)
{
	while(1)
	{
		TIM_SetCompare2(TIM3,176);
		TIM_SetCompare1(TIM3,177);
		TIM_SetCompare3(TIM3,179);
		TIM_SetCompare4(TIM3,178);
				
		TIM_SetCompare1(TIM2,179);
		TIM_SetCompare2(TIM2,178);
		
		vTaskDelay(3000);
	/**	
			
		vTaskDelay(2000);
		TIM_SetCompare2(TIM3,180);
		
		TIM_SetCompare1(TIM3,193);
		TIM_SetCompare3(TIM3,193);
		TIM_SetCompare4(TIM3,176);
		
		vTaskDelay(2000);
		TIM_SetCompare2(TIM3,185);
		
		TIM_SetCompare1(TIM3,190);
		TIM_SetCompare3(TIM3,176);
		TIM_SetCompare4(TIM3,190);
		
		vTaskDelay(2000);
		TIM_SetCompare2(TIM3,190);
		
		TIM_SetCompare1(TIM3,185);
		TIM_SetCompare3(TIM3,176);
		TIM_SetCompare4(TIM3,190);
		**/
		
	
		TIM_SetCompare2(TIM3,194);
		TIM_SetCompare1(TIM3,193);
		TIM_SetCompare3(TIM3,192);
		TIM_SetCompare4(TIM3,191);
		
		TIM_SetCompare1(TIM2,190);
		TIM_SetCompare2(TIM2,189);
		vTaskDelay(3000);
		/**
		TIM_SetCompare1(TIM3,176);
		TIM_SetCompare3(TIM3,180);
		TIM_SetCompare4(TIM3,185);
		**/
	}
	
}

void vLEDToggleTask(void * pvParameters)
{
	while(1)
	{
		
		vTaskDelay(1000);
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		vTaskDelay(1000);
	}
}

int main(void)
{
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	//TIM3_PWM_Init(899,0);
	TIM3_PWM_Init(199,7199);
	//GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	 
  xTaskCreate( vPWMTask, "vPWMTask", 256, NULL, 2, NULL );
	
//	xTaskCreate( vLEDToggleTask, "vLEDToggleTask", 256, NULL, 3, NULL );
	
	vTaskStartScheduler();
	return 0;
}



