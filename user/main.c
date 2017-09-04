#include "stm32f10x.h"
#include "zl10_4out.h"
#include "key.h"
#include "stdio.h"
#include "display.h"
#include "menu.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_exti.h"


#define 	FLASH_START_ADDR 	 						0x08007000

uint8_t CheckFLag=0;

#define _Gpio_12_set  GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12))
#define _Gpio_7_set  GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7))

uint8_t TIM1step=0;

RCC_ClocksTypeDef   SysClock;
/****************************??????****************************/  
extern uint32_t   ShortCircuitLastTime;
volatile uint32_t timenum; 
extern uint8_t  	EventFlag; 
extern uint8_t 	ShortCircuit;
extern uint8_t 	ShortCircuitTimer;
extern int16_t OUT2_TimerCounter;

extern uint8_t OUT2;
extern uint8_t OUT1;
extern uint32_t CPV;

uint8_t LastRegisterA=0;

void timer_init(void);
void GPIO_Config(void);
uint8_t FlashCheck(void);
void GPIO_DEINIT_ALL(void);
void WriteFlash(uint32_t addr,uint32_t data);
/*****************************************/
///////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  //加入以下代码,支持printf函数,而不需要选择use MicroLIB
  */
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART1, (unsigned char) ch);
  return (ch);
}


void RCC_Configuration(void)
{
			RCC_DeInit();//??? RCC?????????
			RCC_HSICmd(ENABLE);//??HSI
			while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)//??HSI???? 
			{
			}
			RCC_HCLKConfig(RCC_SYSCLK_Div1);
			//RCC_PCLKConfig(RCC_HCLK_Div1);
			
			RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_12);
			RCC_PLLCmd(ENABLE);
			//RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET)
			{
				
			}
			
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
			
			while(RCC_GetSYSCLKSource()!=0x08)
			{
			}
		
//		PWR_BackupAccessCmd(ENABLE);//允许修改RTC和后备寄存器
//		RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速时钟
//		PWR_BackupAccessCmd(DISABLE);//禁止修改RTC和后备寄存器
}


void Timer3_init(void)
{
	TIM_TimeBaseInitTypeDef timer_init_structure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                //使能TIM3中断通道  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;          
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	NVIC_Init(&NVIC_InitStructure);
	
	/*TIM3*/
	TIM_DeInit(TIM3);                                               //复位TIM3
	TIM_TimeBaseStructInit(&timer_init_structure);                  //初始化TIM结构体  

	timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //系统时钟,不分频,24M  
	timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //向上计数模式  
	timer_init_structure.TIM_Period = 10;                          //每300 uS触发一次中断,??ADC  
	timer_init_structure.TIM_Prescaler = 47;                      //计数时钟分频,f=1M,systick=1 uS  
	timer_init_structure.TIM_RepetitionCounter = 0x00;              //发生0+1的update事件产生中断 
	
	TIM_TimeBaseInit(TIM3, &timer_init_structure);  
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                       //使能TIM3中断
	TIM_Cmd(TIM3, ENABLE);                                          //使能TIM3

}


void timer_init()  
{  
    TIM_TimeBaseInitTypeDef timer_init_structure;  
//		TIM_OCInitTypeDef timer_OCinit_structure; 
    NVIC_InitTypeDef nvic_init_structure;  
//  	GPIO_InitTypeDef gpio_init_structure;  

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
  
//    GPIO_StructInit(&gpio_init_structure);  
//    //GPIOA                                                         //PA-0~3??ADC  
//    gpio_init_structure.GPIO_Pin = GPIO_Pin_9;  
//    gpio_init_structure.GPIO_Mode = GPIO_Mode_AF;                   //????(??)??  
//    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;              //Fast speed  
//    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
//    GPIO_Init(GPIOA, &gpio_init_structure);
	
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_2);
//	
//    nvic_init_structure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;                //使能TIM1中断通道  
//    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM1中断  
//    nvic_init_structure.NVIC_IRQChannelPriority = 2;                //优先级为0  
//		
//    NVIC_Init(&nvic_init_structure);  
		
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		nvic_init_structure.NVIC_IRQChannel = TIM2_IRQn;                //使能TIM2中断通道  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //使能TIM2中断  
    NVIC_Init(&nvic_init_structure); 
			
		/*TIM2*/
    TIM_DeInit(TIM2);                                               //复位TIM2  
    TIM_TimeBaseStructInit(&timer_init_structure);                  //初始化TIM结构体  
  
    timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //向上计数模式  
    timer_init_structure.TIM_Period = 10;                          //每300 uS触发一次中断,??ADC  
    timer_init_structure.TIM_Prescaler = 47;                      //计数时钟分频,f=1M,systick=1 uS  
    timer_init_structure.TIM_RepetitionCounter = 0;              //发生0+1的update事件产生中断 
		
    TIM_TimeBaseInit(TIM2, &timer_init_structure);  
  
//		timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
//		timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable;
//		timer_OCinit_structure.TIM_Pulse = 10;
//		timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;
		
		
//		TIM_OC4Init(TIM2,&timer_OCinit_structure);
//		TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
//		TIM_ARRPreloadConfig(TIM2,ENABLE);
		
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //使能TIM2中断
		TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);           //选择TIM1的update为触发源  
//		TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
//		TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);//触发模式只启动，门控制启停都可以控制
//		TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//主从模式MSM

		TIM_Cmd(TIM2, ENABLE);


/* TIM1 ??? ---------------------------------------------------
   TIM1 ????(TIM1CLK) ??? APB2 ?? (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock ?48 MHz 
   
   ??????? 4 ?PWM ???17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   ??1??????? 50%
   ??2??????? 37.5%
   ??3??????? 25%
   ??4??????? 12.5%
   ????????????:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	*/  
    /*???????,???????????????*/
  //TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE);
  /* TIM1 ???? */
  
  
  /* Time ??????*/
//  timer_init_structure.TIM_Prescaler = 47;
//  timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time ????????????*/
//  timer_init_structure.TIM_Period = 25;
//  timer_init_structure.TIM_RepetitionCounter = 0;

//  TIM_TimeBaseInit(TIM1, &timer_init_structure);

//  /* ??1,2,3,4?PWM ???? */
//  timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
//  timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable ;//TIM_OutputState_Enable; //PWM?????
//  timer_OCinit_structure.TIM_OutputNState = TIM_OutputNState_Disable ;//TIM_OutputNState_Enable; //??PWM?????
//  timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;  //PWM 1?????
//  timer_OCinit_structure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //PWM?? 0?????
//  timer_OCinit_structure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  timer_OCinit_structure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

//  timer_OCinit_structure.TIM_Pulse = 3; //?????
//  TIM_OC1Init(TIM1, &timer_OCinit_structure);//????1??
//	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
//	
//	timer_OCinit_structure.TIM_Pulse = 3; //?????
//  TIM_OC2Init(TIM1, &timer_OCinit_structure);//????1??
//	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
//	
//	timer_OCinit_structure.TIM_Pulse = 3; //?????
//  TIM_OC3Init(TIM1, &timer_OCinit_structure);//????1??
//	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

//  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                      //使能TIM1中断
//	TIM_ARRPreloadConfig(TIM1,ENABLE);
//	
//  /* TIM1 ?????*/
//	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);							//选择TIM1的timer为触发源  
//	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);							//选择TIM1的timer为触发源  
//	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref);							//选择TIM1的timer为触发源  
//	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);							//选择TIM1的timer为触发源  
//	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);     //清除update事件中断标志
//	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//主从模式MSM  
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	//TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
//	
//  TIM_Cmd(TIM1, ENABLE);
//	
  /* TIM1 ????? */
  

}  

///*重设TIM1进入OnePule模式，用于控制脉冲个数*/

//void ChangeTIM1ToOnePulse(int Counter)
//{
//	TIM_TimeBaseInitTypeDef timer_init_structure;
//	TIM_Cmd(TIM1, DISABLE);
//	
//	timer_init_structure.TIM_RepetitionCounter = Counter;
//  TIM_TimeBaseInit(TIM1, &timer_init_structure);
//	
//	
//	TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
//	TIM_Cmd(TIM1, DISABLE);
//}

/*******************************************
数码GPIO初始化
********************************************/
void SMG_GPIO_INIT(void)
{
	
	   GPIO_InitTypeDef GPIO_InitStructure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);  
	
    GPIO_StructInit(&GPIO_InitStructure);  
	
			//GPIOD3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                
		GPIO_Init(GPIOA, &GPIO_InitStructure);  

		//GPIOD2~4
    GPIO_InitStructure.GPIO_Pin = D4_Pin;  
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                                 
    GPIO_Init(D4_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D3_Pin;
		GPIO_Init(D3_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D2_Pin;
		GPIO_Init(D2_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D1_Pin;
		GPIO_Init(D1_GPIO_Port, &GPIO_InitStructure); 
	  //GPIOD5~7                                                       
    GPIO_InitStructure.GPIO_Pin = D5_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                             
    GPIO_Init(D5_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D6_Pin;  		
		GPIO_Init(D6_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D7_Pin;  
		GPIO_Init(D7_GPIO_Port, &GPIO_InitStructure);  
		//GPIOA                                                        
    GPIO_InitStructure.GPIO_Pin = D9_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                                 
    GPIO_Init(D8_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D9_Pin;  
		GPIO_Init(D9_GPIO_Port, &GPIO_InitStructure);  

		//GPIOB
	  GPIO_InitStructure.GPIO_Pin = A_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;               
		GPIO_Init(A_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = B_Pin; 
		GPIO_Init(B_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = C_Pin; 
		GPIO_Init(C_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = D_Pin; 
		GPIO_Init(D_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = E_Pin; 
		GPIO_Init(E_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = F_Pin; 
		GPIO_Init(F_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = G_Pin; 
		GPIO_Init(G_GPIO_Port, &GPIO_InitStructure); 
		
		
		GPIO_WriteBit(D1_GPIO_Port, D1_Pin, Bit_SET);
		GPIO_WriteBit(D2_GPIO_Port, D2_Pin, Bit_SET);
		GPIO_WriteBit(D3_GPIO_Port, D3_Pin, Bit_SET);
		GPIO_WriteBit(D4_GPIO_Port, D4_Pin, Bit_SET);
		GPIO_WriteBit(D5_GPIO_Port, D5_Pin, Bit_SET);
		GPIO_WriteBit(D6_GPIO_Port, D6_Pin, Bit_SET);
		GPIO_WriteBit(D7_GPIO_Port, D7_Pin, Bit_SET);
		GPIO_WriteBit(D8_GPIO_Port, D8_Pin, Bit_SET);
		GPIO_WriteBit(D9_GPIO_Port, D9_Pin, Bit_SET);

		GPIO_WriteBit(A_GPIO_Port, A_Pin, Bit_SET);
		GPIO_WriteBit(B_GPIO_Port, B_Pin, Bit_SET);
		GPIO_WriteBit(C_GPIO_Port, C_Pin, Bit_SET);
		GPIO_WriteBit(D_GPIO_Port, D_Pin, Bit_SET);
		GPIO_WriteBit(E_GPIO_Port, E_Pin, Bit_SET);
		GPIO_WriteBit(F_GPIO_Port, F_Pin, Bit_SET);
		GPIO_WriteBit(G_GPIO_Port, G_Pin, Bit_SET);

}

void CompareOUT1GPIO(void)
{   
		NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure; 
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
		//COMPARE_OUT1
    GPIO_InitStructure.GPIO_Pin = COMPOUT1_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                
		GPIO_Init(COMPOUT1_GPIO_Port, &GPIO_InitStructure);
	          
    EXTI_ClearITPendingBit(EXTI_Line2);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line= EXTI_Line2;  
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Falling;   
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure);

}

void IO_GPIO_INIT(void)
{
		GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);  
	
    GPIO_StructInit(&gpio_init_structure);  
	
		//OUT1_GPIO_Port,OUT2_GPIO_Port
    gpio_init_structure.GPIO_Pin = OUT1_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT1_GPIO_Port, &gpio_init_structure);

	  gpio_init_structure.GPIO_Pin = OUT2_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT2_GPIO_Port, &gpio_init_structure);

		gpio_init_structure.GPIO_Pin = OUT3_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT3_GPIO_Port, &gpio_init_structure);

		gpio_init_structure.GPIO_Pin = OUT4_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT4_GPIO_Port, &gpio_init_structure);
	
		//SC_GPIO_Port
    gpio_init_structure.GPIO_Pin = SC_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                                 
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;                                
		GPIO_Init(SC_GPIO_Port, &gpio_init_structure);
		
}
	
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET) 
   {  
       EXTI_ClearITPendingBit(EXTI_Line2);  
       CPV++;
			 if(CPV>=CSV)
					CPV = 0;
			 OUT1 = 1;
			 SetOUT1Status();
   }   
}


void TIM2_IRQHandler()  
{
    if(TIM_GetITStatus(TIM2, TIM_FLAG_Update))            //判断发生update事件中断  
    {  
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);     //清除update事件中断标志
    }  
}  

void TIM1_BRK_UP_TRG_COM_IRQHandler()  
{  
    if(TIM_GetITStatus(TIM1, TIM_FLAG_Update))            //判断发生update事件中断  
    {

      TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);     //清除update事件中断标志
    }  
} 

void TIM3_IRQHandler()
{
	  if(TIM_GetITStatus(TIM3, TIM_IT_Update))            //判断发生update事件中断  
    { 
				timenum++;
				//GPIOB->ODR ^= GPIO_Pin_8;
				if(timenum%10==0) /*5*10us=50us*/
				{
					//GPIOB->ODR ^= GPIO_Pin_8;
					if(OUT2)
						OUT2_TimerCounter++;
					
					SMG_Diplay();
					Key_Scan();									//定时扫描按键
					ShortCircuitLastTime++;				
					if(ShortCircuit)
						ShortCircuitCounter++;
					else
						ShortCircuitCounter=0;
				}
				if(timenum>=5000)	/*5000*10us = 500,000us = 500ms*/
				{
					EventFlag = EventFlag | Blink500msFlag;
					timenum = 0;
				}
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //清除update事件中断标志
		}
}

/******************************************
 BSP 底层初始化
******************************************/
void bsp_init(void)  
{  
		RCC_Configuration();
    timer_init();           //  
		Timer3_init();
		RCC_GetClocksFreq(&SysClock);
		IO_GPIO_INIT();
		CompareOUT1GPIO();
		SMG_GPIO_INIT();
		Button_Init();
}

/******************************************
Main主函数
******************************************/
int main(void)
{
		uint32_t checkcouter;
	
		bsp_init();
	
		CheckFLag = FlashCheck();
	
		if(CheckFLag)
			differenttialDC();
		else
			while(1)
			{
				checkcouter++;
			}
}

/*****************************************************************/
void WriteFlash(uint32_t addr,uint32_t data)
{
	FLASH_Unlock(); //??FLASH???????
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????
	FLASH_ErasePage(FLASH_START_ADDR); //???????
	FLASH_ProgramWord(FLASH_START_ADDR+(addr*4),data); //?????0?????
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//?????
	FLASH_Lock(); //??FLASH???????
}


//FLASH??????
uint32_t Flashtemp;
void printFlashTest(void)
{
		uint32_t choose = 0;
		Flashtemp = *(__IO uint32_t*)(FLASH_START_ADDR);
		//DelaymsSet(500);
		//printf("addr:0x%x, data:0x%x\r\n", addr, temp);
		choose = Flashtemp & 0xF0000000;

}

/*************************************************************/
#define 	FLASH_START_ADDR1 	 						0x08008000

uint8_t* UID=(uint8_t*)0x1FFFF7E8;  //获取UID  stm32f0:0x1FFFF7AC,stm32f100:0x1FFFF7E8
uint32_t Fml_Constant 	= 0x19101943;//输入到公式的常熟
uint8_t *C= (uint8_t*)&Fml_Constant;//把常量转化成数组
uint8_t FormulaResult[4];
uint32_t FormulaCheck;
uint32_t UIDFlashResult;
uint16_t Fml_CRC16;
uint8_t D[12];

void Formula_100(uint8_t *D,uint8_t *Result)
{
	D[0] = UID[4];
	D[1] = UID[8];
	D[2] = UID[1];
	D[3] = UID[3];
	D[4] = UID[0];
	D[5] = UID[5];
	D[6] = UID[10];
	D[7] = UID[7];
	D[8] = UID[9];
	D[9] = UID[2];
	D[10] = UID[11];
	D[11] = UID[6];
	
	Result[0] = C[0] ^ D[0];
	Result[1] = C[1] ^ D[6] ^ D[7] ^ D[8] ^ D[9] ^ D[10] ^ D[11] ;
	Result[2] = C[2] ^ D[4] ;
	Result[3] = C[3] ^ D[2] ^ D[1];
}

uint8_t FlashCheck(void)
{
		uint8_t FlashFlag;
		Formula_100(D,FormulaResult);
		FormulaCheck = FormulaResult[0]+(FormulaResult[1]<<8)+(FormulaResult[2]<<16)+(FormulaResult[3]<<24);
		UIDFlashResult = *(__IO uint32_t*)(FLASH_START_ADDR1);
		if(UIDFlashResult==FormulaCheck)
			FlashFlag =1;
		else
			FlashFlag =0;
		
		return FlashFlag;
			
}
uint16_t Formula_CRC16(uint8_t *p,uint8_t len)
{
	uint8_t i;
	while(len--)
	{
		for(i=0x80; i!=0; i>>=1)
		{
			if((Fml_CRC16 & 0x8000) != 0)
			{
				Fml_CRC16 <<= 1;
				Fml_CRC16 ^= 0x1021;
			}
			else
			{
				Fml_CRC16 <<= 1;
			}
			if((*p&i)!=0)
			{
				Fml_CRC16 ^= 0x1021;
			}
		}
		p++;
	}
	return Fml_CRC16;
}
