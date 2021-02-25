#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* degiskenler*/

uint32_t cnt ; 
uint32_t temp ; 
uint32_t ifg ; 
/* address makrolari*/
//timer makrolari
#define RCC_Base_Address                0x40023800
#define APB1_Clock_Enable_Reg_Offset    0x40
#define RCC_APB1_Clock_Enable_Address   RCC_Base_Address + APB1_Clock_Enable_Reg_Offset

#define TIM6_Base_Address               0x40001000
#define TIM6_Control_Reg1_Offset        0x00
#define TIM6_Control_Reg1_Address       TIM6_Base_Address + TIM6_Control_Reg1_Offset
#define TIM6_Prescaler_Reg_Offset       0x28
#define TIM6_Prescaler_Reg_Address      TIM6_Base_Address + TIM6_Prescaler_Reg_Offset
#define TIM6_Counter_Reg_Offset         0x24
#define TIM6_Counter_Reg_Address        TIM6_Base_Address + TIM6_Counter_Reg_Offset
#define TIM6_Event_Gen_Reg_Offset       0x14
#define TIM6_Event_Gen_Address          TIM6_Base_Address + TIM6_Event_Gen_Reg_Offset
#define TIM6_IE_Reg_Offset              0x0C
#define TIM6_IE_Reg_Address             TIM6_Base_Address + TIM6_IE_Reg_Offset
#define TIM6_SR_Reg_Offset              0x10 
#define TIM6_SR_Reg_Address             TIM6_Base_Address + TIM6_SR_Reg_Offset
#define TIM6_ARR_Reg_Offset             0x2C
#define TIM6_ARR_Reg_Address            TIM6_Base_Address + TIM6_ARR_Reg_Offset
#define NVIC_TIM6_IE_Address           0x00000118


//#define TIM6_IFG_Address                0x00000118

// gpio makrolari 
#define RCC_BASE_ADDR                           0x40023800
#define AHB1ENR_OFFSET                          0x30
#define GPIOD_CLK_ENABLE_ADDR                   RCC_BASE_ADDR + AHB1ENR_OFFSET


#define GPIOD_BASE_ADDR                         0x40020C00
#define PORT_MODE_REG_OFFSET                    0x00 
#define GPIOD_PORT_MODE_ADDR                    GPIOD_BASE_ADDR + PORT_MODE_REG_OFFSET
#define PORT_OUTPUT_SPEED_REG_OFFSET            0x08
#define GPIOD_OUTPUT_SPEED_REG_ADDR             GPIOD_BASE_ADDR + PORT_OUTPUT_SPEED_REG_OFFSET
#define PULL_UP_DOWN_REG_OFFSET                 0x0C
#define GPIOD_PULL_UP_DOWN_ADDR                 GPIOD_BASE_ADDR + PULL_UP_DOWN_REG_OFFSET
#define BSSR_REG_OFFSET                         0x18
#define GPIOD_BSSR_ADDR                         GPIOD_BASE_ADDR + BSSR_REG_OFFSET

/* Fonksiyonlar*/
void tim6_clock_enable(void){
//clk enable (48Mhz)
*(uint32_t *)(RCC_APB1_Clock_Enable_Address ) |= 0x00000010;  
 for (int i = 0 ; i< 500000; i++);

}

void tim6_enable(void){
*(uint32_t *)(TIM6_Control_Reg1_Address ) |= 0x00000001;
  
}
void tim6_disable(void){

  *(uint32_t *)(TIM6_Control_Reg1_Address ) &= ~(0x00000001);

}

void tim6_set_prescaler(void){

*(uint32_t *)(TIM6_Prescaler_Reg_Address) |= 0x05FF;
 //0xBB7F ilk deger  --- 47999'u ifade ediyor
 //0x5DBF ikinci deger
 // 0x2EDF ucuncu deger 

}
void tim6_set_arr(void){
*(uint32_t *)(TIM6_ARR_Reg_Address) |= 1000 ;
 


}

void tim6_interrupt_enable(){
*(uint32_t *)(TIM6_IE_Reg_Address) |= 0x01 ; 
}
void NVIC_interrupt_enable(){
*(uint32_t *)(NVIC_TIM6_IE_Address) |= 0x01;
}

void gpio_init(){
// pd15 pin config 
  // RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD; // AHB1 Enable
 //clk enable 
 *(uint32_t *)(GPIOD_CLK_ENABLE_ADDR ) |= 0x00000008;  
 //port mode reg -> output pd15
 *(uint32_t *)(GPIOD_PORT_MODE_ADDR) |= 0x40000000; 
 // output speed -> medium speed pd15
 *(uint32_t *)(GPIOD_OUTPUT_SPEED_REG_ADDR) |= 0x40000000;  
  //pull up/down reg -> pull up pd15
 *(uint32_t *)(GPIOD_PULL_UP_DOWN_ADDR) |= 0x40000000;

 
  //set pd15
 *(uint32_t *)(GPIOD_BSSR_ADDR) |= 0x00008000;


}


void SystemClock_Config(void);
void tim6_clock_enable(void);
void tim6_enable(void);
void tim6_disable(void);
void tim6_set_prescaler(void);
void tim6_set_arr(void);
void tim6_interrupt_enable(void);
void gpio_init(void);
void NVIC_interrupt_enable(void);

int main(void)
{
  
  SystemClock_Config();
  gpio_init();
  tim6_clock_enable();
  tim6_set_prescaler(); 

  tim6_set_arr();
  tim6_interrupt_enable();
 // NVIC_interrupt_enable();
  tim6_enable();
//  
  cnt = 1 ; 
  *(uint32_t *)(TIM6_SR_Reg_Address) &= ~ 0x00000001  ; 
  while(1){
    temp = *(uint32_t *)(TIM6_SR_Reg_Address)  ; 
    if (*(uint32_t *)(TIM6_SR_Reg_Address) == 0x00000001)
    {
      cnt = cnt+1 ; 
   *(uint32_t *)(TIM6_SR_Reg_Address) &= ~ 0x00000001  ;  
   if (cnt % 2){
      //set pd15 (led)
    *(uint32_t *)(GPIOD_BSSR_ADDR) |= 0x00008000;
    }else{
      //reset pd15 (led)
    *(uint32_t *)(GPIOD_BSSR_ADDR) &= 0x00000000;
    *(uint32_t *)(GPIOD_BSSR_ADDR) |= 0x80000000;
    }
   
   
    }
   
   


  }
  
}
//NVIC_EnableIRQ
//NVIC_SetPriority
//TIM2_IRQ_handler
//TIM_DIER_UIE
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
