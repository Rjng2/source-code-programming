/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "OLED.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//LED
#define R_LED1_PIN GPIO_PIN_6   // PA6
#define G_LED1_PIN GPIO_PIN_7   // PA7
#define R_LED2_PIN GPIO_PIN_8   // PA8
#define G_LED2_PIN GPIO_PIN_9   // PA9
#define R_LED3_PIN GPIO_PIN_10  // PA10
#define G_LED3_PIN GPIO_PIN_11  // PA11
#define R_LED4_PIN GPIO_PIN_12  // PA12
#define G_LED4_PIN GPIO_PIN_15  // PA15
//IR
#define IR_A1_PIN  GPIO_PIN_0  // PC0
#define IR_A2_PIN  GPIO_PIN_1  // PC1
#define IR_B1_PIN  GPIO_PIN_2  // PC2
#define IR_B2_PIN  GPIO_PIN_3  // PC3
#define IR_C1_PIN  GPIO_PIN_4  // PC4
#define IR_C2_PIN  GPIO_PIN_5  // PC5
#define IR_D1_PIN  GPIO_PIN_6  // PC6
#define IR_D2_PIN  GPIO_PIN_7  // PC7


#define ALL_RED_PINS (R_LED1_PIN | R_LED2_PIN | R_LED3_PIN | R_LED4_PIN)
#define ALL_GREEN_PINS (G_LED1_PIN | G_LED2_PIN | G_LED3_PIN | G_LED4_PIN)

#define ALL_LED_MASK (R_LED1_PIN | G_LED1_PIN | R_LED2_PIN | G_LED2_PIN | \
                      R_LED3_PIN | G_LED3_PIN | R_LED4_PIN | G_LED4_PIN)

int state = 0;
int current_group = 1;
int current_state = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t last_button_press = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


//OLED Display
void oled_display(void){
		
		OLED_ShowString(1,1,"Group:");
		switch(current_group){
				case 1: OLED_ShowString(1, 7, "A");break;
				case 2: OLED_ShowString(1, 7, "B");break;
				case 3: OLED_ShowString(1, 7, "C");break;
				case 4: OLED_ShowString(1, 7, "D");break;
				default : OLED_ShowString(1, 7, "A");break;
			}
		
		OLED_ShowString(2,1,"State:");
		OLED_ShowString(2, 7, "       ");
		
		switch(current_state){
				case 0: OLED_ShowString(2, 7, "Normal");break;
				case 1: OLED_ShowString(2, 7, "Busy");break;
				case 2: OLED_ShowString(2, 7, "Carless");break;
				default : OLED_ShowString(2, 7, "Normal");break;
			}
		
		OLED_ShowString(3,1,"Time:");
		OLED_ShowString(3, 7, "       ");
		switch(current_state){
				case 0: OLED_ShowString(3, 7, "5s");break;
				case 1: OLED_ShowString(3, 7, "5-15s");break;
				case 2: OLED_ShowString(3, 7, "3s");break;
				default : OLED_ShowString(3, 7, "5s");break;
			}
		
}

//ALL RED ON
void ALL_RED_ON(void) {
    
		GPIOA->ODR &= ~ALL_GREEN_PINS;
    GPIOA->ODR |= ALL_RED_PINS;
		HAL_Delay(2000);
	
}

//ALL LED ON for 2s
void ALL_LED_ON(void) {

		GPIOA->ODR |= ALL_LED_MASK;
		HAL_Delay(2000);
	
}


// Red-ON Green-OFF
void R_LED_ON(int group) {
    switch(group) {
        case 1:  // A
            GPIOA->ODR &= ~G_LED1_PIN;  // G OFF
            GPIOA->ODR |= R_LED1_PIN;   // R ON
            break;
        case 2:  // B
            GPIOA->ODR &= ~G_LED2_PIN;  // G OFF
            GPIOA->ODR |= R_LED2_PIN;   // R ON
            break;
        case 3:  // C
            GPIOA->ODR &= ~G_LED3_PIN;  // G OFF
            GPIOA->ODR |= R_LED3_PIN;   // R ON
            break;
        case 4:  // D
            GPIOA->ODR &= ~G_LED4_PIN;  // G OFF
            GPIOA->ODR |= R_LED4_PIN;   // R ON
            break;
    }
}

// Red-OFF Green-ON
void R_LED_OFF(int group) {
    switch(group) {
        case 1:  // A
            GPIOA->ODR &= ~R_LED1_PIN;  // R OFF
            GPIOA->ODR |= G_LED1_PIN;   // G ON
            break;
        case 2:  // B
            GPIOA->ODR &= ~R_LED2_PIN;  // R OFF
            GPIOA->ODR |= G_LED2_PIN;   // G ON
            break;
        case 3:  // C
            GPIOA->ODR &= ~R_LED3_PIN;  // R OFF
            GPIOA->ODR |= G_LED3_PIN;   // G ON
            break;
        case 4:  // D
            GPIOA->ODR &= ~R_LED4_PIN;  // R OFF
            GPIOA->ODR |= G_LED4_PIN;   // G ON
            break;
    }
}

// Green-ON Red-OFF
void G_LED_ON(int group) {
    switch(group) {
        case 1:  // A
            GPIOA->ODR &= ~R_LED1_PIN;  // R OFF
            GPIOA->ODR |= G_LED1_PIN;   // G ON
            break;
        case 2:  // B
            GPIOA->ODR &= ~R_LED2_PIN;  // R OFF
            GPIOA->ODR |= G_LED2_PIN;   // G ON
            break;
        case 3:  // C
            GPIOA->ODR &= ~R_LED3_PIN;  // R OFF
            GPIOA->ODR |= G_LED3_PIN;   // G ON
            break;
        case 4:  // D
            GPIOA->ODR &= ~R_LED4_PIN;  // R OFF
            GPIOA->ODR |= G_LED4_PIN;   // G ON
            break;
    }
}

// Green-OFF Red-ON
void G_LED_OFF(int group) {
    switch(group) {
        case 1:  // A
            GPIOA->ODR &= ~G_LED1_PIN;  // G OFF
            GPIOA->ODR |= R_LED1_PIN;   // R ON
            break;
        case 2:  // B
            GPIOA->ODR &= ~G_LED2_PIN;  // G OFF
            GPIOA->ODR |= R_LED2_PIN;   // R ON
            break;
        case 3:  // C
            GPIOA->ODR &= ~G_LED3_PIN;  // G OFF
            GPIOA->ODR |= R_LED3_PIN;   // R ON
            break;
        case 4:  // D
            GPIOA->ODR &= ~G_LED4_PIN;  // G OFF
            GPIOA->ODR |= R_LED4_PIN;   // R ON
            break;
    }
}

//Blink G LED
void blink(int group) {
    switch(state) {
				case 0:  // normal:blink once
            // G OFF 0.2s
            switch(group) {
                case 1: GPIOA->ODR &= ~G_LED1_PIN; break;
                case 2: GPIOA->ODR &= ~G_LED2_PIN; break;
                case 3: GPIOA->ODR &= ~G_LED3_PIN; break;
                case 4: GPIOA->ODR &= ~G_LED4_PIN; break;
            }
            HAL_Delay(200);
            
            // G ON 0.2s
            switch(group) {
                case 1: GPIOA->ODR |= G_LED1_PIN; break;
                case 2: GPIOA->ODR |= G_LED2_PIN; break;
                case 3: GPIOA->ODR |= G_LED3_PIN; break;
                case 4: GPIOA->ODR |= G_LED4_PIN; break;
            }
            HAL_Delay(200);
            break;
            
				case 1:  // busy:blink twice
            for(int i = 0; i < 2; i++) {
                //G OFF 0.2s
                switch(group) {
                    case 1: GPIOA->ODR &= ~G_LED1_PIN; break;
                    case 2: GPIOA->ODR &= ~G_LED2_PIN; break;
                    case 3: GPIOA->ODR &= ~G_LED3_PIN; break;
                    case 4: GPIOA->ODR &= ~G_LED4_PIN; break;
                }
                HAL_Delay(500);
                
                // G ON 0.2s
                switch(group) {
                    case 1: GPIOA->ODR |= G_LED1_PIN; break;
                    case 2: GPIOA->ODR |= G_LED2_PIN; break;
                    case 3: GPIOA->ODR |= G_LED3_PIN; break;
                    case 4: GPIOA->ODR |= G_LED4_PIN; break;
                }
                HAL_Delay(500);
            }
            break;
            
				// case 2 (carless) :no blink
    }
}
		

void emergency(int group)
{		
		OLED_ShowString(4, 4, "Acrossing! ");
		R_LED_ON(group);
}


// Read IR1
uint8_t IR1(int group) {
    uint16_t pin;
    switch(group) {
        case 1: pin = IR_A1_PIN; break;  // A
        case 2: pin = IR_B1_PIN; break;  // B
        case 3: pin = IR_C1_PIN; break;  // C
        case 4: pin = IR_D1_PIN; break;  // D
        default: return 0;
    }
    // No car-1; Have car-0
    return (GPIOC->IDR & pin) ? 1 : 0;
}

// Read IR2
uint8_t IR2(int group) {
    uint16_t pin;
    switch(group) {
        case 1: pin = IR_A2_PIN; break;  // A
        case 2: pin = IR_B2_PIN; break;  // B
        case 3: pin = IR_C2_PIN; break;  // C
        case 4: pin = IR_D2_PIN; break;  // D
        default: return 0;
    }
    // No car-1; Have car-0
    return (GPIOC->IDR & pin) ? 1 : 0;
}

// Prediction
void Predict(int group) {
		
    uint8_t sensor1 = IR1(group);
    uint8_t sensor2 = IR2(group);
    
    // Both car detected
    if (sensor1 == 0 && sensor2 == 0) {
        state = 1;  // Set busy mode
    } 
		else if (sensor1 == 1 && sensor2 == 1) {
        state = 2;  // Set no car mode
    } 
    // default
    else {
        state = 0;  // Set normal mode
    }
}


// Normal state
void normal(int group) {
    G_LED_ON(group);      // G ON
		HAL_Delay(200);
		blink(group);
    HAL_Delay(5000);      // 5s
    R_LED_ON(group);      // R ON
}

// Busy state
void busy(int group) {
    uint8_t count = 0;
    uint32_t total_time = 5000;
    
    G_LED_ON(group);  // G ON
		HAL_Delay(200);
		blink(group);
    
    // Delay 5s
    HAL_Delay(5000);
    
    // cheak IR to expand time
    while (count < 5) {  // max 5 times
        if (!IR1(group)) {  // IR=0 car detected
            HAL_Delay(200);  // delay 0.2s
            if (!IR1(group)) {  // detect IR1 again
                HAL_Delay(2000);  // total time +2s
                total_time += 2000;
                count++;
                
                // cheack max time=15s?
                if (total_time >= 15000) break;
            } else {
                break;  // 2nd detect "1" means it's a mistake
            }
        } else {
            break;  // 1st detect "1" means not busy
        }
    }
    
    R_LED_ON(group);  // R ON
}

// No Car
void carless(int group) {
	
    G_LED_ON(group);      // G ON
		//no blink
    HAL_Delay(3000);      // 3s
    R_LED_ON(group);      // R ON
	
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	

		
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
	__disable_irq();
	
	// GPIOA, GPIOB, GPIOC, GPIOH clk
	RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 7);
	
	//GPIOA: ALL LED output
	GPIOA->MODER = 1 << (6*2) | 1 << (7*2) | 1 << (8*2) | 1 << (9*2) | 1 << (10*2) | 1 << (11*2) | 1 << (12*2) | 1 << (15*2);
	GPIOA->OTYPER = 0;
	GPIOA->PUPDR = 0;
	
	//GPIOC: ALL IR input
	GPIOC->MODER = 0;
	GPIOC->PUPDR = 0;
	
	//GPIOB: SCL-PB13/SDA-PB14 output, botton(PB4-7) input
	GPIOB->MODER = 1 << (13*2) | 1 << (14*2);
	GPIOB->OTYPER = 0;
	GPIOB->PUPDR = 1 << (13*2) | 1 << (14*2) | 1 << (4*2) | 1 << (5*2) | 1 << (6*2) | 1 << (7*2);
	
	//Interrupt
	RCC->APB2ENR |= 0x4000;
	// EXTI4 (PB4)
	SYSCFG->EXTICR[1] |= 0x0001;
	EXTI->IMR |= 0x0010;
	EXTI->FTSR |= 0x0010;
	
	// EXTI5 (PB5)
	SYSCFG->EXTICR[1] |= 0x0010;
	EXTI->IMR |= 0x0020;
	EXTI->FTSR |= 0x0020;

	// EXTI6 (PB6)
	SYSCFG->EXTICR[1] |= 0x0100;
	EXTI->IMR |= 0x0040;
	EXTI->FTSR |= 0x0040;

	// EXTI7 (PB7)
	SYSCFG->EXTICR[1] |= 0x1000;
	EXTI->IMR |= 0x0080;
	EXTI->FTSR |= 0x0080;
	
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	__enable_irq();
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	OLED_Init();
	OLED_ShowString(2,2,"Initializing...");
	HAL_Delay(1000);

	
	int group = 1;
	
	ALL_LED_ON();
  ALL_RED_ON();
	
	OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
		for(;group<=4;group++){
			
			current_group = group;
			
			Predict(group);
			
			current_state = state;
			oled_display();
			
			switch(state) {
				case 0:  // normal
					normal(group);
					break;
				case 1:  // busy
					busy(group);
					break;
				case 2:  // no car
					carless(group);
					break;
				default: // normal
					normal(group);
					break;}
			OLED_Clear();
		}
		group = 1;
		
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void EXTI4_IRQHandler(void)
{
	
	if (EXTI->PR & 1<<4) {
		
		EXTI->PR = 1<<4;

		// shake inspection
		uint32_t now = HAL_GetTick();
		if (now - last_button_press < 300) {
			return;
		}
		last_button_press = now;
		
		// if "R ON", do nothing
		if (GPIOA->ODR & R_LED1_PIN) {
			return;
		}
		
		// if "G ON", turn to RED and display
		if (GPIOA->ODR & G_LED1_PIN) {
			emergency(1);
		}
	}

}

void EXTI9_5_IRQHandler(void)
{
	uint32_t now = HAL_GetTick();
	//GroupB
	if (EXTI->PR & 1<<5) {
	
		EXTI->PR = 1<<5;

		// shake inspection
		uint32_t now = HAL_GetTick();
		if (now - last_button_press < 300) {
			return;
		}
		last_button_press = now;
		
		// if "R ON", do nothing
		if (GPIOA->ODR & R_LED2_PIN) {
			return;
		}
		
		// if "G ON", turn to RED and display
		if (GPIOA->ODR & G_LED2_PIN) {
			emergency(2);
		}
	}
	
	//GroupC
	if (EXTI->PR & 1<<6) {

		EXTI->PR = 1<<6;

		// shake inspection
		uint32_t now = HAL_GetTick();
		if (now - last_button_press < 300) {
			return;
		}
		last_button_press = now;
	
		// if "R ON", do nothing
		if (GPIOA->ODR & R_LED3_PIN) {
			return;
		}
	
		// if "G ON", turn to RED and display
		if (GPIOA->ODR & G_LED3_PIN) {
			emergency(3);
		}
	}
	
	//GroupD
	if (EXTI->PR & 1<<7) {

		EXTI->PR = 1<<7;

		// shake inspection
		uint32_t now = HAL_GetTick();
		if (now - last_button_press < 300) {
			return;
		}
		last_button_press = now;
		
		// if "R ON", do nothing
		if (GPIOA->ODR & R_LED4_PIN) {
			return;
		}
		
		// if "G ON", turn to RED and display
		if (GPIOA->ODR & G_LED4_PIN) {
			emergency(4);
		}
	}

}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
