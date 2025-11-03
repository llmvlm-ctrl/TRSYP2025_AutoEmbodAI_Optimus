#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#define MOTOR_GPIO_PORT      GPIOA


/* -----------------------------
   Motor control right (2 PWM / moteur)
   Left  : TIM1_CH1 (RPWM), TIM1_CH2 (LPWM)  
   Right : TIM2_CH1 (RPWM), TIM2_CH2 (LPWM)  
   period = 0..999
   ----------------------------- */


/*---------------------- 
   capteur ultrason centre g_d1 PC5 trigger PC4 echo 
   capteur ultrason centre g_d2 PA6 trigger PA5 echo 
  capteur ultrason centre g_d3 PA0 trigger PA1 echo 


*/
volatile uint32_t g_d1 = 0;
volatile uint32_t g_d2 = 0;
volatile uint32_t g_d3 = 0;


UART_HandleTypeDef huart6;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

osThreadId sensorTaskHandle;
osThreadId motorTaskHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void vTask_Sensor_Reading(void const * argument);
void vTask_Motor_Control(void const * argument);
void Motor_Control(void);

static inline void UART6_Send(const char *s)
{
  HAL_UART_Transmit(&huart6, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static inline uint32_t micros(void)
{
  return __HAL_TIM_GET_COUNTER(&htim5);
}

static inline void US_Trigger(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  for (volatile int i=0; i<100; ++i) __NOP();

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 10) { __NOP();}
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static uint32_t US_ReadCM(GPIO_TypeDef *trig_port, uint16_t trig_pin,
                          GPIO_TypeDef *echo_port, uint16_t echo_pin,
                          uint32_t timeout_us)
{
  US_Trigger(trig_port, trig_pin);

  uint32_t t_start = micros();
  while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET) {
    if ((uint32_t)(micros() - t_start) > timeout_us) return 0;
  }

  uint32_t t_rise = micros();
  while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
    if ((uint32_t)(micros() - t_rise) > timeout_us) return 0;
  }
  uint32_t pulse = micros() - t_rise;

  if (pulse < 100 || pulse > 30000) return 0;
  return pulse / 58U;
}


void Motor_Left_SetSpeed(uint16_t speed) {
    // forward PWM on TIM1_CH1, reverse PWM on TIM1_CH2
    if (speed > 999) speed = 999;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);  // RPWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);      // LPWM = 0
}

void Motor_Right_SetSpeed(uint16_t speed) {
    if (speed > 999) speed = 999;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);  // RPWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // LPWM = 0
}

void Motors_Forward(uint16_t speed) {
    // Left forward: TIM1_CH1 = speed, CH2 = 0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    // Right forward: TIM2_CH1 = speed, CH2 = 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed-7);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void Motors_Backward(uint16_t speed) {
    // Left backward: TIM1_CH1 = 0, CH2 = speed
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);

    // Right backward: TIM2_CH1 = 0, CH2 = speed
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

void Motors_TurnLeft(uint16_t speed) {
    // Left backward, Right forward
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void Motors_TurnRight(uint16_t speed) {
    // Left forward, Right backward
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

void Motors_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
	 // start the PWM outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PA8 - Left RPWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PA9 - Left LPWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA0 - Right RPWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PA1 - Right LPWM

  HAL_TIM_Base_Start(&htim5);
  Motors_Stop();
  UART6_Send("=== PROGRAM RUNNING ===\r\n");
  HAL_Delay(5000);


  osThreadDef(sensorTask, vTask_Sensor_Reading, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  osThreadDef(motorTask, vTask_Motor_Control, osPriorityNormal, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  osKernelStart();
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM5_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = TIM5;
  htim5.Init.Prescaler         = 16 - 1;
  htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim5.Init.Period            = 0xFFFFFFFF;
  htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Configure TIM1 Channel 1 (Left RPWM)
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure TIM1 Channel 2 (Left LPWM)
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  // Optionally configure CH3 if you want to keep it (not used here)
  // if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // TIM2 Channel 1 (Right RPWM)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // TIM2 Channel 2 (Right LPWM)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Keep other GPIO default resets for pins not used for PWM
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  // Note: We will configure PA8, PA9, PA0, PA1 as AF pins for TIM1/TIM2 PWM channels

  // Inputs used by ultrasonic sensor etc.
  GPIO_InitStruct.Pin = GPIO_PIN_6; // trigger D1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5; // echo D1
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4; // echo D2
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5; // trigger D2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
// PB0 and PB1 for the ultrason sensor 
	 GPIO_InitStruct.Pin = GPIO_PIN_0; // PB0 tirge 3
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1; // PB1 echo 3
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // --- PWM pins configuration ---
  // TIM1_CH1 -> PA8 (Left RPWM)
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TIM1_CH2 -> PA9 (Left LPWM)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TIM2_CH1 -> PA0 (Right RPWM)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TIM2_CH2 -> PA1 (Right LPWM)
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
}

void vTask_Sensor_Reading(void const * argument)
{
  for(;;)
  {
    g_d2 = US_ReadCM(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, 30000);
    g_d1 = US_ReadCM(GPIOC, GPIO_PIN_5, GPIOC, GPIO_PIN_4, 30000);
	  g_d3 = US_ReadCM(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 30000);


    char msg[64];
    int n = snprintf(msg, sizeof(msg), "D1=%lu cm  D2=%lu cm\r\n",
                     (unsigned long)g_d1, (unsigned long)g_d2);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);

    osDelay(100);
  }
}

/*void vTask_Motor_Control(void const * argument)
{
  for(;;)
  {
    

    if ((g_d1 < 50) )
    {
      Motors_Stop();


      //Motors_Backward(350);
		if (  g_d2 <20 && g_d3 > 20 )
    {
    	
    	
     Motors_TurnRight(200);
    }
		else if ( g_d3 < 20 && g_d2 > 20 )
    {
    	
      
			Motors_TurnLeft(200);
			

    }
		else if (g_d3 < 20 && g_d2 < 20)
			Motors_TurnLeft(350);
	}
		else 
			Motors_Forward(380);
			
     osDelay(200);

    }   
    
  }*/
void vTask_Motor_Control(void const * argument)
{
    static int stuck_counter = 0; // Stuck detection counter

    // --- Parameters ---
    int safe_distance_front = 50;    // front obstacle threshold
    int safe_distance_side  = 20;    // side obstacle threshold
    int speed_min           = 180;   // safe slower speeds
    int speed_max           = 999;   // max speed in open space
    int speed_turn          = 340;   // turning speed

    for(;;)
    {
        // --- Read ultrasonic sensors ---
        g_d2 = US_ReadCM(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, 30000);
        g_d1 = US_ReadCM(GPIOC, GPIO_PIN_5, GPIOC, GPIO_PIN_4, 30000);
        g_d3 = US_ReadCM(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 30000);

        // --- Debug print ---
        char msg[64];
        int n = snprintf(msg, sizeof(msg), "D1=%lu cm  D2=%lu cm D3=%lu cm\r\n",
                         (unsigned long)g_d1, (unsigned long)g_d2, (unsigned long)g_d3);
        HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);

        // --- Stuck detection ---
        if (g_d1 < safe_distance_front && g_d2 < safe_distance_side && g_d3 < safe_distance_side)
        {
            stuck_counter++;
        }
        else
        {
            stuck_counter = 0;
        }

        if (stuck_counter > 3) // robot stuck for several cycles
        {
            Motors_Backward(speed_min);
            osDelay(400);

            // Slight left turn to compensate weight drift
            Motors_TurnLeft(speed_min + 30);
            stuck_counter = 0;
            continue; // skip rest of loop
        }

        // --- Adaptive forward speed ---
        int forward_speed = speed_max;
        if (g_d1 < 60)
        {
            forward_speed = speed_min + ((g_d1 * (speed_max - speed_min)) / 100);
            if (forward_speed < speed_min) forward_speed = speed_min;
        }

        // --- Obstacle avoidance ---
        if (g_d1 < safe_distance_front)
        {
            Motors_Stop();

            //Motors_Backward(speed_min);
            //osDelay(300);

            if (g_d2 > 20 && g_d3 < 20)
            {
                Motors_TurnLeft(speed_turn);
                return;
            }
            else if (g_d3 > 20 && g_d2 < 20)
            {
                Motors_TurnLeft(speed_turn);
                return;
            }
            else if (g_d2 > 20 && g_d3 > 20)
            {
                Motors_TurnRight(speed_turn);
                return;
            }

        
        }
        else
        {
            // Path clear ? move forward at adaptive speed
            Motors_Forward(forward_speed);

          
        }

        osDelay(50); // small delay for smooth, slower reaction
    }
}

/*void Motor_Control(void)
{
   // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//



     g_d2 = US_ReadCM(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, 30000);
     g_d1 = US_ReadCM(GPIOC, GPIO_PIN_5, GPIOC, GPIO_PIN_4, 30000);
     g_d3 = US_ReadCM(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 30000);
	   char msg[64];
    int n = snprintf(msg, sizeof(msg), "D1=%lu cm  D2=%lu cm D3=%lu cm\r\n",
                     (unsigned long)g_d1, (unsigned long)g_d2,(unsigned long)g_d3);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);

    if ((g_d1 < 30) )
    {
      Motors_Stop();
      HAL_Delay(1000); // pause de 1 milliseconde


      Motors_Backward(350);
      HAL_Delay(300); // pause
			//Motors_Stop();
			 if (  g_d2 >20 && g_d3 < 20 )
    {
    	
    	
      Motors_TurnLeft(550);
			HAL_Delay(300);
			Motors_Stop();
			
    }
		else if ( g_d3 > 20 && g_d2 < 20 )
    {
    	
      
			Motors_TurnLeft(500);
		  HAL_Delay(420);
			Motors_Stop();
			 HAL_Delay(420);
		  
			

    }
		else if (g_d3 > 20 && g_d2 > 20){
			
		Motors_TurnRight(500);
			HAL_Delay(420);
			Motors_Stop();
		HAL_Delay(420);}
		
		

    }
   
    
    else
    {
      Motors_Forward(250);
    }

}*/
void Motor_Control(void)
{
    // Read ultrasonic sensors
    g_d2 = US_ReadCM(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, 30000);
    g_d1 = US_ReadCM(GPIOC, GPIO_PIN_5, GPIOC, GPIO_PIN_4, 30000);
    g_d3 = US_ReadCM(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 30000);

    // Print distances for debugging
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "D1=%lu cm  D2=%lu cm D3=%lu cm\r\n",
                     (unsigned long)g_d1, (unsigned long)g_d2, (unsigned long)g_d3);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)n, 100);

    // --- Parameters ---
    int safe_distance_front = 50;    // obstacle detection threshold
    int safe_distance_side  = 20;
    int speed_min           = 180;   // slower safe speeds
    int speed_max           = 250;
    int speed_turn          = 220;   // slower turning speed

    // --- Stuck detection: all sensors detect close obstacles ---
    static int stuck_counter = 0;
    if (g_d1 < safe_distance_front && g_d2 < safe_distance_side && g_d3 < safe_distance_side)
    {
        stuck_counter++;
    }
    else
    {
        stuck_counter = 0;
    }

    if (stuck_counter > 3) // stuck for multiple cycles
    {
        Motors_Backward(speed_min);
        HAL_Delay(400);

        // Turn slightly left to compensate right drift
        Motors_TurnLeft(speed_min + 30);
        HAL_Delay(300);

        stuck_counter = 0;
        return;
    }

    // --- Adaptive forward speed ---
    int forward_speed = speed_max;
    if (g_d1 < 100) // slow down as it approaches obstacle
    {
        forward_speed = speed_min + ((g_d1 * (speed_max - speed_min)) / 100);
        if (forward_speed < speed_min) forward_speed = speed_min;
    }

    // --- Obstacle avoidance ---
    if (g_d1 < safe_distance_front)
    {
        Motors_Stop();
        HAL_Delay(100);

        Motors_Backward(speed_min);
        HAL_Delay(300);

        if (g_d2 > 20 && g_d3 < 20)
        {
            Motors_TurnLeft(speed_turn);
            HAL_Delay(300);
        }
        else if (g_d3 > 20 && g_d2 < 20)
        {
            Motors_TurnLeft(speed_turn);
            HAL_Delay(420);
        }
        else if (g_d2 > 20 && g_d3 > 20)
        {
            Motors_TurnRight(speed_turn);
            HAL_Delay(420);
        }

        Motors_Stop();
        HAL_Delay(200);
    }
    else
    {
        // Path clear ? move forward at adaptive speed
        Motors_Forward(forward_speed);

        // Side corrections
        if (g_d2 < safe_distance_side)
            Motors_TurnRight(speed_min); // correct left wall
        else if (g_d3 < safe_distance_side)
            Motors_TurnLeft(speed_min);  // correct right wall
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
