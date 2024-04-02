#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#define DESIRED_DISTANCE 10
#define kP 2.5
#define kI 0
#define kD 3.5
#define MIN_DISTANCE_USS 2
#define MAX_DISTANCE_IR 100
#define MAXERROR 100

enum Speed
{
  fast,
  medium,
  slow
} speed;

enum DrivingState
{
  STOP,
  STRAIGHT,
  LEFT,
  RIGHT,
  SHARP_LEFT,
  SHARP_RIGHT
};

// Motor speed values
const int STOPTURNING = 0;
const int TURN_LEFT_LEFT_MOTOR = 78;
const int TURN_LEFT_RIGHT_MOTOR = 67;
const int TURN_SHARP_LEFT_RIGHT_MOTOR = 67;

const int TURN_RIGHT_LEFT_MOTOR = 82;
const int TURN_RIGHT_RIGHT_MOTOR = 73;
const int TURN_SHARP_RIGHT_LEFT_MOTOR = 83;

const int STRAIGHT_SLOW_LEFT_MOTOR = 74;
const int STRAIGHT_SLOW_RIGHT_MOTOR = 77;

const int STRAIGHT_MEDIUM_LEFT_MOTOR = 76;
const int STRAIGHT_MEDIUM_RIGHT_MOTOR = 75;

const int STRAIGHT_FAST_LEFT_MOTOR = 78;
const int STRAIGHT_FAST_RIGHT_MOTOR = 71;
int straightRightMotorSpeed = 77;
int straightLeftMotorSpeed = 74;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

double prevError;
double integral = 0;
double derivative = 0;

volatile int risingEdge = 1;
volatile uint64_t risingEdgeTimeStamp = 0;
volatile int calculateDistance = 0;

double errorDistance = 0;
double prevErrorDistance = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

uint8_t data = 0;

void RightServoPinConfig()
{
  // Enable GPIOA
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Set the pin A1 to alternate function 1
  GPIOA->MODER |= (GPIOA->MODER & ~GPIO_MODER_MODER1) | (0b10 << GPIO_MODER_MODER1_Pos);
  GPIOA->AFR[0] |= (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL1) | (0b01 << GPIO_AFRL_AFRL1_Pos);
}

void LeftServoPinConfig()
{
  // Enable GPIOA on bus
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Set the pin A0 to alternate function 1
  GPIOA->MODER |= (GPIOA->MODER & ~GPIO_MODER_MODER0) | (0b10 << GPIO_MODER_MODER0_Pos);
  GPIOA->AFR[0] |= (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL0) | (0b01 << GPIO_AFRL_AFRL0_Pos);
}

void UltraSonicPinConfig()
{
  // Enable GPIOA on bus
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Set the pin B5 to alternate function 2
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER5) | (0b0010 << GPIO_MODER_MODER5_Pos);
  GPIOB->AFR[0] |= (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL5) | (0b0010 << GPIO_AFRL_AFRL5_Pos);

  // Set up pin B6 to alternate function 2
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER6) | (0b0010 << GPIO_MODER_MODER6_Pos);
  GPIOB->AFR[0] |= (GPIOB->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos);
}

void UltraSonicTimerConfig()
{
  //==================
  // Timer 3
  //==================
  // Enable timer 3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Set psc to 71 to slow down clock to 1 microsecond
  TIM3->PSC = 72 - 1;
  TIM3->ARR = 0xFFFF;

  // Set up duty cycle to 10 microseconds
  TIM3->CCR2 = 10;

  // Set up output of timer to PWM mode 1
  TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM3->CCER |= TIM_CCER_CC2E; // enable channel 2

  // Reset and enable counter
  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;

  //==================
  // Timer 4
  //==================
  // Enable timer 4
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  // Set psc to 71 to slow down clock to 1 microsecond
  TIM4->PSC = 72 - 1;
  TIM4->ARR = 0xFFFF;

  // Set up the timer for input on IC1
  TIM4->CCMR1 |= 0b0001;
  TIM4->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC1P; // Channel set as input on both edges

  TIM4->CCER |= TIM_CCER_CC1E; // Enable channel 1

  // Enable interrupt for timer 4
  TIM4->DIER = TIM_DIER_CC1IE;
  NVIC_EnableIRQ(TIM4_IRQn);

  // clear interrupt flag for the timer
  TIM4->SR = ~TIM_SR_CC1IF;

  // reset and enable the counter
  TIM4->CNT = 0;
  TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(void)
{

  // Was interrupt caused by timer 4 channel 1?
  if (TIM4->SR & TIM_SR_CC1IF)
  {

    // Clear the interrupt flag
    TIM4->SR &= ~TIM_SR_CC1IF;

    if (risingEdge)
    {
      // Store time stamp of rising edge
      risingEdgeTimeStamp = TIM4->CCR1;

      risingEdge = 0;
    }
    else
    {
      uint64_t dutyCycle = TIM4->CCR1 - risingEdgeTimeStamp;

      // Calculate distance in cm
      calculateDistance = dutyCycle / 58;
      risingEdge = 1;
    }
  }
}

void InfraredPinsConfig()
{
  // Enable clock for pins and set them for input pull-down
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Left
  GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3_Msk;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR3_1;

  // L-Center
  GPIOC->MODER &= ~GPIO_MODER_MODER1_Msk;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_Msk;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_1;

  // Center
  GPIOC->MODER &= ~GPIO_MODER_MODER2_Msk;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_Msk;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_1;

  // R-Center
  GPIOC->MODER &= ~GPIO_MODER_MODER3_Msk;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_Msk;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_1;

  // Right
  GPIOC->MODER &= ~GPIO_MODER_MODER7_Msk;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_Msk;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_1;
}

void ServoTimerConfig()
{
  // Enable timer 2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Set the prescaler and auto reload register
  TIM2->PSC = 1440;
  TIM2->ARR = (1000 - 1);

  // Set the counter to up counting
  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;

  // Set up the timer for mode 1 pwm output
  TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
  TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

  // Enable the output for channel 2 and 1
  TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E;
}

int DetermineState()
{
  // Read values of all IR sensors
  uint8_t valueIRLeft = (GPIOC->IDR & GPIO_IDR_0) >> 0;
  uint8_t valueIRLeftCenter = (GPIOC->IDR & GPIO_IDR_1) >> 1;
  uint8_t valueIRRight = (GPIOC->IDR & GPIO_IDR_7) >> 7;
  uint8_t valueIRRightCenter = (GPIOC->IDR & GPIO_IDR_3) >> 3;
  uint8_t valueIRCenter = (GPIOC->IDR & GPIO_IDR_2) >> 2;

  // Determine what state the robot is in according to certain combinations of IR sensor values
  if (calculateDistance < 10 || (valueIRLeft == 0 && valueIRLeftCenter == 0 && valueIRRight == 0 && valueIRRightCenter == 0 && valueIRCenter == 0) || (valueIRLeft == 1 && valueIRLeftCenter == 1 && valueIRRight == 1 && valueIRRightCenter == 1 && valueIRCenter == 1))
  {
    // HAL_UART_Transmit(&huart2, "STOP\n", 5, 0xFFFF);
    return STOP;
  }

  else if (valueIRLeftCenter == 0)
  {
    // HAL_UART_Transmit(&huart2, "L\n", 2, 0xFFFF);
    return LEFT;
  }

  else if (valueIRRightCenter == 0)
  {
    // HAL_UART_Transmit(&huart2, "R\n", 2, 0xFFFF);
    return RIGHT;
  }

  else if (valueIRLeft == 0)
  {
    // HAL_UART_Transmit(&huart2, "SL\n", 3, 0xFFFF);
    return SHARP_LEFT;
  }

  else if (valueIRRight == 0)
  {
    // HAL_UART_Transmit(&huart2, "SR\n", 3, 0xFFFF);
    return SHARP_RIGHT;
  }

  else
  {
    // HAL_UART_Transmit(&huart2, "Straight\n", 10, 0xFFFF);
    return STRAIGHT;
  }
}

int CalculatePIDUltraSonic(int setPointDistance)
{
  errorDistance = calculateDistance - setPointDistance;
  // char buffer[12];

  // sprintf(buffer, "%f \n", errorDistance);
  // HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

  integral = integral + errorDistance;

  if (errorDistance < 0 || errorDistance > setPointDistance)
  {
    if (errorDistance < MIN_DISTANCE_USS)
    {
      integral = 0;
    }
  }

  derivative = errorDistance - prevErrorDistance;
  prevErrorDistance = errorDistance;

  double power = errorDistance * kP + integral * kI + derivative * kD;

  return power;
}

double CalculateAdditionalPower(double power)
{
  double additionalPower = (0.08 * power);
  return additionalPower;
}

void ControlMotor_Left(double value, double additionalPower)
{
  value += additionalPower - speed;
  // Left wheel (stop 74-76, slowest 77, fastest 84)
  // Set duty of a timer
  TIM2->CCR1 = (value);
}

void ControlMotor_Right(double value, double additionalPower)
{
  value += (-1.00 * (additionalPower + speed));
  // Right wheel (stop 74 - 76, slowest 73, fastest 66)
  // Set duty of a timer
  TIM2->CCR2 = (value);
}

void UpdateUserSpeed()
{
  // Check for message in UART
  HAL_UART_Receive(&huart2, &data, 1, 0);
  if (data == 's')
  {
    straightLeftMotorSpeed = STRAIGHT_SLOW_LEFT_MOTOR;
    straightRightMotorSpeed = STRAIGHT_SLOW_RIGHT_MOTOR;
  }
  else if (data == 'm')
  {
    straightLeftMotorSpeed = STRAIGHT_MEDIUM_LEFT_MOTOR;
    straightRightMotorSpeed = STRAIGHT_MEDIUM_RIGHT_MOTOR;
  }
  else if (data == 'f')
  {
    straightLeftMotorSpeed = STRAIGHT_FAST_LEFT_MOTOR;
    straightRightMotorSpeed = STRAIGHT_FAST_RIGHT_MOTOR;
  }
}

void sendDistanceToESP(){
  const int MSGBUFSIZE = 80;
  char msgBuf[MSGBUFSIZE];
  char receivedCommand[15] = ""; 

  USART1->ICR |= USART_ICR_ORECF;
  HAL_StatusTypeDef status1 = (HAL_UART_Receive(&huart1, (uint8_t*) receivedCommand, 1, 0));
  if (status1 == HAL_OK) {
    int char1 = atoi(receivedCommand);
    if(char1 == 1){
      snprintf(msgBuf, 15, "%d", calculateDistance);
      HAL_UART_Transmit(&huart1, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);    
    }
  }
}

void configure_extern_UART(void) {
    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure USART1 Tx (PA9) and Rx (PA10) pins
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // Use AF7 for USART1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable USART1 clock
    __HAL_RCC_USART1_CLK_ENABLE();

    // Configure USART1 settings
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    // Initialize USART1
    HAL_UART_Init(&huart1);
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  configure_extern_UART();


  UltraSonicPinConfig();
  UltraSonicTimerConfig();

  InfraredPinsConfig();

  LeftServoPinConfig();
  RightServoPinConfig();
  ServoTimerConfig();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    double power = CalculatePIDUltraSonic(DESIRED_DISTANCE);
    double additionalPower = CalculateAdditionalPower(power);

    //UpdateUserSpeed();  

    enum DrivingState state = DetermineState();

    sendDistanceToESP();

    switch (state)
    {
    case STOP:
      ControlMotor_Left(STOPTURNING, 0);
      ControlMotor_Right(STOPTURNING, 0);
      break;
    case STRAIGHT:

      ControlMotor_Left(STRAIGHT_FAST_LEFT_MOTOR, additionalPower);
      ControlMotor_Right(STRAIGHT_FAST_RIGHT_MOTOR, additionalPower);
      break;
    case LEFT:
      ControlMotor_Left(TURN_LEFT_LEFT_MOTOR, 0);
      ControlMotor_Right(TURN_LEFT_RIGHT_MOTOR, 0);
      break;
    case RIGHT:
      ControlMotor_Left(TURN_RIGHT_LEFT_MOTOR, 0);
      ControlMotor_Right(TURN_RIGHT_RIGHT_MOTOR, 0);
      break;
    case SHARP_LEFT:
      ControlMotor_Left(STOPTURNING, 0);
      ControlMotor_Right(TURN_SHARP_LEFT_RIGHT_MOTOR, 0);
      break;
    case SHARP_RIGHT:
      ControlMotor_Left(TURN_SHARP_RIGHT_LEFT_MOTOR, 0);
      ControlMotor_Right(STOPTURNING, 0);
      break;
    }
  HAL_Delay(20);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef USE_FULL_ASSERT
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
