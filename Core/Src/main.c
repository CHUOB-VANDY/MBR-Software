/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "nRF24L01.h"
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t RxpipeAddrs = 0x11223344AA;
uint8_t received_byte[2];
// Global variable for MUP_6050

float pid_p_gain = 22;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.3;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 66;                                       //Gain setting for the D-controller (30)
float turning_speed = 10;                                    //Turning speed (20)
float max_target_speed = 100;                                //Max target speed (100)


int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint=0, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;


void TIM3_IRQHandler(void){


	if ( TIM3->SR & TIM_SR_UIF ){

		throttle_counter_left_motor++;
		if ( throttle_counter_left_motor > throttle_left_motor_memory ){
			throttle_counter_left_motor=0;
			throttle_left_motor_memory = throttle_left_motor;
			if ( throttle_left_motor_memory < 0 ){
				GPIOB->BRR |= ( 1<<12 );
				throttle_left_motor_memory *=-1;
			}else{
				GPIOB->BSRR |= ( 1<<12 );
			}
		}
		else if ( throttle_counter_left_motor == 1 )GPIOB->BSRR |=(1<<1);
		else if ( throttle_counter_left_motor == 2 )GPIOB->BRR |= (1<<1);


		throttle_counter_right_motor++;
		if ( throttle_counter_right_motor > throttle_right_motor_memory ){
			throttle_counter_right_motor=0;
			throttle_right_motor_memory = throttle_right_motor;
			if ( throttle_right_motor_memory < 0 ){
				GPIOB->BRR |= ( 1<<15 );
				throttle_right_motor_memory *=-1;
			}else{
						GPIOB->BSRR |= ( 1<<15 );
			}
		}
		else if ( throttle_counter_left_motor == 1 )GPIOB->BSRR |=(1<<13);
		else if ( throttle_counter_left_motor == 2 )GPIOB->BRR |= (1<<13);
	}
	TIM3->SR &= ~TIM_SR_UIF;
}


void setup_mpu_6050_register(void){
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, (uint8_t*) 0x00, 1, 10);
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1C, 1, (uint8_t*) 0x10, 1, 10);
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1B, 1, (uint8_t*) 0x08, 1, 10);
}

void read_mpu_6050_data(void){
	uint8_t Accel_Data_Raw[6];
	HAL_I2C_Mem_Read (&hi2c1, 0x68<<1, 0x3B, 1, Accel_Data_Raw, 6, 1000);

	acc_x = (int16_t)(Accel_Data_Raw[0] << 8 | Accel_Data_Raw [1]);
	acc_y = (int16_t)(Accel_Data_Raw[2] << 8 | Accel_Data_Raw [3]);
	acc_z = (int16_t)(Accel_Data_Raw[4] << 8 | Accel_Data_Raw [5]);

	uint8_t Gyro_Data_Raw[6];
	HAL_I2C_Mem_Read (&hi2c1, 0x68<<1, 0x43, 1, Gyro_Data_Raw, 6, 1000);

	gyro_x = (int16_t)(Gyro_Data_Raw[0] << 8 | Gyro_Data_Raw [1]);
	gyro_y = (int16_t)(Gyro_Data_Raw[2] << 8 | Gyro_Data_Raw [3]);
	gyro_z = (int16_t)(Gyro_Data_Raw[4] << 8 | Gyro_Data_Raw [5]);
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  ///////////////////////////////////////////////////
  // NRF24 setup
  NRF24_begin(CSS_GPIO_Port, CSS_Pin, CEN_Pin, hspi1);
  NRF24_stopListening();
  NRF24_setAutoAck(true);
  NRF24_setChannel(68);
  NRF24_setPayloadSize(32);
  NRF24_openReadingPipe(1, RxpipeAddrs);
  NRF24_startListening();

  // GPIO Configure
  RCC->APB2ENR |= ( (1<<4) | (1<<3) );  // Enable clock for GPIO port C
  // Configure GPIOC pin 13 as output mode
  GPIOC->CRH |= ( ( 1<<20 ) | ( 1<<21 ) ); // OUTPUT mode, max speed 50Mhz
  GPIOC->CRH &= ~( ( 1<<22 ) | ( 1<<23 ) ); // General purpose output push-pull

  // Configure GPIOB for controlling stepper Motor2
  /////////////////////////////////////////////////

  GPIOB->CRL |= ( ( 1<<4 ) | ( 1<<5 ) ); // m2_step pin as output
  GPIOB->CRL &= ~( ( 1<<6 ) | ( 1<<7 ) );

  GPIOB->CRL |= ( ( 1<<8 ) | ( 1<<9 ) ); // m2_enable as output
  GPIOB->CRL &= ~( ( 1<<10 ) | ( 1<<11 ) );

  GPIOB->CRH |= ( ( 1<<16 ) | ( 1<<17) ); // m2_dir as output
  GPIOB->CRH &= ~( ( 1<<18 ) | ( 1<<19 ) );
  //////////////////////////////////////////////////


  // Configure GPIOB for controlling stepper Motor1
  /////////////////////////////////////////////////

  GPIOB->CRH |= ( ( 1<<20 ) | ( 1<<21 ) ); // m1_step pin as output
  GPIOB->CRH &= ~( ( 1<<22 ) | ( 1<<23 ) );

  GPIOB->CRH |= ( ( 1<<24 ) | ( 1<<25 ) ); // m1_enable as output
  GPIOB->CRH &= ~( ( 1<<26 ) | ( 1<<27 ) );

  GPIOB->CRH |= ( ( 1<<28 ) | ( 1<<29 ) ); // m1_dir as output
  GPIOB->CRH &= ~( ( 1<<30 ) | ( 1<<31 ) );
  /////////////////////////////////////////////////

  GPIOB->BRR |= (1<<2);
  GPIOB->BRR |= (1<<14);

  // Timer interrupt configuration

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 				// Enable clock for TIM3
  TIM3->PSC = 2-1;
  TIM3->ARR = 1800-1;
  TIM3->CR1 |= TIM_CR1_CEN; 						// Enable counter
  TIM3->DIER |= TIM_DIER_UIE; 						// Enable timer interrupt
  NVIC_EnableIRQ(TIM3_IRQn); 						// abort enable IRQ line for TIM3

  ////////////////////////////////////////////////////

  setup_mpu_6050_register();
  for ( int i=0; i< 2000; i++){
	  if ( i % 15 == 0);
	  read_mpu_6050_data();
	  gyro_x_cal += gyro_x;
	  gyro_y_cal += gyro_y;
	  gyro_z_cal += gyro_z;
	  GPIOC->ODR ^= (1<<13);
	  HAL_Delay(1);
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  ///////////////////////////////////////////////////

  HAL_TIM_Base_Start(&htim4);

  loop_timer = __HAL_TIM_GET_COUNTER(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  read_mpu_6050_data();
	  gyro_x -= gyro_x_cal;
	  gyro_y -= gyro_y_cal;
	  gyro_z -= gyro_z_cal;
	  // Gyro angle calculations
	  // 0.0000611 = 1 / (250Hz / 65.5)
	  angle_pitch += gyro_x * 0.0000611;
	  angle_roll += gyro_y * 0.0000611;
	  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
	  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);
	  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
	  //Accelerometer angle calculations
	  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
	  //57.296 = 1 / (3.142 / 180)
	  if(abs(acc_x) < acc_total_vector){
	    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
	  }
	  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
//	  angle_roll_acc -= -35.7;
//	  angle_pitch_acc -= 0.0;

	  //angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;

	  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

	  ////////////////////////////////////////////////////////////////////////////////
	  // PID controller
	  pid_error_temp = angle_roll - self_balance_pid_setpoint - pid_setpoint;
	  if ( pid_output > 10 || pid_output < -10 ) pid_error_temp += pid_output * 0.015;
	  pid_i_mem += pid_i_gain * pid_error_temp;
	  if ( pid_i_mem > 400 ) pid_i_mem = 400;
	  else if ( pid_i_mem < -400 ) pid_i_mem = -400;
	  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
	  if ( pid_output > 400 )pid_output = 400;
	  else if ( pid_output < -400 )pid_output = -400;
	  pid_last_d_error = pid_error_temp;
	  if ( pid_output < 5 && pid_output > -5 ) pid_output = 0;
	  if ( angle_roll > 30 || angle_roll < -30 ){
		  pid_output =0;
		  pid_i_mem = 0;
		  self_balance_pid_setpoint=0;
	  }
	  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
	    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
	    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
	  }

	  ///////////////////////////////////////////////////////////////
	  // Remote
	  pid_output_left = pid_output;
	  pid_output_right = pid_output;


	  if ( received_byte[0] & 0b00000001 ){
		  pid_output_left += turning_speed;
		  pid_output_right -= turning_speed;
	  }
	  if ( received_byte[0] & 0b00000010 ){
		  pid_output_left -= turning_speed;
		  pid_output_right += turning_speed;
	  }
	  if ( received_byte[0] & 0b00000100 ){
		  if ( pid_setpoint > -2.5 )pid_setpoint -=0.05;
		  if ( pid_output > max_target_speed*-1)pid_setpoint -=0.005;
	  }
	  if ( received_byte[0] & 0b00001000 ){
		  if ( pid_setpoint < 2.5 ) pid_setpoint +=0.05;
		  if ( pid_output < max_target_speed )pid_setpoint +=0.005;
	  }
	  if ( !(received_byte[0] & 0b00001100 ) ){
		  if ( pid_setpoint > 0.5 ) pid_setpoint -=0.05;
		  else if ( pid_setpoint < -0.5 ) pid_setpoint +=0.05;
		  else pid_setpoint =0;
	  }
	  ///////////////////////////////////////////////////////////////////
	  // Add these lines to make stepper run in linear

	  if ( pid_output_left > 0 )pid_output_left = 405 - (1/(pid_output_left +9)) * 5500;
	  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

	  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
	  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

	  /////////////////////////////////////////////////////////////////////


	  //Calculate the needed pulse time for the left and right stepper motor controllers
	  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
	  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
	  else left_motor = 0;


	  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
	  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
	  else right_motor = 0;

	  throttle_left_motor = left_motor;
	  throttle_right_motor = right_motor;


	  while ( __HAL_TIM_GET_COUNTER(&htim4) - loop_timer < 4000 );
	  loop_timer = __HAL_TIM_GET_COUNTER(&htim4);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
