  /******************************************************
   * ************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "C:\Users\goshe\Personal\NUS Matters\Y2S1\EE2028\Workspace\Assignment2\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_gyro.h"
#include "C:\Users\goshe\Personal\NUS Matters\Y2S1\EE2028\Workspace\Assignment2\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_hsensor.h"
#include "C:\Users\goshe\Personal\NUS Matters\Y2S1\EE2028\Workspace\Assignment2\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_magneto.h"
#include "C:\Users\goshe\Personal\NUS Matters\Y2S1\EE2028\Workspace\Assignment2\Drivers\BSP\B-L475E-IOT01\stm32l475e_iot01_psensor.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#define HEALTHY 0
#define INTENSIVE 1
#define GYRO_THRESHOLD 200.0
#define TEMP_THRESHOLD 37.6
#define MAG_THRESHOLD 0.120
#define HUM_THRESHOLD 70.0
#define WARNING 1
#define SAFE 0

//extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void MX_GPIO_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void accelero_interrupt_config(void);
void delay(uint32_t time);
volatile extern int delaytime, button_timer; //variable for delay func
void pressure_interrupt_config(void);
void mode_switch(void);
static void UART1_Init(void);
void FlagsToZero(void);
//variables for button mode switch
int button_count = 0, mode = HEALTHY, count = 0;
//flags for warnings
volatile short int accflag, gyroflag, pressureflag, tempflag, magflag, humidflag;

UART_HandleTypeDef huart1;// struct for uart1
char message_print[50]; //array for uart transmission

int main(void)
{
	int startcount, endcount, difference;
	int intensive_state, timer_10s, count_healthy, timer_healthy_10s;
	float Mag_Baseline[3], Mag_Difference[3];
	float accel_data[3], gyro_data[3], magneto_data[3];
	float temp_data, hum_data;
	int16_t accel_data_i16[3] = { 0 };	// array to store the x, y and z readings.
	float gyro_data_i16[3] = { 0 };	//array to store gyro ODR data
	float gyro_total;
	float p_data;
	int16_t magneto_data_i16[3] = { 0 };

	 //initialize to 0
	count_healthy = 0;
	//initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	//initialise GPIO Pin for LED2 and push button
	MX_GPIO_Init();
	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	UART1_Init();

	//Peripheral interrupt config
	accelero_interrupt_config();
	pressure_interrupt_config();

	//enable exti interrupt
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);		//enable nvic

	while (1)
	{

		intensive_state = count % 4; 				//timer for 1s used to poll in intensive mode
		timer_10s = count % 40;						//timer for 10s used to transmit in intensive mode
		timer_healthy_10s = count_healthy % 40;		//timer for 10s used to transmit in healthy mode

		//intensive mode functions
		if (mode == INTENSIVE){
			startcount = HAL_GetTick();

			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

			if (intensive_state == 0){
				//Accelerometer
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
				// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
				accel_data[0] = ((float)accel_data_i16[0] / 100.0f) / 9.8f;
				accel_data[1] = ((float)accel_data_i16[1] / 100.0f) / 9.8f;
				accel_data[2] = ((float)accel_data_i16[2] / 100.0f) / 9.8f;

				//temperature and humidity
				temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
				hum_data = BSP_HSENSOR_ReadHumidity();		//read humidity sensor

				//gyroscope
				BSP_GYRO_GetXYZ(gyro_data_i16);		//read gyro
				//convert gyro data into meaningful value in terms of dps;
				gyro_data[0] = (gyro_data_i16[0] + 630.0f) / 1000.0f;
				gyro_data[1] = (gyro_data_i16[1] + 280.0f) / 1000.0f;
				gyro_data[2] = (gyro_data_i16[2] + 140.0f) / 1000.0f ;
				//get root mean square value
				gyro_total = sqrt(pow(gyro_data[0],2)+pow(gyro_data[1],2)+pow(gyro_data[2],2));

				//magnetometer
				BSP_MAGNETO_GetXYZ(magneto_data_i16);	//read magneto
				//convert magneto data into meaningful value in terms of gauss
				magneto_data[0] = (float)magneto_data_i16[0] / 1000.0f;
				magneto_data[1] = (float)magneto_data_i16[1] / 1000.0f;
				magneto_data[2] = (float)magneto_data_i16[2] / 1000.0f;
				if (count == 0){						//set the baseline position when first entered intensive mode
					Mag_Baseline[0] = magneto_data[0];
					Mag_Baseline[1] = magneto_data[1];
					Mag_Baseline[2] = magneto_data[2];
				}
				for (int i = 0; i < 3; i++)
					Mag_Difference[i] = fabs(Mag_Baseline[i]-magneto_data[i]);

				//pressure
				p_data = BSP_PSENSOR_ReadPressure()*100.0f;	//pressure in pascal

				//raising flags for warnings (pressure, gyro and accelero done through INT)

				if(Mag_Difference[0] >= MAG_THRESHOLD || Mag_Difference[1] >= MAG_THRESHOLD || Mag_Difference[2] >= MAG_THRESHOLD)
					magflag = WARNING;
				if (temp_data >= TEMP_THRESHOLD)
					tempflag = WARNING;
				if (hum_data <= HUM_THRESHOLD)
					humidflag = WARNING;
			}

			endcount = HAL_GetTick();

			if (timer_10s == 0){
				//readings transmission
				sprintf(message_print, "%03d TEMP %0.2f ACC %0.2f %0.2f %0.2f\r\n", count/40, temp_data, accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				sprintf(message_print, "%03d GYRO %0.1f MAGNETO %0.2f %0.2f %0.2f\r\n", count/40, gyro_total, magneto_data[0], magneto_data[1], magneto_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				sprintf(message_print, "%03d HUMIDITY %0.2f and BARO %0.2f\r\n",count/40,  hum_data, p_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				//warnings transmission
				if (tempflag == WARNING){
					sprintf(message_print, "Fever is detected\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					tempflag = SAFE;
				}
				if (accflag == WARNING){
					sprintf(message_print, "Falling is detected\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					accflag = SAFE;
				}
				if (gyroflag == WARNING){
					sprintf(message_print, "Patient in pain!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					gyroflag = SAFE;
				}
				if (pressureflag == WARNING || humidflag == WARNING){
					sprintf(message_print, "Check patient's breath!\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					pressureflag = SAFE;
					humidflag = SAFE;
				}
				if (magflag == WARNING){
					sprintf(message_print, "Check patient's orientation\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					magflag = SAFE;
				}
			}

			difference = endcount - startcount;
			delay(250 - difference);
			count++;
		}
		if (mode == HEALTHY){
			float temp_data;
			temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
			//raise flag for temperature warning
			if (temp_data >= TEMP_THRESHOLD)
				tempflag = WARNING;

			if (tempflag == WARNING || accflag == WARNING){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			if (timer_healthy_10s == 0){
				if (tempflag == WARNING){
					sprintf(message_print, "Fever is detected\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}
				if (accflag == WARNING){
					sprintf(message_print, "Falling is detected\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}
				if(accflag == SAFE && tempflag == SAFE){
					sprintf(message_print, "All good\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}
				accflag = SAFE;
				tempflag = SAFE;
			}

			count_healthy ++;
			delay(250);
		}
	}
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitTypeDef GPIO_PushButton = {0};
  GPIO_InitTypeDef GPIO_lsm6dsl = {0};
  GPIO_InitTypeDef GPIO_lps22hb = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();		//for led2
  __HAL_RCC_GPIOC_CLK_ENABLE();		//for push button
  __HAL_RCC_GPIOD_CLK_ENABLE();		//for lsm6dsl and lps22hb
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BUTTON_EXTI13_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, LSM6DSL_INT1_EXTI11_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, LPS22HB_INT_DRDY_EXTI0_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  //configure GPIO pin BUT_EXTI13
  GPIO_PushButton.Pin = BUTTON_EXTI13_Pin;
  GPIO_PushButton.Mode = GPIO_MODE_IT_FALLING;
  GPIO_PushButton.Pull = GPIO_NOPULL;
  GPIO_PushButton.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_PushButton);
  //configure GPIO Pin lsm6dsl
  GPIO_lsm6dsl.Pin = LSM6DSL_INT1_EXTI11_Pin;
  GPIO_lsm6dsl.Mode = GPIO_MODE_IT_RISING;
  GPIO_lsm6dsl.Pull = GPIO_NOPULL;
  GPIO_lsm6dsl.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_lsm6dsl);
  //configure GPIO Pin lps22hb
  GPIO_lps22hb.Pin = LPS22HB_INT_DRDY_EXTI0_Pin;
  GPIO_lps22hb.Mode = GPIO_MODE_IT_RISING;
  GPIO_lps22hb.Pull = GPIO_NOPULL;
  GPIO_lps22hb.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_lps22hb);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == BUTTON_EXTI13_Pin){
		mode_switch();
	}

	if (GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin){
		uint8_t temp;
		temp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW+1, LSM6DSL_ACC_GYRO_WAKE_UP_SRC);
		temp &= 0x20; //read bit[5] to determine if FF flag was raised by device
		if (temp)
			accflag = WARNING;
		else
			gyroflag = WARNING;
	}

	if (GPIO_Pin == LPS22HB_INT_DRDY_EXTI0_Pin){
		if (mode == INTENSIVE){
			pressureflag = WARNING;
		}
	}
}

/* Configure interrupt for accelero and gyro */
void accelero_interrupt_config(void){
	uint8_t Buffer[1];
	Buffer[0] = 0x80; //set bit[7] to enable basic interrupts
	SENSOR_IO_WriteMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, Buffer, 1);
	Buffer[0] = 0x08; //FF_Dur [4:0] = 00001, FF_Ths [2:0] = 000
	SENSOR_IO_WriteMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, Buffer, 1);
	Buffer [0] = 0;
	Buffer[0] |= (1U << 4 | 1U << 2); //set bit[4] to route FF interrupt to INT1, set bit[2] to route Gyro INT
	SENSOR_IO_WriteMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, Buffer, 1);
}

/** Delay function
 *  param: time = time in milliseconds**/
void delay(uint32_t time){
	delaytime = time;
	while (delaytime != 0){
	}
}

/** Configure interrupt for pressure sensor **/
void pressure_interrupt_config(void){
	uint8_t Buffer[1];
	Buffer[0] = 0x29;	//set autozero = 0x2X, DIFF_EN and PHE to enable interrupt and pressure high event = 0xX9
	SENSOR_IO_WriteMultiple(0xBA, LPS22HB_INTERRUPT_CFG_REG, Buffer, 1);
	//set upper threshold of 50 Pascal
	//Threshold formula: (hPA) = THS_P / 16
	Buffer[0] = 0x08;	//setting LSB for upper threshold
	SENSOR_IO_WriteMultiple(0xBA, LPS22HB_THS_P_LOW_REG, Buffer, 1);
	Buffer[0] = 0x00;	//setting MSB for upper threshold
	SENSOR_IO_WriteMultiple(0xBA, LPS22HB_THS_P_HIGH_REG, Buffer, 1);
	Buffer[0] = 0x01;
	SENSOR_IO_WriteMultiple(0xBA, LPS22HB_CTRL_REG3, Buffer, 1); //allow for high pressure event on INT_DRDY pin
	return;
}

/** Switch function for USER PUSH BUTTON **/
void mode_switch(void){
	//timeout function of 1s
	if (button_timer >= 1000){
		button_count = 0;
		button_timer = 0;
	}

	button_count++;

	if (button_count == 1 && mode != INTENSIVE){
		button_timer = 0;
	}

	if (button_count == 1 && mode == INTENSIVE){
		mode = HEALTHY;
		button_count = 0;
		sprintf(message_print, "Switched to healthy mode\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
		FlagsToZero();
	}

	if (button_count == 2){
		mode = INTENSIVE;
		button_count = 0;
		count = 0;
		sprintf(message_print, "Switched to intensive mode\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		FlagsToZero();
	}
}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}

/** Sets all flags to SAFE **/
void FlagsToZero(void){
	accflag = SAFE;
	gyroflag = SAFE;
	pressureflag = SAFE;
	tempflag = SAFE;
	magflag = SAFE;
	humidflag = SAFE;
}
