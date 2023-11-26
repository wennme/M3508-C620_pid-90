/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "bsp_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_MAX 4 //最大电机数
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t desireRpm[MOTOR_MAX];
float desireangle[MOTOR_MAX];
MOTOR_TypeDef motor;
motor_measure_t motor_data;
float motor_err[MOTOR_MAX]; 
float motor_POS_ABS=0; //绝对位置
float pos, pos_old;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float ABS(float number)
{
	if(number<0) {return -number;}
	else {return number;}
}

void Motor_Angle_Cal(unsigned short int motor_num,float T)//电极ID与电机自传一圈角度
{
	float  res1, res2;
//	int  res3, res4;
	static float eer[MOTOR_MAX];
	
	eer[motor_num]=pos - pos_old;
	
	if(eer[motor_num]>0) 	
	{
		res1=eer[motor_num]-T;//反转，自减
		res2=eer[motor_num];
	}
	else
	{
		res1=eer[motor_num]+T;//正转，自加一个周期的角度值（360）
		res2=eer[motor_num];
	}
	
	if(ABS(res1)<ABS(res2)) //不管正反转，肯定是转的角度小的那个是真的
	{
		motor_POS_ABS += res1;
	}
	else
	{
		motor_POS_ABS += res2;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_data, rx_data);
						pos=motor_data.ecd;
					  pos_old = motor_data.last_ecd;
						Motor_Angle_Cal(i,8192);
            break;
        }
				
        default:
        {
            break;
        }
    }
}

void CAN1_control_motor(uint16_t current){
	  CAN_TxHeaderTypeDef chassis_tx_message={0};
	  uint32_t send_mail_box;
		uint8_t chassis_can_send_data[8]={0};
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = current >> 8;
    chassis_can_send_data[1] = current;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
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
	uint8_t num=0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();

  /* USER SCODE BEGIN 2 */
	can_filter_init();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	motor_err[num]=50;
	motor.PID_ANGLE[num].P=2;
	motor.PID_ANGLE[num].I=0;
	motor.PID_ANGLE[num].D=0;
	motor.PID_ANGLE[num].OUT_LIMIT=1000;
	motor.PID_ANGLE[num].I_LIMIT=400;		
	motor.PID_SPEED[num].P=2;
	motor.PID_SPEED[num].I=0;
	motor.PID_SPEED[num].D=0;
	motor.PID_SPEED[num].OUT_LIMIT=1000;
	motor.PID_SPEED[num].I_LIMIT=400;
		
  /* USER CODE END 2 */

  /* Infinite loop */
//		error[num] = desireangle[num] - motor_POS_ABS[num];
//		float pOut = kp * error[num];
//		integral += ki * error[num];
//		float iOut = integral;
//		float dOut = kd * (error[num]-lastErr[num]);
//		lastErr[num] = error[num];
//		CAN1_control_motor(pOut + iOut + dOut);
//		HAL_Delay(1);
  /* USER CODE BEGIN WHILE */
	desireangle[num]=8192/4*3591/187;//转子角度
	motor_POS_ABS=0;

  while (1)
  {
    /* USER CODE END WHILE */
		
		
    /* USER CODE BEGIN 3 */

		PID_Cal_Limt(&motor.PID_ANGLE[num], motor_err[num], motor_POS_ABS,desireangle[num]);
		PID_Cal_Limt( &motor.PID_SPEED[num], 10, motor_data.speed_rpm, motor.PID_ANGLE[num].OUT);
		CAN_cmd_chassis((int16_t)(motor.PID_SPEED[num].OUT),0,0,0);
		HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float PID_Cal_Limt(_PID *PID, float limit, float get, float set)//PID死区修改
{
	PID->err = set - get;
	PID->err_err = PID->err - PID->err_old;
	
	PID->P_OUT  = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT  = PID->D * PID->err_err;
	
	PID->I_OUT = (PID->I_OUT > PID->I_LIMIT)?(PID->I_LIMIT):((PID->I_OUT < -PID->I_LIMIT)?(-PID->I_LIMIT):(PID->I_OUT));
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = (PID->OUT > PID->OUT_LIMIT)?(PID->OUT_LIMIT):((PID->OUT < -PID->OUT_LIMIT)?(-PID->OUT_LIMIT):(PID->OUT));
	
	if(ABS(PID->err) <= ABS(limit))
	{
	  PID->I_OUT=0;
	  PID->OUT=0;
	}
	
	PID->err_old = PID->err;
	
	return PID->OUT;
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
