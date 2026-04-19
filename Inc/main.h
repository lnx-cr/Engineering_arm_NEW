/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "struct_typedef.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t Rx_data[18];
typedef struct
{
    struct
    { 
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint16_t ch4;
        uint8_t s1;
        uint8_t s2;
    }rc;
    
    struct 
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }mouse;
   
    struct
    {
        unsigned short v;
    }key;
		union {
    uint16_t key_code;
    struct
    {
      uint16_t W ;
      uint16_t S ;
      uint16_t A ;
      uint16_t D ;
      uint16_t SHIFT ;
      uint16_t CTRL ;
      uint16_t Q ;
      uint16_t E ;
			uint16_t F ;
			uint16_t G ;
			uint16_t R ;
			uint16_t Z ;
			uint16_t X ;
			uint16_t C ;
			uint16_t V ;
	    uint16_t B ;
    } bit;
  } kb;
		struct
		{
      char shift ;
      char ctrl ;
      char q ;
      char e ;
			char f ;
			char g ;
			char r ;
			char z ;
			char x ;
			char c ;
			char v ;
			char b ;
		}flag;
}RC_Ctl_t;
typedef struct
{ 
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int16_t ch4;
		int8_t s1;
		int8_t s2;
}RC;
extern RC_Ctl_t RC_Ctl,RC_Ctl2;
extern RC rc_Ctrl;
extern uint16_t CAN2_data[8]; 
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_HEAT_Pin GPIO_PIN_6
#define IMU_HEAT_GPIO_Port GPIOF
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
