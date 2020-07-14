/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
static double VDD;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t flash_read(uint32_t address);
void flash_write(uint32_t address,uint32_t data);
void flash_write_d(uint32_t address,double *data);
double flash_read_d(uint32_t address);

void flash_lock(void);
void flash_unlock(void);
void flash_erase_page(uint32_t address);
uint8_t flash_ready(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define J4_Pin GPIO_PIN_12
#define J4_GPIO_Port GPIOB
#define J5_Pin GPIO_PIN_13
#define J5_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_9
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOA
#define DE_Pin GPIO_PIN_12
#define DE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
// адреса для сохранения во flash
#define StartSettingsAddres 0x0803fc00
#define K_Addres StartSettingsAddres
#define pointsROtate_Addres K_Addres + 8
#define startAngle_Addres pointsROtate_Addres + 4
#define finishAngle_Addres startAngle_Addres + 4
#define min_U_Addres finishAngle_Addres + 4
#define max_U_Addres min_U_Addres + 8
#define boarate_Addres max_U_Addres + 8
#define slaveAdr_Addres boarate_Addres + 4
#define min_PosPWM_Addres slaveAdr_Addres + 4
#define max_PosPWM_Addres min_PosPWM_Addres + 4
#define startAngle_Cal_Addres max_PosPWM_Addres + 4
#define finishAngle_Cal_Addres startAngle_Cal_Addres + 4


//настройки для калибровки
#define PAUSE_CAL 100
#define STEP 10
#define HYSTERESIS 10

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
