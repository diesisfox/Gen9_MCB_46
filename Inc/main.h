/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LEFT_SIG_SWITCH_Pin GPIO_PIN_0
#define LEFT_SIG_SWITCH_GPIO_Port GPIOC
#define SCREEN_LEFT_Pin GPIO_PIN_2
#define SCREEN_LEFT_GPIO_Port GPIOC
#define SCREEN_RIGHT_Pin GPIO_PIN_3
#define SCREEN_RIGHT_GPIO_Port GPIOC
#define BRK_ADC_Pin GPIO_PIN_1
#define BRK_ADC_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SCREEN_CS_Pin GPIO_PIN_4
#define SCREEN_CS_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define COIL_STA_DOWN_Pin GPIO_PIN_6
#define COIL_STA_DOWN_GPIO_Port GPIOA
#define REGEN_ON_Pin GPIO_PIN_7
#define REGEN_ON_GPIO_Port GPIOA
#define ENGAGE_LIGHT_Pin GPIO_PIN_4
#define ENGAGE_LIGHT_GPIO_Port GPIOC
#define HV_MAIN__MC_ON_Pin GPIO_PIN_5
#define HV_MAIN__MC_ON_GPIO_Port GPIOC
#define ACC_ADC_Pin GPIO_PIN_0
#define ACC_ADC_GPIO_Port GPIOB
#define ARRAY_PRE_SIG_Pin GPIO_PIN_1
#define ARRAY_PRE_SIG_GPIO_Port GPIOB
#define ARRAY_ON_SIG_Pin GPIO_PIN_2
#define ARRAY_ON_SIG_GPIO_Port GPIOB
#define PWM_LOCK_Pin GPIO_PIN_10
#define PWM_LOCK_GPIO_Port GPIOB
#define MOTOR_PRE_SIG_Pin GPIO_PIN_12
#define MOTOR_PRE_SIG_GPIO_Port GPIOB
#define ENGAGE_SIG_Pin GPIO_PIN_13
#define ENGAGE_SIG_GPIO_Port GPIOB
#define RIGHT_SIG_SWITCH_Pin GPIO_PIN_14
#define RIGHT_SIG_SWITCH_GPIO_Port GPIOB
#define MOTOR_ON_SIG_Pin GPIO_PIN_15
#define MOTOR_ON_SIG_GPIO_Port GPIOB
#define HV_BOTH__MC_ON_Pin GPIO_PIN_6
#define HV_BOTH__MC_ON_GPIO_Port GPIOC
#define GEAR_N_Pin GPIO_PIN_7
#define GEAR_N_GPIO_Port GPIOC
#define HV_BOTH__MC_OFF_Pin GPIO_PIN_8
#define HV_BOTH__MC_OFF_GPIO_Port GPIOC
#define MAG_TIM_UP_Pin GPIO_PIN_9
#define MAG_TIM_UP_GPIO_Port GPIOC
#define GEAR_F_Pin GPIO_PIN_8
#define GEAR_F_GPIO_Port GPIOA
#define GEAR_R_Pin GPIO_PIN_9
#define GEAR_R_GPIO_Port GPIOA
#define PWM_DOWN_Pin GPIO_PIN_10
#define PWM_DOWN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RIGHT_LIGHT_Pin GPIO_PIN_12
#define RIGHT_LIGHT_GPIO_Port GPIOC
#define LEFT_LIGHT_Pin GPIO_PIN_2
#define LEFT_LIGHT_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PWM_UP_Pin GPIO_PIN_4
#define PWM_UP_GPIO_Port GPIOB
#define BRK_LIGHT_Pin GPIO_PIN_7
#define BRK_LIGHT_GPIO_Port GPIOB
#define MAG_TIM_DOWN_Pin GPIO_PIN_8
#define MAG_TIM_DOWN_GPIO_Port GPIOB
#define COIL_STA_UP_Pin GPIO_PIN_9
#define COIL_STA_UP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
