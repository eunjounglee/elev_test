/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void ADC1_StartIT(void);               // ★ 인터럽트 모드 시작 API
extern volatile uint16_t adc_value;

static inline uint32_t adc_map_u32(uint16_t v,
                                   uint16_t in_min, uint16_t in_max,
                                   uint32_t out_min, uint32_t out_max,
                                   bool invert)
{
    if (v < in_min) v = in_min;
    if (v > in_max) v = in_max;
    uint32_t num = (uint32_t)(v - in_min) * (out_max - out_min);
    uint32_t den = (uint32_t)(in_max - in_min);
    uint32_t out = out_min + (den ? (num / den) : 0);
    return invert ? (out_max - (out - out_min)) : out;
}
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

