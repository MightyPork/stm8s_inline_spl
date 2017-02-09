/**
  ******************************************************************************
  * @file    stm8s_beep.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototype and macros for the BEEP peripheral.
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_BEEP_H
#define __STM8S_BEEP_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup BEEP_Exported_Types
  * @{
  */

/**
  * @brief  BEEP Frequency selection
  */
typedef enum {
	BEEP_FREQUENCY_1KHZ = (uint8_t) 0x00,  /*!< Beep signal output frequency equals to 1 KHz */
	BEEP_FREQUENCY_2KHZ = (uint8_t) 0x40,  /*!< Beep signal output frequency equals to 2 KHz */
	BEEP_FREQUENCY_4KHZ = (uint8_t) 0x80   /*!< Beep signal output frequency equals to 4 KHz */
} BEEP_Frequency_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup BEEP_Exported_Constants
  * @{
  */

#define BEEP_CALIBRATION_DEFAULT ((uint8_t)0x0B) /*!< Default value when calibration is not done */

#define LSI_FREQUENCY_MIN ((uint32_t)110000) /*!< LSI minimum value in Hertz */
#define LSI_FREQUENCY_MAX ((uint32_t)150000) /*!< LSI maximum value in Hertz */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @addtogroup BEEP_Private_Macros
  * @{
  */

/**
  * @brief  Macro used by the assert function to check the different functions parameters.
  */

/**
  * @brief  Macro used by the assert function to check the BEEP frequencies.
  */
#define IS_BEEP_FREQUENCY_OK(FREQ) \
  (((FREQ) == BEEP_FREQUENCY_1KHZ) || \
   ((FREQ) == BEEP_FREQUENCY_2KHZ) || \
   ((FREQ) == BEEP_FREQUENCY_4KHZ))

/**
  * @brief   Macro used by the assert function to check the LSI frequency (in Hz).
  */
#define IS_LSI_FREQUENCY_OK(FREQ) \
  (((FREQ) >= LSI_FREQUENCY_MIN) && \
   ((FREQ) <= LSI_FREQUENCY_MAX))

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

/** @addtogroup BEEP_Exported_Functions
  * @{
  */

#if 0
void BEEP_DeInit(void);

void BEEP_Init(BEEP_Frequency_TypeDef BEEP_Frequency);

void BEEP_Cmd(FunctionalState NewState);

void BEEP_LSICalibrationConfig(uint32_t LSIFreqHz);
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
  * @addtogroup BEEP_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the BEEP peripheral registers to their default reset
  * values.
  * @param  None
  * @retval None
  */
inline void BEEP_DeInit(void)
{
	BEEP->CSR = BEEP_CSR_RESET_VALUE;
}

/**
  * @brief  Initializes the BEEP function according to the specified parameters.
  * @param   BEEP_Frequency Frequency selection.
  * can be one of the values of @ref BEEP_Frequency_TypeDef.
  * @retval None
  * @par Required preconditions:
  * The LS RC calibration must be performed before calling this function.
  */
inline void BEEP_Init(BEEP_Frequency_TypeDef BEEP_Frequency)
{
	/* Check parameter */
	assert_param(IS_BEEP_FREQUENCY_OK(BEEP_Frequency));

	/* Set a default calibration value if no calibration is done */
	if ((BEEP->CSR & BEEP_CSR_BEEPDIV) == BEEP_CSR_BEEPDIV) {
		BEEP->CSR &= (uint8_t) (~BEEP_CSR_BEEPDIV); /* Clear bits */
		BEEP->CSR |= BEEP_CALIBRATION_DEFAULT;
	}

	/* Select the output frequency */
	BEEP->CSR &= (uint8_t) (~BEEP_CSR_BEEPSEL);
	BEEP->CSR |= (uint8_t) (BEEP_Frequency);
}

/**
  * @brief  Enable or disable the BEEP function.
  * @param   NewState Indicates the new state of the BEEP function.
  * @retval None
  * @par Required preconditions:
  * Initialisation of BEEP and LS RC calibration must be done before.
  */
inline void BEEP_Cmd(FunctionalState NewState)
{
	if (NewState != DISABLE) {
		/* Enable the BEEP peripheral */
		BEEP->CSR |= BEEP_CSR_BEEPEN;
	} else {
		/* Disable the BEEP peripheral */
		BEEP->CSR &= (uint8_t) (~BEEP_CSR_BEEPEN);
	}
}

/**
  * @brief  Update CSR register with the measured LSI frequency.
  * @par Note on the APR calculation:
  * A is the integer part of LSIFreqkHz/4 and x the decimal part.
  * x <= A/(1+2A) is equivalent to A >= x(1+2A) and also to 4A >= 4x(1+2A) [F1]
  * but we know that A + x = LSIFreqkHz/4 ==> 4x = LSIFreqkHz-4A
  * so [F1] can be written :
  * 4A >= (LSIFreqkHz-4A)(1+2A)
  * @param   LSIFreqHz Low Speed RC frequency measured by timer (in Hz).
  * @retval None
  * @par Required preconditions:
  * - BEEP must be disabled to avoid unwanted interrupts.
  */
inline void BEEP_LSICalibrationConfig(uint32_t LSIFreqHz)
{
	uint16_t lsifreqkhz;
	uint16_t A;

	/* Check parameter */
	assert_param(IS_LSI_FREQUENCY_OK(LSIFreqHz));

	lsifreqkhz = (uint16_t) (LSIFreqHz / 1000); /* Converts value in kHz */

	/* Calculation of BEEPER calibration value */

	BEEP->CSR &= (uint8_t) (~BEEP_CSR_BEEPDIV); /* Clear bits */

	A = (uint16_t) (lsifreqkhz >> 3U); /* Division by 8, keep integer part only */

	if ((8U * A) >= ((lsifreqkhz - (8U * A)) * (1U + (2U * A)))) {
		BEEP->CSR |= (uint8_t) (A - 2U);
	} else {
		BEEP->CSR |= (uint8_t) (A - 1U);
	}
}

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* __STM8S_BEEP_H */
