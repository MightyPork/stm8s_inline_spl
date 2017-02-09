/**
  ******************************************************************************
  * @file    stm8s_awu.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototype and macros for the AWU peripheral.
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
#ifndef __STM8S_AWU_H
#define __STM8S_AWU_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup AWU_Exported_Types
  * @{
  */

/**
  * @brief  AWU TimeBase selection
  */

typedef enum {
	AWU_TIMEBASE_NO_IT = (uint8_t) 0,    /*!< No AWU interrupt selected */
	AWU_TIMEBASE_250US = (uint8_t) 1,    /*!< AWU Timebase equals 0.25 ms */
	AWU_TIMEBASE_500US = (uint8_t) 2,    /*!< AWU Timebase equals 0.5 ms */
	AWU_TIMEBASE_1MS = (uint8_t) 3,    /*!< AWU Timebase equals 1 ms */
	AWU_TIMEBASE_2MS = (uint8_t) 4,    /*!< AWU Timebase equals 2 ms */
	AWU_TIMEBASE_4MS = (uint8_t) 5,    /*!< AWU Timebase equals 4 ms */
	AWU_TIMEBASE_8MS = (uint8_t) 6,    /*!< AWU Timebase equals 8 ms */
	AWU_TIMEBASE_16MS = (uint8_t) 7,    /*!< AWU Timebase equals 16 ms */
	AWU_TIMEBASE_32MS = (uint8_t) 8,    /*!< AWU Timebase equals 32 ms */
	AWU_TIMEBASE_64MS = (uint8_t) 9,    /*!< AWU Timebase equals 64 ms */
	AWU_TIMEBASE_128MS = (uint8_t) 10,   /*!< AWU Timebase equals 128 ms */
	AWU_TIMEBASE_256MS = (uint8_t) 11,   /*!< AWU Timebase equals 256 ms */
	AWU_TIMEBASE_512MS = (uint8_t) 12,   /*!< AWU Timebase equals 512 ms */
	AWU_TIMEBASE_1S = (uint8_t) 13,   /*!< AWU Timebase equals 1 s */
	AWU_TIMEBASE_2S = (uint8_t) 14,   /*!< AWU Timebase equals 2 s */
	AWU_TIMEBASE_12S = (uint8_t) 15,   /*!< AWU Timebase equals 12 s */
	AWU_TIMEBASE_30S = (uint8_t) 16    /*!< AWU Timebase equals 30 s */
} AWU_Timebase_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup AWU_Exported_Constants
  * @{
  */

#define LSI_FREQUENCY_MIN ((uint32_t)110000) /*!< LSI minimum value in Hertz */
#define LSI_FREQUENCY_MAX ((uint32_t)150000) /*!< LSI maximum value in Hertz */

/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/** @addtogroup AWU_Private_Macros
  * @{
  */

/**
  * @brief  Macro used by the assert function to check the different functions parameters.
  */

/**
  * @brief   Macro used by the assert function to check the AWU timebases
  */
#define IS_AWU_TIMEBASE_OK(TB) \
  (((TB) == AWU_TIMEBASE_NO_IT) || \
   ((TB) == AWU_TIMEBASE_250US) || \
   ((TB) == AWU_TIMEBASE_500US) || \
   ((TB) == AWU_TIMEBASE_1MS)   || \
   ((TB) == AWU_TIMEBASE_2MS)   || \
   ((TB) == AWU_TIMEBASE_4MS)   || \
   ((TB) == AWU_TIMEBASE_8MS)   || \
   ((TB) == AWU_TIMEBASE_16MS)  || \
   ((TB) == AWU_TIMEBASE_32MS)  || \
   ((TB) == AWU_TIMEBASE_64MS)  || \
   ((TB) == AWU_TIMEBASE_128MS) || \
   ((TB) == AWU_TIMEBASE_256MS) || \
   ((TB) == AWU_TIMEBASE_512MS) || \
   ((TB) == AWU_TIMEBASE_1S)    || \
   ((TB) == AWU_TIMEBASE_2S)    || \
   ((TB) == AWU_TIMEBASE_12S)   || \
   ((TB) == AWU_TIMEBASE_30S))

/**
  * @brief    Macro used by the assert function to check the LSI frequency (in Hz)
  */
#define IS_LSI_FREQUENCY_OK(FREQ) \
  (((FREQ) >= LSI_FREQUENCY_MIN) && \
   ((FREQ) <= LSI_FREQUENCY_MAX))

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

#if 0
/** @addtogroup AWU_Exported_Functions
  * @{
  */
void AWU_DeInit(void);

void AWU_Init(AWU_Timebase_TypeDef AWU_TimeBase);

void AWU_Cmd(FunctionalState NewState);

void AWU_LSICalibrationConfig(uint32_t LSIFreqHz);

void AWU_IdleModeEnable(void);

FlagStatus AWU_GetFlagStatus(void);

#endif
/**
  * @}
  */

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
  * @addtogroup AWU_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the AWU peripheral registers to their default reset
  * values.
  * @param  None
  * @retval None
  */
inline void AWU_DeInit(void)
{
	AWU->CSR = AWU_CSR_RESET_VALUE;
	AWU->APR = AWU_APR_RESET_VALUE;
	AWU->TBR = AWU_TBR_RESET_VALUE;
}

/**
  * @brief  Initializes the AWU peripheral according to the specified parameters.
  * @param   AWU_TimeBase : Time base selection (interval between AWU interrupts).
  * can be one of the values of @ref AWU_Timebase_TypeDef.
  * @retval None
  * @par Required preconditions:
  * The LS RC calibration must be performed before calling this function.
  */
inline void AWU_Init(AWU_Timebase_TypeDef AWU_TimeBase)
{
/* See also AWU_Timebase_TypeDef structure in stm8s_awu.h file :
                          N   2   5   1   2   4   8   1   3   6   1   2   5   1   2   1   3
                          O   5   0   m   m   m   m   6   2   4   2   5   1   s   s   2   0
                          I   0   0   s   s   s   s   m   m   m   8   6   2           s   s
                          T   u   u                   s   s   s   m   m   m
                              s   s                               s   s   s
*/
/** Contains the different values to write in the APR register (used by AWU_Init function) */
	static CONST uint8_t APR_Array[17] =
		{
			0, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 61, 23, 23, 62
		};

/** Contains the different values to write in the TBR register (used by AWU_Init function) */
	static CONST uint8_t TBR_Array[17] =
		{
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 14, 15, 15
		};

	/* Check parameter */
	assert_param(IS_AWU_TIMEBASE_OK(AWU_TimeBase));

	/* Enable the AWU peripheral */
	AWU->CSR |= AWU_CSR_AWUEN;

	/* Set the TimeBase */
	AWU->TBR &= (uint8_t) (~AWU_TBR_AWUTB);
	AWU->TBR |= TBR_Array[(uint8_t) AWU_TimeBase];

	/* Set the APR divider */
	AWU->APR &= (uint8_t) (~AWU_APR_APR);
	AWU->APR |= APR_Array[(uint8_t) AWU_TimeBase];
}

/**
  * @brief  Enable or disable the AWU peripheral.
  * @param   NewState Indicates the new state of the AWU peripheral.
  * @retval None
  * @par Required preconditions:
  * Initialisation of AWU and LS RC calibration must be done before.
  */
inline void AWU_Cmd(FunctionalState NewState)
{
	if (NewState != DISABLE) {
		/* Enable the AWU peripheral */
		AWU->CSR |= AWU_CSR_AWUEN;
	} else {
		/* Disable the AWU peripheral */
		AWU->CSR &= (uint8_t) (~AWU_CSR_AWUEN);
	}
}

/**
  * @brief  Update APR register with the measured LSI frequency.
  * @par Note on the APR calculation:
  * A is the integer part of lsifreqkhz/4 and x the decimal part.
  * x <= A/(1+2A) is equivalent to A >= x(1+2A) and also to 4A >= 4x(1+2A) [F1]
  * but we know that A + x = lsifreqkhz/4 ==> 4x = lsifreqkhz-4A
  * so [F1] can be written :
  * 4A >= (lsifreqkhz-4A)(1+2A)
  * @param   LSIFreqHz Low Speed RC frequency measured by timer (in Hz).
  * @retval None
  * @par Required preconditions:
  * - AWU must be disabled to avoid unwanted interrupts.
  */
inline void AWU_LSICalibrationConfig(uint32_t LSIFreqHz)
{
	uint16_t lsifreqkhz = 0x0;
	uint16_t A = 0x0;

	/* Check parameter */
	assert_param(IS_LSI_FREQUENCY_OK(LSIFreqHz));

	lsifreqkhz = (uint16_t) (LSIFreqHz / 1000); /* Converts value in kHz */

	/* Calculation of AWU calibration value */

	A = (uint16_t) (lsifreqkhz >> 2U); /* Division by 4, keep integer part only */

	if ((4U * A) >= ((lsifreqkhz - (4U * A)) * (1U + (2U * A)))) {
		AWU->APR = (uint8_t) (A - 2U);
	} else {
		AWU->APR = (uint8_t) (A - 1U);
	}
}

/**
  * @brief  Configures AWU in Idle mode to reduce power consumption.
  * @param  None
  * @retval None
  */
inline void AWU_IdleModeEnable(void)
{
	/* Disable AWU peripheral */
	AWU->CSR &= (uint8_t) (~AWU_CSR_AWUEN);

	/* No AWU timebase */
	AWU->TBR = (uint8_t) (~AWU_TBR_AWUTB);
}

/**
  * @brief  Returns status of the AWU peripheral flag.
  * @param  None
  * @retval FlagStatus : Status of the AWU flag.
  * This parameter can be any of the @ref FlagStatus enumeration.
  */
inline FlagStatus AWU_GetFlagStatus(void)
{
	return ((FlagStatus) (((uint8_t) (AWU->CSR & AWU_CSR_AWUF) == (uint8_t) 0x00) ? RESET : SET));
}


/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



#endif /* __STM8S_AWU_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
