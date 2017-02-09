/**
  ******************************************************************************
  * @file    stm8s_clk.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototype and macros for the CLK peripheral.
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
#ifndef __STM8S_CLK_H
#define __STM8S_CLK_H

/* Includes ------------------------------------------------------------------*/
/* Contains the description of all STM8 hardware registers */
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup CLK_Exported_Types
  * @{
  */

/**
   * @brief  Switch Mode Auto, Manual.
   */
typedef enum {
	CLK_SWITCHMODE_MANUAL = (uint8_t) 0x00, /*!< Enable the manual clock switching mode */
	CLK_SWITCHMODE_AUTO = (uint8_t) 0x01  /*!< Enable the automatic clock switching mode */
} CLK_SwitchMode_TypeDef;

/**
   * @brief  Current Clock State.
   */
typedef enum {
	CLK_CURRENTCLOCKSTATE_DISABLE = (uint8_t) 0x00, /*!< Current clock disable */
	CLK_CURRENTCLOCKSTATE_ENABLE = (uint8_t) 0x01  /*!< Current clock enable */
} CLK_CurrentClockState_TypeDef;

/**
   * @brief   Clock security system configuration.
   */
typedef enum {
	CLK_CSSCONFIG_ENABLEWITHIT = (uint8_t) 0x05, /*!< Enable CSS with detection interrupt */
	CLK_CSSCONFIG_ENABLE = (uint8_t) 0x01, /*!< Enable CSS without detection interrupt */
	CLK_CSSCONFIG_DISABLE = (uint8_t) 0x00  /*!< Leave CSS desactivated (to be used in CLK_Init() function) */
} CLK_CSSConfig_TypeDef;

/**
   * @brief   CLK Clock Source.
   */
typedef enum {
	CLK_SOURCE_HSI = (uint8_t) 0xE1, /*!< Clock Source HSI. */
	CLK_SOURCE_LSI = (uint8_t) 0xD2, /*!< Clock Source LSI. */
	CLK_SOURCE_HSE = (uint8_t) 0xB4 /*!< Clock Source HSE. */
} CLK_Source_TypeDef;

/**
   * @brief   CLK HSI Calibration Value.
   */
typedef enum {
	CLK_HSITRIMVALUE_0 = (uint8_t) 0x00, /*!< HSI Calibration Value 0 */
	CLK_HSITRIMVALUE_1 = (uint8_t) 0x01, /*!< HSI Calibration Value 1 */
	CLK_HSITRIMVALUE_2 = (uint8_t) 0x02, /*!< HSI Calibration Value 2 */
	CLK_HSITRIMVALUE_3 = (uint8_t) 0x03, /*!< HSI Calibration Value 3 */
	CLK_HSITRIMVALUE_4 = (uint8_t) 0x04, /*!< HSI Calibration Value 4 */
	CLK_HSITRIMVALUE_5 = (uint8_t) 0x05, /*!< HSI Calibration Value 5 */
	CLK_HSITRIMVALUE_6 = (uint8_t) 0x06, /*!< HSI Calibration Value 6 */
	CLK_HSITRIMVALUE_7 = (uint8_t) 0x07  /*!< HSI Calibration Value 7 */
} CLK_HSITrimValue_TypeDef;

/**
   * @brief    CLK  Clock Output
   */
typedef enum {
	CLK_OUTPUT_HSI = (uint8_t) 0x00, /*!< Clock Output HSI */
	CLK_OUTPUT_LSI = (uint8_t) 0x02, /*!< Clock Output LSI */
	CLK_OUTPUT_HSE = (uint8_t) 0x04, /*!< Clock Output HSE */
	CLK_OUTPUT_CPU = (uint8_t) 0x08, /*!< Clock Output CPU */
	CLK_OUTPUT_CPUDIV2 = (uint8_t) 0x0A, /*!< Clock Output CPU/2 */
	CLK_OUTPUT_CPUDIV4 = (uint8_t) 0x0C, /*!< Clock Output CPU/4 */
	CLK_OUTPUT_CPUDIV8 = (uint8_t) 0x0E, /*!< Clock Output CPU/8 */
	CLK_OUTPUT_CPUDIV16 = (uint8_t) 0x10, /*!< Clock Output CPU/16 */
	CLK_OUTPUT_CPUDIV32 = (uint8_t) 0x12, /*!< Clock Output CPU/32 */
	CLK_OUTPUT_CPUDIV64 = (uint8_t) 0x14, /*!< Clock Output CPU/64 */
	CLK_OUTPUT_HSIRC = (uint8_t) 0x16, /*!< Clock Output HSI RC */
	CLK_OUTPUT_MASTER = (uint8_t) 0x18, /*!< Clock Output Master */
	CLK_OUTPUT_OTHERS = (uint8_t) 0x1A  /*!< Clock Output OTHER */
} CLK_Output_TypeDef;

/**
   * @brief    CLK Enable peripheral
   */
/* Elements values convention: 0xXY
    X = choice between the peripheral registers
        X = 0 : PCKENR1
        X = 1 : PCKENR2
    Y = Peripheral position in the register
*/
typedef enum {
	CLK_PERIPHERAL_I2C = (uint8_t) 0x00, /*!< Peripheral Clock Enable 1, I2C */
	CLK_PERIPHERAL_SPI = (uint8_t) 0x01, /*!< Peripheral Clock Enable 1, SPI */
#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
	CLK_PERIPHERAL_UART1   = (uint8_t)0x02, /*!< Peripheral Clock Enable 1, UART1 */
#else
	CLK_PERIPHERAL_UART1 = (uint8_t) 0x03, /*!< Peripheral Clock Enable 1, UART1 */
#endif
	CLK_PERIPHERAL_UART2 = (uint8_t) 0x03, /*!< Peripheral Clock Enable 1, UART2 */
	CLK_PERIPHERAL_UART3 = (uint8_t) 0x03, /*!< Peripheral Clock Enable 1, UART3 */
	CLK_PERIPHERAL_TIMER6 = (uint8_t) 0x04, /*!< Peripheral Clock Enable 1, Timer6 */
	CLK_PERIPHERAL_TIMER4 = (uint8_t) 0x04, /*!< Peripheral Clock Enable 1, Timer4 */
	CLK_PERIPHERAL_TIMER5 = (uint8_t) 0x05, /*!< Peripheral Clock Enable 1, Timer5 */
	CLK_PERIPHERAL_TIMER2 = (uint8_t) 0x05, /*!< Peripheral Clock Enable 1, Timer2 */
	CLK_PERIPHERAL_TIMER3 = (uint8_t) 0x06, /*!< Peripheral Clock Enable 1, Timer3 */
	CLK_PERIPHERAL_TIMER1 = (uint8_t) 0x07, /*!< Peripheral Clock Enable 1, Timer1 */
	CLK_PERIPHERAL_AWU = (uint8_t) 0x12, /*!< Peripheral Clock Enable 2, AWU */
	CLK_PERIPHERAL_ADC = (uint8_t) 0x13, /*!< Peripheral Clock Enable 2, ADC */
	CLK_PERIPHERAL_CAN = (uint8_t) 0x17 /*!< Peripheral Clock Enable 2, CAN */
} CLK_Peripheral_TypeDef;

/**
   * @brief   CLK Flags.
   */
/* Elements values convention: 0xXZZ
    X = choice between the flags registers
        X = 1 : ICKR
        X = 2 : ECKR
        X = 3 : SWCR
    X = 4 : CSSR
 X = 5 : CCOR
   ZZ = flag mask in the register (same as map file)
*/
typedef enum {
	CLK_FLAG_LSIRDY = (uint16_t) 0x0110, /*!< Low speed internal oscillator ready Flag */
	CLK_FLAG_HSIRDY = (uint16_t) 0x0102, /*!< High speed internal oscillator ready Flag */
	CLK_FLAG_HSERDY = (uint16_t) 0x0202, /*!< High speed external oscillator ready Flag */
	CLK_FLAG_SWIF = (uint16_t) 0x0308, /*!< Clock switch interrupt Flag */
	CLK_FLAG_SWBSY = (uint16_t) 0x0301, /*!< Switch busy Flag */
	CLK_FLAG_CSSD = (uint16_t) 0x0408, /*!< Clock security system detection Flag */
	CLK_FLAG_AUX = (uint16_t) 0x0402, /*!< Auxiliary oscillator connected to master clock */
	CLK_FLAG_CCOBSY = (uint16_t) 0x0504, /*!< Configurable clock output busy */
	CLK_FLAG_CCORDY = (uint16_t) 0x0502 /*!< Configurable clock output ready */
} CLK_Flag_TypeDef;

/**
   * @brief  CLK interrupt configuration and Flags cleared by software.
   */
typedef enum {
	CLK_IT_CSSD = (uint8_t) 0x0C, /*!< Clock security system detection Flag */
	CLK_IT_SWIF = (uint8_t) 0x1C /*!< Clock switch interrupt Flag */
} CLK_IT_TypeDef;

/**
   * @brief   CLK Clock Divisor.
   */

/* Warning:
   0xxxxxx = HSI divider
   1xxxxxx = CPU divider
   Other bits correspond to the divider's bits mapping
*/
typedef enum {
	CLK_PRESCALER_HSIDIV1 = (uint8_t) 0x00, /*!< High speed internal clock prescaler: 1 */
	CLK_PRESCALER_HSIDIV2 = (uint8_t) 0x08, /*!< High speed internal clock prescaler: 2 */
	CLK_PRESCALER_HSIDIV4 = (uint8_t) 0x10, /*!< High speed internal clock prescaler: 4 */
	CLK_PRESCALER_HSIDIV8 = (uint8_t) 0x18, /*!< High speed internal clock prescaler: 8 */
	CLK_PRESCALER_CPUDIV1 = (uint8_t) 0x80, /*!< CPU clock division factors 1 */
	CLK_PRESCALER_CPUDIV2 = (uint8_t) 0x81, /*!< CPU clock division factors 2 */
	CLK_PRESCALER_CPUDIV4 = (uint8_t) 0x82, /*!< CPU clock division factors 4 */
	CLK_PRESCALER_CPUDIV8 = (uint8_t) 0x83, /*!< CPU clock division factors 8 */
	CLK_PRESCALER_CPUDIV16 = (uint8_t) 0x84, /*!< CPU clock division factors 16 */
	CLK_PRESCALER_CPUDIV32 = (uint8_t) 0x85, /*!< CPU clock division factors 32 */
	CLK_PRESCALER_CPUDIV64 = (uint8_t) 0x86, /*!< CPU clock division factors 64 */
	CLK_PRESCALER_CPUDIV128 = (uint8_t) 0x87  /*!< CPU clock division factors 128 */
} CLK_Prescaler_TypeDef;

/**
   * @brief   SWIM Clock divider.
   */
typedef enum {
	CLK_SWIMDIVIDER_2 = (uint8_t) 0x00, /*!< SWIM clock is divided by 2 */
	CLK_SWIMDIVIDER_OTHER = (uint8_t) 0x01 /*!< SWIM clock is not divided by 2 */
} CLK_SWIMDivider_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup CLK_Exported_Constants
  * @{
  */
#define CLK_TIMEOUT ((uint16_t)0xFFFF) /*!< Max Timeout for the clock switch operation. */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CLK_Private_Macros
  * @{
  */

/**
  * @brief  Macros used by the assert function in order to check the different functions parameters.
  */

/**
  * @brief  Macros used by the assert function in order to check the clock switching modes.
  */
#define IS_CLK_SWITCHMODE_OK(MODE) (((MODE) == CLK_SWITCHMODE_MANUAL) || ((MODE) == CLK_SWITCHMODE_AUTO))

/**
  * @brief  Macros used by the assert function in order to check the current clock state.
  */
#define IS_CLK_CURRENTCLOCKSTATE_OK(STATE) (((STATE) == CLK_CURRENTCLOCKSTATE_DISABLE) ||\
                                            ((STATE) == CLK_CURRENTCLOCKSTATE_ENABLE))

/**
  * @brief  Macros used by the assert function in order to check the CSS configuration.
  */
#define IS_CLK_CSSCONFIG_OK(CSSVALUE) (((CSSVALUE) == CLK_CSSCONFIG_ENABLEWITHIT) ||\
                                       ((CSSVALUE) == CLK_CSSCONFIG_ENABLE) ||\
                                       ((CSSVALUE) == CLK_CSSCONFIG_DISABLE))

/**
  * @brief  Macros used by the assert function in order to check the different clock sources.
  */
#define IS_CLK_SOURCE_OK(SOURCE) (((SOURCE) == CLK_SOURCE_HSI) ||\
                                  ((SOURCE) == CLK_SOURCE_LSI) ||\
                                  ((SOURCE) == CLK_SOURCE_HSE))

/**
  * @brief  Macros used by the assert function in order to check the different HSI trimming values.
  */
#define IS_CLK_HSITRIMVALUE_OK(TRIMVALUE) (((TRIMVALUE) == CLK_HSITRIMVALUE_0) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_1) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_2) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_3) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_4) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_5) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_6) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_7))

/**
  * @brief  Macros used by the assert function in order to check the different clocks to output.
  */
#define IS_CLK_OUTPUT_OK(OUTPUT) (((OUTPUT) == CLK_OUTPUT_HSI) ||\
                                  ((OUTPUT) == CLK_OUTPUT_HSE) ||\
                                  ((OUTPUT) == CLK_OUTPUT_LSI) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPU) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV2) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV4) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV8) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV16) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV32) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV64) ||\
                                  ((OUTPUT) == CLK_OUTPUT_HSIRC) ||\
                                  ((OUTPUT) == CLK_OUTPUT_MASTER) ||\
                                  ((OUTPUT) == CLK_OUTPUT_OTHERS))

/**
  * @brief  Macros used by the assert function in order to check the different peripheral's clock.
  */
#define IS_CLK_PERIPHERAL_OK(PERIPHERAL) (((PERIPHERAL) == CLK_PERIPHERAL_I2C) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_SPI) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART3) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART2) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART1) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER4) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER2) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER5) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER6) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER3) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER1) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_CAN) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_ADC) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_AWU))

/**
  * @brief  Macros used by the assert function in order to check the different clock flags.
  */
#define IS_CLK_FLAG_OK(FLAG) (((FLAG) == CLK_FLAG_LSIRDY) ||\
                              ((FLAG) == CLK_FLAG_HSIRDY) ||\
                              ((FLAG) == CLK_FLAG_HSERDY) ||\
                              ((FLAG) == CLK_FLAG_SWIF) ||\
                              ((FLAG) == CLK_FLAG_SWBSY) ||\
                              ((FLAG) == CLK_FLAG_CSSD) ||\
                              ((FLAG) == CLK_FLAG_AUX) ||\
                              ((FLAG) == CLK_FLAG_CCOBSY) ||\
                              ((FLAG) == CLK_FLAG_CCORDY))

/**
  * @brief  Macros used by the assert function in order to check the different clock IT pending bits.
  */
#define IS_CLK_IT_OK(IT) (((IT) == CLK_IT_CSSD) || ((IT) == CLK_IT_SWIF))

/**
  * @brief  Macros used by the assert function in order to check the different HSI prescaler values.
  */
#define IS_CLK_HSIPRESCALER_OK(PRESCALER) (((PRESCALER) == CLK_PRESCALER_HSIDIV1) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV2) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV4) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV8))

/**
  * @brief  Macros used by the assert function in order to check the different clock  prescaler values.
  */
#define IS_CLK_PRESCALER_OK(PRESCALER) (((PRESCALER) == CLK_PRESCALER_HSIDIV1) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV2) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV4) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV8) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV1) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV2) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV4) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV8) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV16) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV32) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV64) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV128))

/**
  * @brief  Macros used by the assert function in order to check the different SWIM dividers values.
  */
#define IS_CLK_SWIMDIVIDER_OK(SWIMDIVIDER) (((SWIMDIVIDER) == CLK_SWIMDIVIDER_2) || ((SWIMDIVIDER) == CLK_SWIMDIVIDER_OTHER))

#if 0
/**
  * @}
  */

/** @addtogroup CLK_Exported_functions
  * @{
  */
void CLK_DeInit(void);

void CLK_HSECmd(FunctionalState NewState);

void CLK_HSICmd(FunctionalState NewState);

void CLK_LSICmd(FunctionalState NewState);

void CLK_CCOCmd(FunctionalState NewState);

void CLK_ClockSwitchCmd(FunctionalState NewState);

void CLK_FastHaltWakeUpCmd(FunctionalState NewState);

void CLK_SlowActiveHaltWakeUpCmd(FunctionalState NewState);

void CLK_PeripheralClockConfig(CLK_Peripheral_TypeDef CLK_Peripheral, FunctionalState NewState);

ErrorStatus
CLK_ClockSwitchConfig(CLK_SwitchMode_TypeDef CLK_SwitchMode, CLK_Source_TypeDef CLK_NewClock, FunctionalState ITState,
					  CLK_CurrentClockState_TypeDef CLK_CurrentClockState);

void CLK_HSIPrescalerConfig(CLK_Prescaler_TypeDef HSIPrescaler);

void CLK_CCOConfig(CLK_Output_TypeDef CLK_CCO);

void CLK_ITConfig(CLK_IT_TypeDef CLK_IT, FunctionalState NewState);

void CLK_SYSCLKConfig(CLK_Prescaler_TypeDef CLK_Prescaler);

void CLK_SWIMConfig(CLK_SWIMDivider_TypeDef CLK_SWIMDivider);

void CLK_ClockSecuritySystemEnable(void);

void CLK_SYSCLKEmergencyClear(void);

void CLK_AdjustHSICalibrationValue(CLK_HSITrimValue_TypeDef CLK_HSICalibrationValue);

uint32_t CLK_GetClockFreq(void);

CLK_Source_TypeDef CLK_GetSYSCLKSource(void);

FlagStatus CLK_GetFlagStatus(CLK_Flag_TypeDef CLK_FLAG);

ITStatus CLK_GetITStatus(CLK_IT_TypeDef CLK_IT);

void CLK_ClearITPendingBit(CLK_IT_TypeDef CLK_IT);
#endif

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/

/**
  * @addtogroup CLK_Private_Constants
  * @{
  */


//CONST uint8_t CLKPrescTable[8] = {1, 2, 4, 8, 10, 16, 20, 40}; /*!< Holds the different CLK prescaler values */

/**
  * @}
  */

/* Public functions ----------------------------------------------------------*/
/**
  * @addtogroup CLK_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the CLK peripheral registers to their default reset
  * values.
  * @param  None
  * @retval None
  * @par Warning:
  * Resetting the CCOR register: \n
  * When the CCOEN bit is set, the reset of the CCOR register require
  * two consecutive write instructions in order to reset first the CCOEN bit
  * and the second one is to reset the CCOSEL bits.
  */
inline void CLK_DeInit(void)
{
	CLK->ICKR = CLK_ICKR_RESET_VALUE;
	CLK->ECKR = CLK_ECKR_RESET_VALUE;
	CLK->SWR = CLK_SWR_RESET_VALUE;
	CLK->SWCR = CLK_SWCR_RESET_VALUE;
	CLK->CKDIVR = CLK_CKDIVR_RESET_VALUE;
	CLK->PCKENR1 = CLK_PCKENR1_RESET_VALUE;
	CLK->PCKENR2 = CLK_PCKENR2_RESET_VALUE;
	CLK->CSSR = CLK_CSSR_RESET_VALUE;
	CLK->CCOR = CLK_CCOR_RESET_VALUE;
	while ((CLK->CCOR & CLK_CCOR_CCOEN) != 0) {}
	CLK->CCOR = CLK_CCOR_RESET_VALUE;
	CLK->HSITRIMR = CLK_HSITRIMR_RESET_VALUE;
	CLK->SWIMCCR = CLK_SWIMCCR_RESET_VALUE;
}

/**
  * @brief   Configures the High Speed Internal oscillator (HSI).
  * @par Full description:
  * If CLK_FastHaltWakeup is enabled, HSI oscillator is automatically
  * switched-on (HSIEN=1) and selected as next clock master
  * (CKM=SWI=HSI) when resuming from HALT/ActiveHalt modes.\n
  * @param   NewState this parameter is the Wake-up Mode state.
  * @retval None
  */
inline void CLK_FastHaltWakeUpCmd(FunctionalState NewState)
{
	/* check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set FHWU bit (HSI oscillator is automatically switched-on) */
		CLK->ICKR |= CLK_ICKR_FHWU;
	} else  /* FastHaltWakeup = DISABLE */
	{
		/* Reset FHWU bit */
		CLK->ICKR &= (uint8_t) (~CLK_ICKR_FHWU);
	}
}

/**
  * @brief  Enable or Disable the External High Speed oscillator (HSE).
  * @param   NewState new state of HSEEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
inline void CLK_HSECmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set HSEEN bit */
		CLK->ECKR |= CLK_ECKR_HSEEN;
	} else {
		/* Reset HSEEN bit */
		CLK->ECKR &= (uint8_t) (~CLK_ECKR_HSEEN);
	}
}

/**
  * @brief  Enables or disables the Internal High Speed oscillator (HSI).
  * @param   NewState new state of HSIEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
inline void CLK_HSICmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set HSIEN bit */
		CLK->ICKR |= CLK_ICKR_HSIEN;
	} else {
		/* Reset HSIEN bit */
		CLK->ICKR &= (uint8_t) (~CLK_ICKR_HSIEN);
	}
}

/**
  * @brief  Enables or disables the Internal Low Speed oscillator (LSI).
  * @param  NewState new state of LSIEN, value accepted ENABLE, DISABLE.
  * @note   Before using the LSI clock you have to enable the option bytes (LSI_EN option bit is set).
  * @retval None
  */
inline void CLK_LSICmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set LSIEN bit */
		CLK->ICKR |= CLK_ICKR_LSIEN;
	} else {
		/* Reset LSIEN bit */
		CLK->ICKR &= (uint8_t) (~CLK_ICKR_LSIEN);
	}
}

/**
  * @brief  Enables or disablle the Configurable Clock Output (CCO).
  * @param   NewState : New state of CCEN bit (CCO register).
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
inline void CLK_CCOCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set CCOEN bit */
		CLK->CCOR |= CLK_CCOR_CCOEN;
	} else {
		/* Reset CCOEN bit */
		CLK->CCOR &= (uint8_t) (~CLK_CCOR_CCOEN);
	}
}

/**
  * @brief  Starts or Stops manually the clock switch execution.
  * @par Full description:
  * NewState parameter set the SWEN.
  * @param   NewState new state of SWEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
inline void CLK_ClockSwitchCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the Clock Switch */
		CLK->SWCR |= CLK_SWCR_SWEN;
	} else {
		/* Disable the Clock Switch */
		CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWEN);
	}
}

/**
  * @brief  Configures the slow active halt wake up
  * @param   NewState: specifies the Slow Active Halt wake up state.
  * can be set of the following values:
  * - DISABLE: Slow Active Halt mode disabled;
  * - ENABLE:  Slow Active Halt mode enabled.
  * @retval None
  */
inline void CLK_SlowActiveHaltWakeUpCmd(FunctionalState NewState)
{
	/* check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Set S_ACTHALT bit */
		CLK->ICKR |= CLK_ICKR_SWUAH;
	} else {
		/* Reset S_ACTHALT bit */
		CLK->ICKR &= (uint8_t) (~CLK_ICKR_SWUAH);
	}
}

/**
  * @brief   Enables or disables the specified peripheral CLK.
  * @param   CLK_Peripheral : This parameter specifies the peripheral clock to gate.
  * This parameter can be any of the  @ref CLK_Peripheral_TypeDef enumeration.
  * @param   NewState : New state of specified peripheral clock.
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
inline void CLK_PeripheralClockConfig(CLK_Peripheral_TypeDef CLK_Peripheral, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));
	assert_param(IS_CLK_PERIPHERAL_OK(CLK_Peripheral));

	if (((uint8_t) CLK_Peripheral & (uint8_t) 0x10) == 0x00) {
		if (NewState != DISABLE) {
			/* Enable the peripheral Clock */
			CLK->PCKENR1 |= (uint8_t) ((uint8_t) 1 << ((uint8_t) CLK_Peripheral & (uint8_t) 0x0F));
		} else {
			/* Disable the peripheral Clock */
			CLK->PCKENR1 &= (uint8_t) (~(uint8_t) (((uint8_t) 1 << ((uint8_t) CLK_Peripheral & (uint8_t) 0x0F))));
		}
	} else {
		if (NewState != DISABLE) {
			/* Enable the peripheral Clock */
			CLK->PCKENR2 |= (uint8_t) ((uint8_t) 1 << ((uint8_t) CLK_Peripheral & (uint8_t) 0x0F));
		} else {
			/* Disable the peripheral Clock */
			CLK->PCKENR2 &= (uint8_t) (~(uint8_t) (((uint8_t) 1 << ((uint8_t) CLK_Peripheral & (uint8_t) 0x0F))));
		}
	}
}

/**
  * @brief  configures the Switch from one clock to another
  * @param   CLK_SwitchMode select the clock switch mode.
  * It can be set of the values of @ref CLK_SwitchMode_TypeDef
  * @param   CLK_NewClock choice of the future clock.
  * It can be set of the values of @ref CLK_Source_TypeDef
  * @param   NewState Enable or Disable the Clock Switch interrupt.
  * @param   CLK_CurrentClockState current clock to switch OFF or to keep ON.
  * It can be set of the values of @ref CLK_CurrentClockState_TypeDef
  * @note LSI selected as master clock source only if LSI_EN option bit is set.
  * @retval ErrorStatus this shows the clock switch status (ERROR/SUCCESS).
  */
inline ErrorStatus
CLK_ClockSwitchConfig(CLK_SwitchMode_TypeDef CLK_SwitchMode, CLK_Source_TypeDef CLK_NewClock, FunctionalState ITState,
					  CLK_CurrentClockState_TypeDef CLK_CurrentClockState)
{
	CLK_Source_TypeDef clock_master;
	uint16_t DownCounter = CLK_TIMEOUT;
	ErrorStatus Swif = ERROR;

	/* Check the parameters */
	assert_param(IS_CLK_SOURCE_OK(CLK_NewClock));
	assert_param(IS_CLK_SWITCHMODE_OK(CLK_SwitchMode));
	assert_param(IS_FUNCTIONALSTATE_OK(ITState));
	assert_param(IS_CLK_CURRENTCLOCKSTATE_OK(CLK_CurrentClockState));

	/* Current clock master saving */
	clock_master = (CLK_Source_TypeDef) CLK->CMSR;

	/* Automatic switch mode management */
	if (CLK_SwitchMode == CLK_SWITCHMODE_AUTO) {
		/* Enables Clock switch */
		CLK->SWCR |= CLK_SWCR_SWEN;

		/* Enables or Disables Switch interrupt */
		if (ITState != DISABLE) {
			CLK->SWCR |= CLK_SWCR_SWIEN;
		} else {
			CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWIEN);
		}

		/* Selection of the target clock source */
		CLK->SWR = (uint8_t) CLK_NewClock;

		/* Wait until the target clock source is ready */
		while ((((CLK->SWCR & CLK_SWCR_SWBSY) != 0) && (DownCounter != 0))) {
			DownCounter--;
		}

		if (DownCounter != 0) {
			Swif = SUCCESS;
		} else {
			Swif = ERROR;
		}
	} else /* CLK_SwitchMode == CLK_SWITCHMODE_MANUAL */
	{
		/* Enables or Disables Switch interrupt  if required  */
		if (ITState != DISABLE) {
			CLK->SWCR |= CLK_SWCR_SWIEN;
		} else {
			CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWIEN);
		}

		/* Selection of the target clock source */
		CLK->SWR = (uint8_t) CLK_NewClock;

		/* Wait until the target clock source is ready */
		while ((((CLK->SWCR & CLK_SWCR_SWIF) != 0) && (DownCounter != 0))) {
			DownCounter--;
		}

		if (DownCounter != 0) {
			/* Enables Clock switch */
			CLK->SWCR |= CLK_SWCR_SWEN;
			Swif = SUCCESS;
		} else {
			Swif = ERROR;
		}
	}
	if (Swif != ERROR) {
		/* Switch OFF current clock if required */
		if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && (clock_master == CLK_SOURCE_HSI)) {
			CLK->ICKR &= (uint8_t) (~CLK_ICKR_HSIEN);
		} else if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && (clock_master == CLK_SOURCE_LSI)) {
			CLK->ICKR &= (uint8_t) (~CLK_ICKR_LSIEN);
		} else if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && (clock_master == CLK_SOURCE_HSE)) {
			CLK->ECKR &= (uint8_t) (~CLK_ECKR_HSEEN);
		}
	}
	return (Swif);
}

/**
  * @brief  Configures the HSI clock dividers.
  * @param   HSIPrescaler : Specifies the HSI clock divider to apply.
  * This parameter can be any of the @ref CLK_Prescaler_TypeDef enumeration.
  * @retval None
  */
inline void CLK_HSIPrescalerConfig(CLK_Prescaler_TypeDef HSIPrescaler)
{
	/* check the parameters */
	assert_param(IS_CLK_HSIPRESCALER_OK(HSIPrescaler));

	/* Clear High speed internal clock prescaler */
	CLK->CKDIVR &= (uint8_t) (~CLK_CKDIVR_HSIDIV);

	/* Set High speed internal clock prescaler */
	CLK->CKDIVR |= (uint8_t) HSIPrescaler;
}

/**
  * @brief  Output the selected clock on a dedicated I/O pin.
  * @param   CLK_CCO : Specifies the clock source.
  * This parameter can be any of the  @ref CLK_Output_TypeDef enumeration.
  * @retval None
  * @par Required preconditions:
  * The dedicated I/O pin must be set at 1 in the corresponding Px_CR1 register \n
  * to be set as input with pull-up or push-pull output.
  */
inline void CLK_CCOConfig(CLK_Output_TypeDef CLK_CCO)
{
	/* check the parameters */
	assert_param(IS_CLK_OUTPUT_OK(CLK_CCO));

	/* Clears of the CCO type bits part */
	CLK->CCOR &= (uint8_t) (~CLK_CCOR_CCOSEL);

	/* Selects the source provided on cco_ck output */
	CLK->CCOR |= (uint8_t) CLK_CCO;

	/* Enable the clock output */
	CLK->CCOR |= CLK_CCOR_CCOEN;
}

/**
  * @brief   Enables or disables the specified CLK interrupts.
  * @param   CLK_IT This parameter specifies the interrupt sources.
  * It can be one of the values of @ref CLK_IT_TypeDef.
  * @param   NewState New state of the Interrupt.
  * Value accepted ENABLE, DISABLE.
  * @retval None
  */
inline void CLK_ITConfig(CLK_IT_TypeDef CLK_IT, FunctionalState NewState)
{
	/* check the parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));
	assert_param(IS_CLK_IT_OK(CLK_IT));

	if (NewState != DISABLE) {
		switch (CLK_IT) {
			case CLK_IT_SWIF: /* Enable the clock switch interrupt */
				CLK->SWCR |= CLK_SWCR_SWIEN;
				break;
			case CLK_IT_CSSD: /* Enable the clock security system detection interrupt */
				CLK->CSSR |= CLK_CSSR_CSSDIE;
				break;
			default:
				break;
		}
	} else  /*(NewState == DISABLE)*/
	{
		switch (CLK_IT) {
			case CLK_IT_SWIF: /* Disable the clock switch interrupt */
				CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWIEN);
				break;
			case CLK_IT_CSSD: /* Disable the clock security system detection interrupt */
				CLK->CSSR &= (uint8_t) (~CLK_CSSR_CSSDIE);
				break;
			default:
				break;
		}
	}
}

/**
  * @brief  Configures the HSI and CPU clock dividers.
  * @param   ClockPrescaler Specifies the HSI or CPU clock divider to apply.
  * @retval None
  */
inline void CLK_SYSCLKConfig(CLK_Prescaler_TypeDef CLK_Prescaler)
{
	/* check the parameters */
	assert_param(IS_CLK_PRESCALER_OK(CLK_Prescaler));

	if (((uint8_t) CLK_Prescaler & (uint8_t) 0x80) == 0x00) /* Bit7 = 0 means HSI divider */
	{
		CLK->CKDIVR &= (uint8_t) (~CLK_CKDIVR_HSIDIV);
		CLK->CKDIVR |= (uint8_t) ((uint8_t) CLK_Prescaler & (uint8_t) CLK_CKDIVR_HSIDIV);
	} else /* Bit7 = 1 means CPU divider */
	{
		CLK->CKDIVR &= (uint8_t) (~CLK_CKDIVR_CPUDIV);
		CLK->CKDIVR |= (uint8_t) ((uint8_t) CLK_Prescaler & (uint8_t) CLK_CKDIVR_CPUDIV);
	}
}

/**
  * @brief  Configures the SWIM clock frequency on the fly.
  * @param   CLK_SWIMDivider Specifies the SWIM clock divider to apply.
  * can be one of the value of @ref CLK_SWIMDivider_TypeDef
  * @retval None
  */
inline void CLK_SWIMConfig(CLK_SWIMDivider_TypeDef CLK_SWIMDivider)
{
	/* check the parameters */
	assert_param(IS_CLK_SWIMDIVIDER_OK(CLK_SWIMDivider));

	if (CLK_SWIMDivider != CLK_SWIMDIVIDER_2) {
		/* SWIM clock is not divided by 2 */
		CLK->SWIMCCR |= CLK_SWIMCCR_SWIMDIV;
	} else /* CLK_SWIMDivider == CLK_SWIMDIVIDER_2 */
	{
		/* SWIM clock is divided by 2 */
		CLK->SWIMCCR &= (uint8_t) (~CLK_SWIMCCR_SWIMDIV);
	}
}

/**
  * @brief  Enables the Clock Security System.
  * @par Full description:
  * once CSS is enabled it cannot be disabled until the next reset.
  * @param  None
  * @retval None
  */
inline void CLK_ClockSecuritySystemEnable(void)
{
	/* Set CSSEN bit */
	CLK->CSSR |= CLK_CSSR_CSSEN;
}

/**
  * @brief  Returns the clock source used as system clock.
  * @param  None
  * @retval  Clock source used.
  * can be one of the values of @ref CLK_Source_TypeDef
  */
inline CLK_Source_TypeDef CLK_GetSYSCLKSource(void)
{
	return ((CLK_Source_TypeDef) CLK->CMSR);
}

/**
  * @brief  This function returns the frequencies of different on chip clocks.
  * @param  None
  * @retval the master clock frequency
  */
inline uint32_t CLK_GetClockFreq(void)
{
//	static const CONST uint8_t HSIDivFactor[4] = {1, 2, 4, 8}; /*!< Holds the different HSI Divider factors */
	uint32_t clockfrequency = 0;
	CLK_Source_TypeDef clocksource = CLK_SOURCE_HSI;
	uint8_t tmp = 0, presc = 0;

	/* Get CLK source. */
	clocksource = (CLK_Source_TypeDef) CLK->CMSR;

	if (clocksource == CLK_SOURCE_HSI) {
		tmp = (uint8_t) (CLK->CKDIVR & CLK_CKDIVR_HSIDIV);
		tmp = (uint8_t) (tmp >> 3);
		presc = (uint8_t)(1<<tmp);// HSIDivFactor[tmp]; // CHANGED
		clockfrequency = HSI_VALUE / presc;
	} else if (clocksource == CLK_SOURCE_LSI) {
		clockfrequency = LSI_VALUE;
	} else {
		clockfrequency = HSE_VALUE;
	}

	return ((uint32_t) clockfrequency);
}

/**
  * @brief  Adjusts the Internal High Speed oscillator (HSI) calibration value.
  * @par Full description:
  * @param   CLK_HSICalibrationValue calibration trimming value.
  * can be one of the values of @ref CLK_HSITrimValue_TypeDef
  * @retval None
  */
inline void CLK_AdjustHSICalibrationValue(CLK_HSITrimValue_TypeDef CLK_HSICalibrationValue)
{
	/* check the parameters */
	assert_param(IS_CLK_HSITRIMVALUE_OK(CLK_HSICalibrationValue));

	/* Store the new value */
	CLK->HSITRIMR = (uint8_t) ((uint8_t) (CLK->HSITRIMR & (uint8_t) (~CLK_HSITRIMR_HSITRIM)) |
							   ((uint8_t) CLK_HSICalibrationValue));
}

/**
  * @brief  Reset the SWBSY flag (SWICR Register)
  * @par Full description:
  * This function reset SWBSY flag in order to reset clock switch operations (target
  * oscillator is broken, stabilization is longing too much, etc.).  If at the same time \n
  * software attempts to set SWEN and clear SWBSY, SWBSY action takes precedence.
  * @param  None
  * @retval None
  */
inline void CLK_SYSCLKEmergencyClear(void)
{
	CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWBSY);
}

/**
  * @brief  Checks whether the specified CLK flag is set or not.
  * @par Full description:
  * @param   CLK_FLAG Flag to check.
  * can be one of the values of @ref CLK_Flag_TypeDef
  * @retval FlagStatus, status of the checked flag
  */
inline FlagStatus CLK_GetFlagStatus(CLK_Flag_TypeDef CLK_FLAG)
{
	uint16_t statusreg = 0;
	uint8_t tmpreg = 0;
	FlagStatus bitstatus = RESET;

	/* check the parameters */
	assert_param(IS_CLK_FLAG_OK(CLK_FLAG));

	/* Get the CLK register index */
	statusreg = (uint16_t) ((uint16_t) CLK_FLAG & (uint16_t) 0xFF00);


	if (statusreg == 0x0100) /* The flag to check is in ICKRregister */
	{
		tmpreg = CLK->ICKR;
	} else if (statusreg == 0x0200) /* The flag to check is in ECKRregister */
	{
		tmpreg = CLK->ECKR;
	} else if (statusreg == 0x0300) /* The flag to check is in SWIC register */
	{
		tmpreg = CLK->SWCR;
	} else if (statusreg == 0x0400) /* The flag to check is in CSS register */
	{
		tmpreg = CLK->CSSR;
	} else /* The flag to check is in CCO register */
	{
		tmpreg = CLK->CCOR;
	}

	if ((tmpreg & (uint8_t) CLK_FLAG) != (uint8_t) RESET) {
		bitstatus = SET;
	} else {
		bitstatus = RESET;
	}

	/* Return the flag status */
	return ((FlagStatus) bitstatus);
}

/**
  * @brief  Checks whether the specified CLK interrupt has is enabled or not.
  * @param   CLK_IT specifies the CLK interrupt.
  * can be one of the values of @ref CLK_IT_TypeDef
  * @retval ITStatus, new state of CLK_IT (SET or RESET).
  */
inline ITStatus CLK_GetITStatus(CLK_IT_TypeDef CLK_IT)
{
	ITStatus bitstatus = RESET;

	/* check the parameters */
	assert_param(IS_CLK_IT_OK(CLK_IT));

	if (CLK_IT == CLK_IT_SWIF) {
		/* Check the status of the clock switch interrupt */
		if ((CLK->SWCR & (uint8_t) CLK_IT) == (uint8_t) 0x0C) {
			bitstatus = SET;
		} else {
			bitstatus = RESET;
		}
	} else /* CLK_IT == CLK_IT_CSSDIE */
	{
		/* Check the status of the security system detection interrupt */
		if ((CLK->CSSR & (uint8_t) CLK_IT) == (uint8_t) 0x0C) {
			bitstatus = SET;
		} else {
			bitstatus = RESET;
		}
	}

	/* Return the CLK_IT status */
	return bitstatus;
}

/**
  * @brief  Clears the CLK's interrupt pending bits.
  * @param   CLK_IT specifies the interrupt pending bits.
  * can be one of the values of @ref CLK_IT_TypeDef
  * @retval None
  */
inline void CLK_ClearITPendingBit(CLK_IT_TypeDef CLK_IT)
{
	/* check the parameters */
	assert_param(IS_CLK_IT_OK(CLK_IT));

	if (CLK_IT == (uint8_t) CLK_IT_CSSD) {
		/* Clear the status of the security system detection interrupt */
		CLK->CSSR &= (uint8_t) (~CLK_CSSR_CSSD);
	} else /* CLK_PendingBit == (uint8_t)CLK_IT_SWIF */
	{
		/* Clear the status of the clock switch interrupt */
		CLK->SWCR &= (uint8_t) (~CLK_SWCR_SWIF);
	}

}

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



/**
  * @}
  */
#endif /* __STM8S_CLK_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
