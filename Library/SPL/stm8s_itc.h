/**
  ******************************************************************************
  * @file    stm8s_itc.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototype and macros for the ITC peripheral.
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
#ifndef __STM8S_ITC_H
#define __STM8S_ITC_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup ITC_Exported_Types
  * @{
  */

/**
  * @brief  ITC Interrupt Lines selection
  */
typedef enum {
	ITC_IRQ_TLI = (uint8_t) 0,   /*!< Software interrupt */
	ITC_IRQ_AWU = (uint8_t) 1,   /*!< Auto wake up from halt interrupt */
	ITC_IRQ_CLK = (uint8_t) 2,   /*!< Clock controller interrupt */
	ITC_IRQ_PORTA = (uint8_t) 3,   /*!< Port A external interrupts */
	ITC_IRQ_PORTB = (uint8_t) 4,   /*!< Port B external interrupts */
	ITC_IRQ_PORTC = (uint8_t) 5,   /*!< Port C external interrupts */
	ITC_IRQ_PORTD = (uint8_t) 6,   /*!< Port D external interrupts */
	ITC_IRQ_PORTE = (uint8_t) 7,   /*!< Port E external interrupts */

#if defined(STM8S208) || defined(STM8AF52Ax)
	ITC_IRQ_CAN_RX         = (uint8_t)8,   /*!< beCAN RX interrupt */
	ITC_IRQ_CAN_TX         = (uint8_t)9,   /*!< beCAN TX/ER/SC interrupt */
#endif /*STM8S208 or STM8AF52Ax */

#if defined(STM8S903) || defined(STM8AF622x)
	ITC_IRQ_PORTF          = (uint8_t)8,   /*!< Port F external interrupts */
#endif /*STM8S903 or STM8AF622x */

	ITC_IRQ_SPI = (uint8_t) 10,  /*!< SPI interrupt */
	ITC_IRQ_TIM1_OVF = (uint8_t) 11,  /*!< TIM1 update/overflow/underflow/trigger/
                                         break interrupt*/
	ITC_IRQ_TIM1_CAPCOM = (uint8_t) 12,  /*!< TIM1 capture/compare interrupt */

#if defined(STM8S903) || defined(STM8AF622x)
	ITC_IRQ_TIM5_OVFTRI    = (uint8_t)13,  /*!< TIM5 update/overflow/underflow/trigger/
                                         interrupt */
	ITC_IRQ_TIM5_CAPCOM    = (uint8_t)14,  /*!< TIM5 capture/compare interrupt */
#else
	ITC_IRQ_TIM2_OVF = (uint8_t) 13,  /*!< TIM2 update /overflow interrupt */
	ITC_IRQ_TIM2_CAPCOM = (uint8_t) 14,  /*!< TIM2 capture/compare interrupt */
#endif /*STM8S903 or STM8AF622x */

	ITC_IRQ_TIM3_OVF = (uint8_t) 15,  /*!< TIM3 update /overflow interrupt*/
	ITC_IRQ_TIM3_CAPCOM = (uint8_t) 16,  /*!< TIM3 update /overflow interrupt */

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
	ITC_IRQ_UART1_TX = (uint8_t) 17,  /*!< UART1 TX interrupt */
	ITC_IRQ_UART1_RX = (uint8_t) 18,  /*!< UART1 RX interrupt */
#endif /*STM8S208 or STM8S207 or STM8S007 or STM8S103 or STM8S003 or STM8S903 or STM8AF52Ax or STM8AF62Ax */ 
#if defined(STM8AF622x)
	ITC_IRQ_UART4_TX       = (uint8_t)17,  /*!< UART4 TX interrupt */
	ITC_IRQ_UART4_RX       = (uint8_t)18,  /*!< UART4 RX interrupt */
#endif /*STM8AF622x */

	ITC_IRQ_I2C = (uint8_t) 19,  /*!< I2C interrupt */

#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
	ITC_IRQ_UART2_TX       = (uint8_t)20,  /*!< USART2 TX interrupt */
	ITC_IRQ_UART2_RX       = (uint8_t)21,  /*!< USART2 RX interrupt */
#endif /*STM8S105 or STM8AF626x */

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
	ITC_IRQ_UART3_TX       = (uint8_t)20,  /*!< USART3 TX interrupt */
	ITC_IRQ_UART3_RX       = (uint8_t)21,  /*!< USART3 RX interrupt */
	ITC_IRQ_ADC2           = (uint8_t)22,  /*!< ADC2 interrupt */
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)
	ITC_IRQ_ADC1 = (uint8_t) 22,  /*!< ADC2 interrupt */
#endif /*STM8S105 or STM8S005 or STM8S003 or STM8S103 or STM8S903 or STM8AF626x or STM8AF622x */

#if defined(STM8S903) || defined(STM8AF622x)
	ITC_IRQ_TIM6_OVFTRI    = (uint8_t)23,  /*!< TIM6 update/overflow/underflow/trigger/
                                         interrupt */
#else
	ITC_IRQ_TIM4_OVF = (uint8_t) 23,  /*!< TIM4 update /overflow interrupt */
#endif /*STM8S903 or STM8AF622x */

	ITC_IRQ_EEPROM_EEC = (uint8_t) 24  /*!< Flash interrupt */
} ITC_Irq_TypeDef;

/**
  * @brief  ITC Priority Levels selection
  */
typedef enum {
	ITC_PRIORITYLEVEL_0 = (uint8_t) 0x02, /*!< Software priority level 0 (cannot be written) */
	ITC_PRIORITYLEVEL_1 = (uint8_t) 0x01, /*!< Software priority level 1 */
	ITC_PRIORITYLEVEL_2 = (uint8_t) 0x00, /*!< Software priority level 2 */
	ITC_PRIORITYLEVEL_3 = (uint8_t) 0x03  /*!< Software priority level 3 */
} ITC_PriorityLevel_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup ITC_Exported_Constants
  * @{
  */
#define CPU_SOFT_INT_DISABLED ((uint8_t)0x28) /*!< Mask for I1 and I0 bits in CPU_CC register */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/**
  * @brief  Macros used by the assert function in order to check the different functions parameters.
  * @addtogroup ITC_Private_Macros
  * @{
  */

/* Used by assert function */
#define IS_ITC_IRQ_OK(IRQ) ((IRQ) <= (uint8_t)24)

/* Used by assert function */
#define IS_ITC_PRIORITY_OK(PriorityValue) \
  (((PriorityValue) == ITC_PRIORITYLEVEL_0) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_1) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_2) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_3))

/* Used by assert function */
#define IS_ITC_INTERRUPTS_DISABLED (ITC_GetSoftIntStatus() == CPU_SOFT_INT_DISABLED)

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

/** @addtogroup ITC_Exported_Functions
  * @{
  */

#if 0
uint8_t ITC_GetCPUCC(void);

void ITC_DeInit(void);

uint8_t ITC_GetSoftIntStatus(void);

void ITC_SetSoftwarePriority(ITC_Irq_TypeDef IrqNum, ITC_PriorityLevel_TypeDef PriorityValue);

ITC_PriorityLevel_TypeDef ITC_GetSoftwarePriority(ITC_Irq_TypeDef IrqNum);
#endif



/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup ITC_Private_Functions
  * @{
  */

/**
  * @brief  Utility function used to read CC register.
  * @param  None
  * @retval CPU CC register value
  */
inline uint8_t ITC_GetCPUCC(void)
{
#ifdef _COSMIC_
	_asm("push cc");
	_asm("pop a");
	return; /* Ignore compiler warning, the returned value is in A register */
#elif defined _RAISONANCE_ /* _RAISONANCE_ */
	return _getCC_();
#else /* _IAR_ */
	__asm__("push cc");
	__asm__("pop a"); /* Ignore compiler warning, the returned value is in A register */
#endif /* _COSMIC_*/
}


/**
  * @}
  */

/* Public functions ----------------------------------------------------------*/

/** @addtogroup ITC_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the ITC registers to their default reset value.
  * @param  None
  * @retval None
  */
inline void ITC_DeInit(void)
{
	ITC->ISPR1 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR2 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR3 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR4 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR5 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR6 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR7 = ITC_SPRX_RESET_VALUE;
	ITC->ISPR8 = ITC_SPRX_RESET_VALUE;
}

/**
  * @brief  Gets the interrupt software priority bits (I1, I0) value from CPU CC register.
  * @param  None
  * @retval The interrupt software priority bits value.
  */
inline uint8_t ITC_GetSoftIntStatus(void)
{
	return (uint8_t) (ITC_GetCPUCC() & CPU_CC_I1I0);
}

/**
  * @brief  Gets the software priority of the specified interrupt source.
  * @param  IrqNum : Specifies the peripheral interrupt source.
  * @retval ITC_PriorityLevel_TypeDef : Specifies the software priority of the interrupt source.
  */
inline ITC_PriorityLevel_TypeDef ITC_GetSoftwarePriority(ITC_Irq_TypeDef IrqNum)
{
	uint8_t Value = 0;
	uint8_t Mask = 0;

	/* Check function parameters */
	assert_param(IS_ITC_IRQ_OK((uint8_t) IrqNum));

	/* Define the mask corresponding to the bits position in the SPR register */
	Mask = (uint8_t) (0x03U << (((uint8_t) IrqNum % 4U) * 2U));

	switch (IrqNum) {
		case ITC_IRQ_TLI: /* TLI software priority can be read but has no meaning */
		case ITC_IRQ_AWU:
		case ITC_IRQ_CLK:
		case ITC_IRQ_PORTA:
			Value = (uint8_t) (ITC->ISPR1 & Mask); /* Read software priority */
			break;

		case ITC_IRQ_PORTB:
		case ITC_IRQ_PORTC:
		case ITC_IRQ_PORTD:
		case ITC_IRQ_PORTE:
			Value = (uint8_t) (ITC->ISPR2 & Mask); /* Read software priority */
			break;

#if defined(STM8S208) || defined(STM8AF52Ax)
		case ITC_IRQ_CAN_RX:
		case ITC_IRQ_CAN_TX:
#endif /*STM8S208 or STM8AF52Ax */
#if defined(STM8S903) || defined(STM8AF622x)
			case ITC_IRQ_PORTF:
#endif /*STM8S903 or STM8AF622x */
		case ITC_IRQ_SPI:
		case ITC_IRQ_TIM1_OVF:
			Value = (uint8_t) (ITC->ISPR3 & Mask); /* Read software priority */
			break;

		case ITC_IRQ_TIM1_CAPCOM:
#if defined (STM8S903) || defined (STM8AF622x)
			case ITC_IRQ_TIM5_OVFTRI:
			case ITC_IRQ_TIM5_CAPCOM:
#else
		case ITC_IRQ_TIM2_OVF:
		case ITC_IRQ_TIM2_CAPCOM:
#endif /* STM8S903 or STM8AF622x*/
		case ITC_IRQ_TIM3_OVF:
			Value = (uint8_t) (ITC->ISPR4 & Mask); /* Read software priority */
			break;

		case ITC_IRQ_TIM3_CAPCOM:
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
		case ITC_IRQ_UART1_TX:
		case ITC_IRQ_UART1_RX:
#endif /*STM8S208 or STM8S207 or STM8S007 or STM8S103 or STM8S003 or STM8S903 or STM8AF52Ax or STM8AF62Ax */
#if defined(STM8AF622x)
			case ITC_IRQ_UART4_TX:
			case ITC_IRQ_UART4_RX:
#endif /*STM8AF622x */
		case ITC_IRQ_I2C:
			Value = (uint8_t) (ITC->ISPR5 & Mask); /* Read software priority */
			break;

#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
		case ITC_IRQ_UART2_TX:
		case ITC_IRQ_UART2_RX:
#endif /*STM8S105 or STM8AF626x*/
#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || \
    defined(STM8AF62Ax)
		case ITC_IRQ_UART3_TX:
		case ITC_IRQ_UART3_RX:
		case ITC_IRQ_ADC2:
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */
#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
    defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)
		case ITC_IRQ_ADC1:
#endif /*STM8S105, STM8S005, STM8S103 or STM8S003 or STM8S903 or STM8AF626x or STM8AF622x */
#if defined (STM8S903) || defined (STM8AF622x)
			case ITC_IRQ_TIM6_OVFTRI:
#else
		case ITC_IRQ_TIM4_OVF:
#endif /*STM8S903 or STM8AF622x */
			Value = (uint8_t) (ITC->ISPR6 & Mask); /* Read software priority */
			break;

		case ITC_IRQ_EEPROM_EEC:
			Value = (uint8_t) (ITC->ISPR7 & Mask); /* Read software priority */
			break;

		default:
			break;
	}

	Value >>= (uint8_t) (((uint8_t) IrqNum % 4u) * 2u);

	return ((ITC_PriorityLevel_TypeDef) Value);
}

/**
  * @brief  Sets the software priority of the specified interrupt source.
  * @note   - The modification of the software priority is only possible when
  *         the interrupts are disabled.
  *         - The normal behavior is to disable the interrupt before calling
  *         this function, and re-enable it after.
  *         - The priority level 0 cannot be set (see product specification
  *         for more details).
  * @param  IrqNum : Specifies the peripheral interrupt source.
  * @param  PriorityValue : Specifies the software priority value to set,
  *         can be a value of @ref  ITC_PriorityLevel_TypeDef .
  * @retval None
*/
inline void ITC_SetSoftwarePriority(ITC_Irq_TypeDef IrqNum, ITC_PriorityLevel_TypeDef PriorityValue)
{
	uint8_t Mask = 0;
	uint8_t NewPriority = 0;

	/* Check function parameters */
	assert_param(IS_ITC_IRQ_OK((uint8_t) IrqNum));
	assert_param(IS_ITC_PRIORITY_OK(PriorityValue));

	/* Check if interrupts are disabled */
	assert_param(IS_ITC_INTERRUPTS_DISABLED);

	/* Define the mask corresponding to the bits position in the SPR register */
	/* The mask is reversed in order to clear the 2 bits after more easily */
	Mask = (uint8_t) (~(uint8_t) (0x03U << (((uint8_t) IrqNum % 4U) * 2U)));

	/* Define the new priority to write */
	NewPriority = (uint8_t) ((uint8_t) (PriorityValue) << (((uint8_t) IrqNum % 4U) * 2U));

	switch (IrqNum) {
		case ITC_IRQ_TLI: /* TLI software priority can be written but has no meaning */
		case ITC_IRQ_AWU:
		case ITC_IRQ_CLK:
		case ITC_IRQ_PORTA:
			ITC->ISPR1 &= Mask;
			ITC->ISPR1 |= NewPriority;
			break;

		case ITC_IRQ_PORTB:
		case ITC_IRQ_PORTC:
		case ITC_IRQ_PORTD:
		case ITC_IRQ_PORTE:
			ITC->ISPR2 &= Mask;
			ITC->ISPR2 |= NewPriority;
			break;

#if defined(STM8S208) || defined(STM8AF52Ax)
		case ITC_IRQ_CAN_RX:
		case ITC_IRQ_CAN_TX:
#endif /*STM8S208 or STM8AF52Ax */
#if defined(STM8S903) || defined(STM8AF622x)
			case ITC_IRQ_PORTF:
#endif /*STM8S903 or STM8AF622x */
		case ITC_IRQ_SPI:
		case ITC_IRQ_TIM1_OVF:
			ITC->ISPR3 &= Mask;
			ITC->ISPR3 |= NewPriority;
			break;

		case ITC_IRQ_TIM1_CAPCOM:
#if defined(STM8S903) || defined(STM8AF622x)
			case ITC_IRQ_TIM5_OVFTRI:
			case ITC_IRQ_TIM5_CAPCOM:
#else
		case ITC_IRQ_TIM2_OVF:
		case ITC_IRQ_TIM2_CAPCOM:
#endif /*STM8S903 or STM8AF622x */
		case ITC_IRQ_TIM3_OVF:
			ITC->ISPR4 &= Mask;
			ITC->ISPR4 |= NewPriority;
			break;

		case ITC_IRQ_TIM3_CAPCOM:
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
		case ITC_IRQ_UART1_TX:
		case ITC_IRQ_UART1_RX:
#endif /*STM8S208 or STM8S207 or STM8S007 or STM8S103 or STM8S003 or STM8S903 or STM8AF52Ax or STM8AF62Ax */
#if defined(STM8AF622x)
			case ITC_IRQ_UART4_TX:
			case ITC_IRQ_UART4_RX:
#endif /*STM8AF622x */
		case ITC_IRQ_I2C:
			ITC->ISPR5 &= Mask;
			ITC->ISPR5 |= NewPriority;
			break;

#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
		case ITC_IRQ_UART2_TX:
		case ITC_IRQ_UART2_RX:
#endif /*STM8S105 or STM8AF626x */

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || \
    defined(STM8AF62Ax)
		case ITC_IRQ_UART3_TX:
		case ITC_IRQ_UART3_RX:
		case ITC_IRQ_ADC2:
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
    defined(STM8S903) || defined(STM8AF626x) || defined (STM8AF622x)
		case ITC_IRQ_ADC1:
#endif /*STM8S105, STM8S005, STM8S103 or STM8S003 or STM8S903 or STM8AF626x or STM8AF622x */

#if defined (STM8S903) || defined (STM8AF622x)
			case ITC_IRQ_TIM6_OVFTRI:
#else
		case ITC_IRQ_TIM4_OVF:
#endif /* STM8S903 or STM8AF622x */
			ITC->ISPR6 &= Mask;
			ITC->ISPR6 |= NewPriority;
			break;

		case ITC_IRQ_EEPROM_EEC:
			ITC->ISPR7 &= Mask;
			ITC->ISPR7 |= NewPriority;
			break;

		default:
			break;
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

#endif /* __STM8S_ITC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
