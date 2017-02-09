/**
  ******************************************************************************
  * @file    stm8s_it.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains the headers of the interrupt handlers
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
#ifndef __STM8S_IT_H
#define __STM8S_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef _COSMIC_
 void _stext(void); /* RESET startup routine */
 INTERRUPT void NonHandledInterrupt(void);
#endif /* _COSMIC_ */

#if !defined(_RAISONANCE_) // SDCC patch: interrupt keyword required after function
// INTERRUPT void TRAP_IRQHandler(void); /* TRAP */
 void TLI_IRQHandler(void) INTERRUPT 0; /* TLI */
 void AWU_IRQHandler(void) INTERRUPT 1; /* AWU */
 void CLK_IRQHandler(void) INTERRUPT 2; /* CLOCK */
 void EXTI_PORTA_IRQHandler(void) INTERRUPT 3; /* EXTI PORTA */
 void EXTI_PORTB_IRQHandler(void) INTERRUPT 4; /* EXTI PORTB */
 void EXTI_PORTC_IRQHandler(void) INTERRUPT 5; /* EXTI PORTC */
 void EXTI_PORTD_IRQHandler(void) INTERRUPT 6; /* EXTI PORTD */
 void EXTI_PORTE_IRQHandler(void) INTERRUPT 7; /* EXTI PORTE */

#if defined(STM8S903) || defined(STM8AF622x)
 void EXTI_PORTF_IRQHandler(void) INTERRUPT 8; /* EXTI PORTF */
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined (STM8AF52Ax)
 void CAN_RX_IRQHandler(void) INTERRUPT 8; /* CAN RX */
 void CAN_TX_IRQHandler(void) INTERRUPT 9; /* CAN TX/ER/SC */
#endif /* (STM8S208) || (STM8AF52Ax) */

 void SPI_IRQHandler(void) INTERRUPT 10; /* SPI */
 void TIM1_CAP_COM_IRQHandler(void) INTERRUPT 12; /* TIM1 CAP/COM */
 void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) INTERRUPT 11; /* TIM1 UPD/OVF/TRG/BRK */

#if defined(STM8S903) || defined(STM8AF622x)
 void TIM5_UPD_OVF_BRK_TRG_IRQHandler(void) INTERRUPT 13; /* TIM5 UPD/OVF/BRK/TRG */
 void TIM5_CAP_COM_IRQHandler(void) INTERRUPT 14; /* TIM5 CAP/COM */
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8A626x) */
 void TIM2_UPD_OVF_BRK_IRQHandler(void) INTERRUPT 13; /* TIM2 UPD/OVF/BRK */
 void TIM2_CAP_COM_IRQHandler(void) INTERRUPT 14; /* TIM2 CAP/COM */
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 void TIM3_UPD_OVF_BRK_IRQHandler(void) INTERRUPT 15; /* TIM3 UPD/OVF/BRK */
 void TIM3_CAP_COM_IRQHandler(void) INTERRUPT 16; /* TIM3 CAP/COM */
#endif /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8A626x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8S903)
 void UART1_TX_IRQHandler(void) INTERRUPT 17; /* UART1 TX */
 void UART1_RX_IRQHandler(void) INTERRUPT 18; /* UART1 RX */
#endif /* (STM8S208) || (STM8S207) || (STM8S903) || (STM8S103) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8AF622x)
 void UART4_TX_IRQHandler(void) INTERRUPT 17; /* UART4 TX */
 void UART4_RX_IRQHandler(void) INTERRUPT 18; /* UART4 RX */
#endif /* (STM8AF622x) */

 void I2C_IRQHandler(void) INTERRUPT 19; /* I2C */

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
 void UART2_RX_IRQHandler(void) INTERRUPT 20; /* UART2 RX */
 void UART2_TX_IRQHandler(void) INTERRUPT 21; /* UART2 TX */
#endif /* (STM8S105) || (STM8AF626x) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 void UART3_RX_IRQHandler(void) INTERRUPT 20; /* UART3 RX */
 void UART3_TX_IRQHandler(void) INTERRUPT 21; /* UART3 TX */
#endif /* (STM8S207) || (STM8S208) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 void ADC2_IRQHandler(void) INTERRUPT 22; /* ADC2 */
#else /* (STM8S105) || (STM8S103) || (STM8S903) || (STM8AF622x) */
 void ADC1_IRQHandler(void) INTERRUPT 22; /* ADC1 */
#endif /* (STM8S207) || (STM8S208) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8S903) || defined(STM8AF622x)
 void TIM6_UPD_OVF_TRG_IRQHandler(void) INTERRUPT 23; /* TIM6 UPD/OVF/TRG */
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */
 void TIM4_UPD_OVF_IRQHandler(void) INTERRUPT 23; /* TIM4 UPD/OVF */
#endif /* (STM8S903) || (STM8AF622x) */
 void EEPROM_EEC_IRQHandler(void) INTERRUPT 24; /* EEPROM ECC CORRECTION */
#endif /* _RAISONANCE_ */

#endif /* __STM8S_IT_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
