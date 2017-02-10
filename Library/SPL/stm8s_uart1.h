/**
  ********************************************************************************
  * @file    stm8s_uart1.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototypes and macros for the UART1 peripheral.
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
#ifndef __STM8S_UART1_H
#define __STM8S_UART1_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @addtogroup UART1_Exported_Types
  * @{
  */


/**
  * @brief  UART1 Irda Modes
  */

typedef enum {
	UART1_IRDAMODE_NORMAL = (uint8_t) 0x00, /**< 0x00 Irda Normal Mode   */
	UART1_IRDAMODE_LOWPOWER = (uint8_t) 0x01  /**< 0x01 Irda Low Power Mode */
} UART1_IrDAMode_TypeDef;

/**
  * @brief  UART1 WakeUP Modes
  */
typedef enum {
	UART1_WAKEUP_IDLELINE = (uint8_t) 0x00, /**< 0x01 Idle Line wake up                */
	UART1_WAKEUP_ADDRESSMARK = (uint8_t) 0x08  /**< 0x02 Address Mark wake up          */
} UART1_WakeUp_TypeDef;

/**
  * @brief  UART1 LIN Break detection length possible values
  */
typedef enum {
	UART1_LINBREAKDETECTIONLENGTH_10BITS = (uint8_t) 0x00, /**< 0x01 10 bits Lin Break detection            */
	UART1_LINBREAKDETECTIONLENGTH_11BITS = (uint8_t) 0x01  /**< 0x02 11 bits Lin Break detection          */
} UART1_LINBreakDetectionLength_TypeDef;

/**
  * @brief  UART1 stop bits possible values
  */

typedef enum {
	UART1_STOPBITS_1 = (uint8_t) 0x00,    /**< One stop bit is  transmitted at the end of frame*/
	UART1_STOPBITS_0_5 = (uint8_t) 0x10,    /**< Half stop bits is transmitted at the end of frame*/
	UART1_STOPBITS_2 = (uint8_t) 0x20,    /**< Two stop bits are  transmitted at the end of frame*/
	UART1_STOPBITS_1_5 = (uint8_t) 0x30     /**< One and half stop bits*/
} UART1_StopBits_TypeDef;

/**
  * @brief  UART1 parity possible values
  */
typedef enum {
	UART1_PARITY_NO = (uint8_t) 0x00,      /**< No Parity*/
	UART1_PARITY_EVEN = (uint8_t) 0x04,      /**< Even Parity*/
	UART1_PARITY_ODD = (uint8_t) 0x06       /**< Odd Parity*/
} UART1_Parity_TypeDef;

/**
  * @brief  UART1 Synchrone modes
  */
typedef enum {
	UART1_SYNCMODE_CLOCK_DISABLE = (uint8_t) 0x80, /**< 0x80 Sync mode Disable, SLK pin Disable */
	UART1_SYNCMODE_CLOCK_ENABLE = (uint8_t) 0x08, /**< 0x08 Sync mode Enable, SLK pin Enable     */
	UART1_SYNCMODE_CPOL_LOW = (uint8_t) 0x40, /**< 0x40 Steady low value on SCLK pin outside transmission window */
	UART1_SYNCMODE_CPOL_HIGH = (uint8_t) 0x04, /**< 0x04 Steady high value on SCLK pin outside transmission window */
	UART1_SYNCMODE_CPHA_MIDDLE = (uint8_t) 0x20, /**< 0x20 SCLK clock line activated in middle of data bit     */
	UART1_SYNCMODE_CPHA_BEGINING = (uint8_t) 0x02, /**< 0x02 SCLK clock line activated at beginning of data bit  */
	UART1_SYNCMODE_LASTBIT_DISABLE = (uint8_t) 0x10, /**< 0x10 The clock pulse of the last data bit is not output to the SCLK pin */
	UART1_SYNCMODE_LASTBIT_ENABLE = (uint8_t) 0x01  /**< 0x01 The clock pulse of the last data bit is output to the SCLK pin */
} UART1_SyncMode_TypeDef;

/**
  * @brief  UART1 Word length possible values
  */
typedef enum {
	UART1_WORDLENGTH_8D = (uint8_t) 0x00,/**< 0x00 8 bits Data  */
	UART1_WORDLENGTH_9D = (uint8_t) 0x10 /**< 0x10 9 bits Data  */
} UART1_WordLength_TypeDef;

/**
  * @brief  UART1 Mode possible values
  */
typedef enum {
	UART1_MODE_RX_ENABLE = (uint8_t) 0x08,  /**< 0x08 Receive Enable */
	UART1_MODE_TX_ENABLE = (uint8_t) 0x04,  /**< 0x04 Transmit Enable */
	UART1_MODE_TX_DISABLE = (uint8_t) 0x80,  /**< 0x80 Transmit Disable */
	UART1_MODE_RX_DISABLE = (uint8_t) 0x40,  /**< 0x40 Single-wire Half-duplex mode */
	UART1_MODE_TXRX_ENABLE = (uint8_t) 0x0C  /**< 0x0C Transmit Enable and Receive Enable */
} UART1_Mode_TypeDef;

/**
  * @brief  UART1 Flag possible values
  */
typedef enum {
	UART1_FLAG_TXE = (uint16_t) 0x0080, /*!< Transmit Data Register Empty flag */
	UART1_FLAG_TC = (uint16_t) 0x0040, /*!< Transmission Complete flag */
	UART1_FLAG_RXNE = (uint16_t) 0x0020, /*!< Read Data Register Not Empty flag */
	UART1_FLAG_IDLE = (uint16_t) 0x0010, /*!< Idle line detected flag */
	UART1_FLAG_OR = (uint16_t) 0x0008, /*!< OverRun error flag */
	UART1_FLAG_NF = (uint16_t) 0x0004, /*!< Noise error flag */
	UART1_FLAG_FE = (uint16_t) 0x0002, /*!< Framing Error flag */
	UART1_FLAG_PE = (uint16_t) 0x0001, /*!< Parity Error flag */
	UART1_FLAG_LBDF = (uint16_t) 0x0210, /*!< Line Break Detection Flag */
	UART1_FLAG_SBK = (uint16_t) 0x0101  /*!< Send Break characters Flag */
} UART1_Flag_TypeDef;

/**
  * @brief  UART1 Interrupt definition
  * UART1_IT possible values
  * Elements values convention: 0xZYX
  * X: Position of the corresponding Interrupt
  *   - For the following values, X means the interrupt position in the CR2 register.
  *     UART1_IT_TXE
  *     UART1_IT_TC
  *     UART1_IT_RXNE
  *     UART1_IT_IDLE 
  *     UART1_IT_OR 
  *   - For the UART1_IT_PE value, X means the flag position in the CR1 register.
  *   - For the UART1_IT_LBDF value, X means the flag position in the CR4 register.
  * Y: Flag position
  *  - For the following values, Y means the flag (pending bit) position in the SR register.
  *     UART1_IT_TXE
  *     UART1_IT_TC
  *     UART1_IT_RXNE
  *     UART1_IT_IDLE 
  *     UART1_IT_OR
  *     UART1_IT_PE
  *  - For the UART1_IT_LBDF value, Y means the flag position in the CR4 register.
  * Z: Register index: indicate in which register the dedicated interrupt source is:
  *  - 1==> CR1 register
  *  - 2==> CR2 register
  *  - 3==> CR4 register
  */
typedef enum {
	UART1_IT_TXE = (uint16_t) 0x0277, /*!< Transmit interrupt */
	UART1_IT_TC = (uint16_t) 0x0266, /*!< Transmission Complete interrupt */
	UART1_IT_RXNE = (uint16_t) 0x0255, /*!< Receive interrupt */
	UART1_IT_IDLE = (uint16_t) 0x0244, /*!< IDLE line interrupt */
	UART1_IT_OR = (uint16_t) 0x0235, /*!< Overrun Error interrupt */
	UART1_IT_PE = (uint16_t) 0x0100, /*!< Parity Error interrupt */
	UART1_IT_LBDF = (uint16_t) 0x0346, /**< LIN break detection interrupt */
	UART1_IT_RXNE_OR = (uint16_t) 0x0205  /*!< Receive/Overrun interrupt */
} UART1_IT_TypeDef;



/**
 * Baud rates at 16 MHz
 */
typedef enum {
	UART_BAUD_9600 = (uint16_t) 0x0368,
	UART_BAUD_19200 = (uint16_t) 0x0134,
	UART_BAUD_57600 = (uint16_t) 0x0611,
	UART_BAUD_115200 = (uint16_t) 0x0B08,
	UART_BAUD_230400 = (uint16_t) 0x0504,
	UART_BAUD_460800 = (uint16_t) 0x0302,
	UART_BAUD_921600 = (uint16_t) 0x0101
} UART_Baud_TypeDef;

/**
 * Initialize UART for a 16 MHz clock to 8-bit, 1 stopbit, no parity.
 * Sets baud rate, enables Tx and Rx and turns on the peripheral.
 *
 * @param baud - UART_BAUD_*
 */
void inline UART_SimpleInit(UART_Baud_TypeDef baud) {
	UART1->BRR2 = (uint8_t) (((baud) >> 8) & 0xFF);
	UART1->BRR1 = (uint8_t) ((baud) & 0xFF);
	UART1->CR2 = (uint8_t) (UART1_CR2_TEN | UART1_CR2_REN);
}

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macros ------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/** @addtogroup UART1_Private_Macros
  * @{
  */

/**
  * @brief  Macro used by the assert function to check the different functions parameters.
  */

/**
 * @brief Macro used by the assert_param function in order to check the different
 *        sensitivity values for the MODEs possible combination should be one of
 *        the following
 */
#define IS_UART1_MODE_OK(Mode) \
  (((Mode) == (uint8_t)UART1_MODE_RX_ENABLE) || \
   ((Mode) == (uint8_t)UART1_MODE_RX_DISABLE) || \
   ((Mode) == (uint8_t)UART1_MODE_TX_ENABLE) || \
   ((Mode) == (uint8_t)UART1_MODE_TX_DISABLE) || \
   ((Mode) == (uint8_t)UART1_MODE_TXRX_ENABLE) || \
   ((Mode) == (uint8_t)((uint8_t)UART1_MODE_TX_ENABLE|(uint8_t)UART1_MODE_RX_ENABLE)) || \
   ((Mode) == (uint8_t)((uint8_t)UART1_MODE_TX_ENABLE|(uint8_t)UART1_MODE_RX_DISABLE)) || \
   ((Mode) == (uint8_t)((uint8_t)UART1_MODE_TX_DISABLE|(uint8_t)UART1_MODE_RX_DISABLE)) || \
   ((Mode) == (uint8_t)((uint8_t)UART1_MODE_TX_DISABLE|(uint8_t)UART1_MODE_RX_ENABLE)))

/**
 * @brief Macro used by the assert_param function in order to check the different
 *        sensitivity values for the WordLengths
 */
#define IS_UART1_WORDLENGTH_OK(WordLength) \
  (((WordLength) == UART1_WORDLENGTH_8D) || \
   ((WordLength) == UART1_WORDLENGTH_9D))

/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the SyncModes; it should exclude values such 
  *         as  UART1_CLOCK_ENABLE|UART1_CLOCK_DISABLE
  */
#define IS_UART1_SYNCMODE_OK(SyncMode) \
  (!((((SyncMode)&(((uint8_t)UART1_SYNCMODE_CLOCK_ENABLE)|((uint8_t)UART1_SYNCMODE_CLOCK_DISABLE))) == (((uint8_t)UART1_SYNCMODE_CLOCK_ENABLE)|((uint8_t)UART1_SYNCMODE_CLOCK_DISABLE))) \
    || (((SyncMode)&(((uint8_t)UART1_SYNCMODE_CPOL_LOW )|((uint8_t)UART1_SYNCMODE_CPOL_HIGH))) == (((uint8_t)UART1_SYNCMODE_CPOL_LOW )|((uint8_t)UART1_SYNCMODE_CPOL_HIGH))) \
    ||(((SyncMode)&(((uint8_t)UART1_SYNCMODE_CPHA_MIDDLE)|((uint8_t)UART1_SYNCMODE_CPHA_BEGINING))) ==  (((uint8_t)UART1_SYNCMODE_CPHA_MIDDLE)|((uint8_t)UART1_SYNCMODE_CPHA_BEGINING))) \
    || (((SyncMode)&(((uint8_t)UART1_SYNCMODE_LASTBIT_DISABLE)|((uint8_t)UART1_SYNCMODE_LASTBIT_ENABLE))) == (((uint8_t)UART1_SYNCMODE_LASTBIT_DISABLE)|((uint8_t)UART1_SYNCMODE_LASTBIT_ENABLE)))))

/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the FLAGs
  */
#define IS_UART1_FLAG_OK(Flag) \
  (((Flag) == UART1_FLAG_TXE) || \
   ((Flag) == UART1_FLAG_TC)  || \
   ((Flag) == UART1_FLAG_RXNE) || \
   ((Flag) == UART1_FLAG_IDLE) || \
   ((Flag) == UART1_FLAG_OR) || \
   ((Flag) == UART1_FLAG_NF) || \
   ((Flag) == UART1_FLAG_FE) || \
   ((Flag) == UART1_FLAG_PE) || \
   ((Flag) == UART1_FLAG_SBK) || \
   ((Flag) == UART1_FLAG_LBDF))
/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the FLAGs that can be cleared by writing 0
  */
#define IS_UART1_CLEAR_FLAG_OK(Flag) \
  (((Flag) == UART1_FLAG_RXNE) || \
   ((Flag) == UART1_FLAG_LBDF))



/**
  * @brief  Macro used by the assert_param function in order to check the different 
  *         sensitivity values for the Interrupts
  */

#define IS_UART1_CONFIG_IT_OK(Interrupt) \
  (((Interrupt) == UART1_IT_PE) || \
   ((Interrupt) == UART1_IT_TXE) || \
   ((Interrupt) == UART1_IT_TC) || \
   ((Interrupt) == UART1_IT_RXNE_OR ) || \
   ((Interrupt) == UART1_IT_IDLE) || \
   ((Interrupt) == UART1_IT_LBDF))

/**
  * @brief  Macro used by the assert function in order to check the different 
  *         sensitivity values for the pending bit
  */
#define IS_UART1_GET_IT_OK(ITPendingBit) \
  (((ITPendingBit) == UART1_IT_TXE)  || \
   ((ITPendingBit) == UART1_IT_TC)   || \
   ((ITPendingBit) == UART1_IT_RXNE) || \
   ((ITPendingBit) == UART1_IT_IDLE) || \
   ((ITPendingBit) == UART1_IT_OR)  || \
   ((ITPendingBit) == UART1_IT_LBDF)  || \
   ((ITPendingBit) == UART1_IT_PE))

/**
  * @brief  Macro used by the assert function in order to check the different 
  *         sensitivity values for the pending bit that can be cleared by writing 0
  */
#define IS_UART1_CLEAR_IT_OK(ITPendingBit) \
  (((ITPendingBit) == UART1_IT_RXNE) || \
   ((ITPendingBit) == UART1_IT_LBDF))


/**
 * @brief Macro used by the assert_param function in order to check the different
 *        sensitivity values for the IrDAModes
 */
#define IS_UART1_IRDAMODE_OK(IrDAMode) \
  (((IrDAMode) == UART1_IRDAMODE_LOWPOWER) || \
   ((IrDAMode) == UART1_IRDAMODE_NORMAL))

/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the WakeUps
  */
#define IS_UART1_WAKEUP_OK(WakeUp) \
  (((WakeUp) == UART1_WAKEUP_IDLELINE) || \
   ((WakeUp) == UART1_WAKEUP_ADDRESSMARK))

/**
  * @brief  Macro used by the assert_param function in order to check the different 
  *        sensitivity values for the LINBreakDetectionLengths
  */
#define IS_UART1_LINBREAKDETECTIONLENGTH_OK(LINBreakDetectionLength) \
  (((LINBreakDetectionLength) == UART1_LINBREAKDETECTIONLENGTH_10BITS) || \
   ((LINBreakDetectionLength) == UART1_LINBREAKDETECTIONLENGTH_11BITS))

/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the UART1_StopBits
  */
#define IS_UART1_STOPBITS_OK(StopBit) (((StopBit) == UART1_STOPBITS_1) || \
                                       ((StopBit) == UART1_STOPBITS_0_5) || \
                                       ((StopBit) == UART1_STOPBITS_2) || \
                                       ((StopBit) == UART1_STOPBITS_1_5 ))

/**
 * @brief Macro used by the assert_param function in order to check the different
 *        sensitivity values for the Parity
 */
#define IS_UART1_PARITY_OK(Parity) (((Parity) == UART1_PARITY_NO) || \
                                    ((Parity) == UART1_PARITY_EVEN) || \
                                    ((Parity) == UART1_PARITY_ODD ))

/**
 * @brief Macro used by the assert_param function in order to check the maximum
 *        baudrate value
 */
#define IS_UART1_BAUDRATE_OK(NUM) ((NUM) <= (uint32_t)625000)


/**
 * @brief Macro used by the assert_param function in order to check the address
 *        of the UART1 or UART node
 */
#define UART1_ADDRESS_MAX ((uint8_t)16)
#define IS_UART1_ADDRESS_OK(node) ((node) < UART1_ADDRESS_MAX )

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

#if 0
/** @addtogroup UART1_Exported_Functions
  * @{
  */
void UART1_DeInit(void);

void UART1_Init(uint32_t BaudRate, UART1_WordLength_TypeDef WordLength,
				UART1_StopBits_TypeDef StopBits, UART1_Parity_TypeDef Parity,
				UART1_SyncMode_TypeDef SyncMode, UART1_Mode_TypeDef Mode);

void UART1_Cmd(FunctionalState NewState);

void UART1_ITConfig(UART1_IT_TypeDef UART1_IT, FunctionalState NewState);

void UART1_HalfDuplexCmd(FunctionalState NewState);

void UART1_IrDAConfig(UART1_IrDAMode_TypeDef UART1_IrDAMode);

void UART1_IrDACmd(FunctionalState NewState);

void UART1_LINBreakDetectionConfig(UART1_LINBreakDetectionLength_TypeDef UART1_LINBreakDetectionLength);

void UART1_LINCmd(FunctionalState NewState);

void UART1_SmartCardCmd(FunctionalState NewState);

void UART1_SmartCardNACKCmd(FunctionalState NewState);

void UART1_WakeUpConfig(UART1_WakeUp_TypeDef UART1_WakeUp);

void UART1_ReceiverWakeUpCmd(FunctionalState NewState);

uint8_t UART1_ReceiveData8(void);

uint16_t UART1_ReceiveData9(void);

void UART1_SendData8(uint8_t Data);

void UART1_SendData9(uint16_t Data);

void UART1_SendBreak(void);

void UART1_SetAddress(uint8_t UART1_Address);

void UART1_SetGuardTime(uint8_t UART1_GuardTime);

void UART1_SetPrescaler(uint8_t UART1_Prescaler);

FlagStatus UART1_GetFlagStatus(UART1_Flag_TypeDef UART1_FLAG);

void UART1_ClearFlag(UART1_Flag_TypeDef UART1_FLAG);

ITStatus UART1_GetITStatus(UART1_IT_TypeDef UART1_IT);

void UART1_ClearITPendingBit(UART1_IT_TypeDef UART1_IT);
#endif
/**
  * @}
  */

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

/** @}
  * @addtogroup UART1_Public_Functions
  * @{
  */
/**
  * @brief  Deinitializes the UART peripheral.
  * @param  None
  * @retval None
	*/
inline void UART1_DeInit(void)
{
	/* Clear the Idle Line Detected bit in the status register by a read
	to the UART1_SR register followed by a Read to the UART1_DR register */
	(void) UART1->SR;
	(void) UART1->DR;

	UART1->BRR2 = UART1_BRR2_RESET_VALUE;  /* Set UART1_BRR2 to reset value 0x00 */
	UART1->BRR1 = UART1_BRR1_RESET_VALUE;  /* Set UART1_BRR1 to reset value 0x00 */

	UART1->CR1 = UART1_CR1_RESET_VALUE;  /* Set UART1_CR1 to reset value 0x00 */
	UART1->CR2 = UART1_CR2_RESET_VALUE;  /* Set UART1_CR2 to reset value 0x00 */
	UART1->CR3 = UART1_CR3_RESET_VALUE;  /* Set UART1_CR3 to reset value 0x00 */
	UART1->CR4 = UART1_CR4_RESET_VALUE;  /* Set UART1_CR4 to reset value 0x00 */
	UART1->CR5 = UART1_CR5_RESET_VALUE;  /* Set UART1_CR5 to reset value 0x00 */

	UART1->GTR = UART1_GTR_RESET_VALUE;
	UART1->PSCR = UART1_PSCR_RESET_VALUE;
}

/**
  * @brief  Initializes the UART1 according to the specified parameters.
  * @note   Configure in Push Pull or Open Drain mode the Tx pin by setting the
  *         correct I/O Port register according the product package and line
  *         configuration
  * @param  BaudRate: The baudrate.
  * @param  WordLength : This parameter can be any of the
  *         @ref UART1_WordLength_TypeDef enumeration.
  * @param  StopBits: This parameter can be any of the
  *         @ref UART1_StopBits_TypeDef enumeration.
  * @param  Parity: This parameter can be any of the
  *         @ref UART1_Parity_TypeDef enumeration.
  * @param  SyncMode: This parameter can be any of the
  *         @ref UART1_SyncMode_TypeDef values.
  * @param  Mode: This parameter can be any of the @ref UART1_Mode_TypeDef values
  * @retval None
  */
inline void UART1_Init(uint32_t BaudRate, UART1_WordLength_TypeDef WordLength,
					   UART1_StopBits_TypeDef StopBits, UART1_Parity_TypeDef Parity,
					   UART1_SyncMode_TypeDef SyncMode, UART1_Mode_TypeDef Mode)
{
	uint32_t BaudRate_Mantissa = 0, BaudRate_Mantissa100 = 0;

	/* Check the parameters */
	assert_param(IS_UART1_BAUDRATE_OK(BaudRate));
	assert_param(IS_UART1_WORDLENGTH_OK(WordLength));
	assert_param(IS_UART1_STOPBITS_OK(StopBits));
	assert_param(IS_UART1_PARITY_OK(Parity));
	assert_param(IS_UART1_MODE_OK((uint8_t) Mode));
	assert_param(IS_UART1_SYNCMODE_OK((uint8_t) SyncMode));

	/* Clear the word length bit */
	UART1->CR1 &= (uint8_t) (~UART1_CR1_M);

	/* Set the word length bit according to UART1_WordLength value */
	UART1->CR1 |= (uint8_t) WordLength;

	/* Clear the STOP bits */
	UART1->CR3 &= (uint8_t) (~UART1_CR3_STOP);
	/* Set the STOP bits number according to UART1_StopBits value  */
	UART1->CR3 |= (uint8_t) StopBits;

	/* Clear the Parity Control bit */
	UART1->CR1 &= (uint8_t) (~(UART1_CR1_PCEN | UART1_CR1_PS));
	/* Set the Parity Control bit to UART1_Parity value */
	UART1->CR1 |= (uint8_t) Parity;

	/* Clear the LSB mantissa of UART1DIV  */
	UART1->BRR1 &= (uint8_t) (~UART1_BRR1_DIVM);
	/* Clear the MSB mantissa of UART1DIV  */
	UART1->BRR2 &= (uint8_t) (~UART1_BRR2_DIVM);
	/* Clear the Fraction bits of UART1DIV */
	UART1->BRR2 &= (uint8_t) (~UART1_BRR2_DIVF);

	/* Set the UART1 BaudRates in BRR1 and BRR2 registers according to UART1_BaudRate value */
	BaudRate_Mantissa = ((uint32_t) CLK_GetClockFreq() / (BaudRate << 4));
	BaudRate_Mantissa100 = (((uint32_t) CLK_GetClockFreq() * 100) / (BaudRate << 4));
	/* Set the fraction of UART1DIV  */
	UART1->BRR2 |= (uint8_t) ((uint8_t) (((BaudRate_Mantissa100 - (BaudRate_Mantissa * 100)) << 4) / 100) &
							  (uint8_t) 0x0F);
	/* Set the MSB mantissa of UART1DIV  */
	UART1->BRR2 |= (uint8_t) ((BaudRate_Mantissa >> 4) & (uint8_t) 0xF0);
	/* Set the LSB mantissa of UART1DIV  */
	UART1->BRR1 |= (uint8_t) BaudRate_Mantissa;

	/* Disable the Transmitter and Receiver before setting the LBCL, CPOL and CPHA bits */
	UART1->CR2 &= (uint8_t) ~(UART1_CR2_TEN | UART1_CR2_REN);
	/* Clear the Clock Polarity, lock Phase, Last Bit Clock pulse */
	UART1->CR3 &= (uint8_t) ~(UART1_CR3_CPOL | UART1_CR3_CPHA | UART1_CR3_LBCL);
	/* Set the Clock Polarity, lock Phase, Last Bit Clock pulse */
	UART1->CR3 |= (uint8_t) ((uint8_t) SyncMode & (uint8_t) (UART1_CR3_CPOL |
															 UART1_CR3_CPHA | UART1_CR3_LBCL));

	if ((uint8_t) (Mode & UART1_MODE_TX_ENABLE)) {
		/* Set the Transmitter Enable bit */
		UART1->CR2 |= (uint8_t) UART1_CR2_TEN;
	} else {
		/* Clear the Transmitter Disable bit */
		UART1->CR2 &= (uint8_t) (~UART1_CR2_TEN);
	}
	if ((uint8_t) (Mode & UART1_MODE_RX_ENABLE)) {
		/* Set the Receiver Enable bit */
		UART1->CR2 |= (uint8_t) UART1_CR2_REN;
	} else {
		/* Clear the Receiver Disable bit */
		UART1->CR2 &= (uint8_t) (~UART1_CR2_REN);
	}
	/* Set the Clock Enable bit, lock Polarity, lock Phase and Last Bit Clock
	pulse bits according to UART1_Mode value */
	if ((uint8_t) (SyncMode & UART1_SYNCMODE_CLOCK_DISABLE)) {
		/* Clear the Clock Enable bit */
		UART1->CR3 &= (uint8_t) (~UART1_CR3_CKEN);
	} else {
		UART1->CR3 |= (uint8_t) ((uint8_t) SyncMode & UART1_CR3_CKEN);
	}
}

/**
  * @brief  Enable the UART1 peripheral.
  * @param  NewState : The new state of the UART Communication.
  *         This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
inline void UART1_Cmd(FunctionalState NewState)
{
	if (NewState != DISABLE) {
		/* UART1 Enable */
		UART1->CR1 &= (uint8_t) (~UART1_CR1_UARTD);
	} else {
		/* UART Disable */
		UART1->CR1 |= UART1_CR1_UARTD;
	}
}

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  UART1_IT specifies the USART interrupt sources to be enabled or disabled.
  *         This parameter can be one of the following values:
  *         - UART1_IT_TXE:  Transmit Data Register empty interrupt
  *         - UART1_IT_TC:   Transmission complete interrupt
  *         - UART1_IT_RXNE_OR: Receive Data register not empty and Overrun interrupt
  *         - UART1_IT_IDLE: Idle line detection interrupt
  *         - USRT1_IT_ERR:  Error interrupt
  * @param  NewState new state of the specified USART interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_ITConfig(UART1_IT_TypeDef UART1_IT, FunctionalState NewState)
{
	uint8_t uartreg = 0, itpos = 0x00;

	/* Check the parameters */
	assert_param(IS_UART1_CONFIG_IT_OK(UART1_IT));
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	/* Get the UART1 register index */
	uartreg = (uint8_t) ((uint16_t) UART1_IT >> 0x08);
	/* Get the UART1 IT index */
	itpos = (uint8_t) ((uint8_t) 1 << (uint8_t) ((uint8_t) UART1_IT & (uint8_t) 0x0F));

	if (NewState != DISABLE) {
		/**< Enable the Interrupt bits according to UART1_IT mask */
		if (uartreg == 0x01) {
			UART1->CR1 |= itpos;
		} else if (uartreg == 0x02) {
			UART1->CR2 |= itpos;
		} else {
			UART1->CR4 |= itpos;
		}
	} else {
		/**< Disable the interrupt bits according to UART1_IT mask */
		if (uartreg == 0x01) {
			UART1->CR1 &= (uint8_t) (~itpos);
		} else if (uartreg == 0x02) {
			UART1->CR2 &= (uint8_t) (~itpos);
		} else {
			UART1->CR4 &= (uint8_t) (~itpos);
		}
	}

}

/**
  * @brief  Enables or disables the UART's Half Duplex communication.
  * @param  NewState new state of the UART Communication.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_HalfDuplexCmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		UART1->CR5 |= UART1_CR5_HDSEL;  /**< UART1 Half Duplex Enable  */
	} else {
		UART1->CR5 &= (uint8_t) ~UART1_CR5_HDSEL; /**< UART1 Half Duplex Disable */
	}
}

/**
  * @brief  Configures the UART's IrDA interface.
  * @param  UART1_IrDAMode specifies the IrDA mode.
  *         This parameter can be any of the @ref UART1_IrDAMode_TypeDef values.
  * @retval None
  */
inline void UART1_IrDAConfig(UART1_IrDAMode_TypeDef UART1_IrDAMode)
{
	assert_param(IS_UART1_IRDAMODE_OK(UART1_IrDAMode));

	if (UART1_IrDAMode != UART1_IRDAMODE_NORMAL) {
		UART1->CR5 |= UART1_CR5_IRLP;
	} else {
		UART1->CR5 &= ((uint8_t) ~UART1_CR5_IRLP);
	}
}

/**
  * @brief  Enables or disables the UART's IrDA interface.
  * @param  NewState new state of the IrDA mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_IrDACmd(FunctionalState NewState)
{
	/* Check parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the IrDA mode by setting the IREN bit in the CR3 register */
		UART1->CR5 |= UART1_CR5_IREN;
	} else {
		/* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
		UART1->CR5 &= ((uint8_t) ~UART1_CR5_IREN);
	}
}

/**
  * @brief  Sets the UART1 LIN Break detection length.
  * @param  UART1_LINBreakDetectionLength specifies the LIN break detection length.
  *         This parameter can be any of the
  *         @ref UART1_LINBreakDetectionLength_TypeDef values.
  * @retval None
  */
inline void UART1_LINBreakDetectionConfig(UART1_LINBreakDetectionLength_TypeDef UART1_LINBreakDetectionLength)
{
	assert_param(IS_UART1_LINBREAKDETECTIONLENGTH_OK(UART1_LINBreakDetectionLength));

	if (UART1_LINBreakDetectionLength != UART1_LINBREAKDETECTIONLENGTH_10BITS) {
		UART1->CR4 |= UART1_CR4_LBDL;
	} else {
		UART1->CR4 &= ((uint8_t) ~UART1_CR4_LBDL);
	}
}

/**
  * @brief  Enables or disables the UART1's LIN mode.
  * @param  NewState is new state of the UART1 LIN mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_LINCmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the LIN mode by setting the LINE bit in the CR2 register */
		UART1->CR3 |= UART1_CR3_LINEN;
	} else {
		/* Disable the LIN mode by clearing the LINE bit in the CR2 register */
		UART1->CR3 &= ((uint8_t) ~UART1_CR3_LINEN);
	}
}

/**
  * @brief  Enables or disables the UART1 Smart Card mode.
  * @param  NewState: new state of the Smart Card mode.
  * This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_SmartCardCmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the SC mode by setting the SCEN bit in the CR5 register */
		UART1->CR5 |= UART1_CR5_SCEN;
	} else {
		/* Disable the SC mode by clearing the SCEN bit in the CR5 register */
		UART1->CR5 &= ((uint8_t) (~UART1_CR5_SCEN));
	}
}

/**
  * @brief  Enables or disables NACK transmission.
  * @note   This function is valid only for UART1 because is related to SmartCard mode.
  * @param  NewState: new state of the Smart Card mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_SmartCardNACKCmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the NACK transmission by setting the NACK bit in the CR5 register */
		UART1->CR5 |= UART1_CR5_NACK;
	} else {
		/* Disable the NACK transmission by clearing the NACK bit in the CR5 register */
		UART1->CR5 &= ((uint8_t) ~(UART1_CR5_NACK));
	}
}

/**
  * @brief  Selects the UART1 WakeUp method.
  * @param  UART1_WakeUp: specifies the UART1 wakeup method.
  *         This parameter can be any of the @ref UART1_WakeUp_TypeDef values.
  * @retval None
  */
inline void UART1_WakeUpConfig(UART1_WakeUp_TypeDef UART1_WakeUp)
{
	assert_param(IS_UART1_WAKEUP_OK(UART1_WakeUp));

	UART1->CR1 &= ((uint8_t) ~UART1_CR1_WAKE);
	UART1->CR1 |= (uint8_t) UART1_WakeUp;
}

/**
  * @brief  Determines if the UART1 is in mute mode or not.
  * @param  NewState: new state of the UART1 mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void UART1_ReceiverWakeUpCmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		/* Enable the mute mode UART1 by setting the RWU bit in the CR2 register */
		UART1->CR2 |= UART1_CR2_RWU;
	} else {
		/* Disable the mute mode UART1 by clearing the RWU bit in the CR1 register */
		UART1->CR2 &= ((uint8_t) ~UART1_CR2_RWU);
	}
}

/**
  * @brief  Returns the most recent received data by the UART1 peripheral.
  * @param  None
  * @retval The received data.
  */
inline uint8_t UART1_ReceiveData8(void)
{
	return ((uint8_t) UART1->DR);
}

/**
  * @brief  Returns the most recent received data by the UART1 peripheral.
  * @param  None
  * @retval The received data.
  */
inline uint16_t UART1_ReceiveData9(void)
{
	uint16_t temp = 0;

	temp = (uint16_t) (((uint16_t) ((uint16_t) UART1->CR1 & (uint16_t) UART1_CR1_R8)) << 1);
	return (uint16_t) ((((uint16_t) UART1->DR) | temp) & ((uint16_t) 0x01FF));
}

/**
  * @brief  Transmits 8 bit data through the UART1 peripheral.
  * @param  Data: The data to transmit.
  * @retval None
  */
inline void UART1_SendData8(uint8_t Data)
{
	/* Transmit Data */
	UART1->DR = Data;
}

/**
  * @brief  Transmits 9 bit data through the UART peripheral.
  * @param  Data : The data to transmit.
  *         This parameter should be lower than 0x1FF.
  * @retval None
  */
inline void UART1_SendData9(uint16_t Data)
{
	/**< Clear the transmit data bit 8 [8]  */
	UART1->CR1 &= ((uint8_t) ~UART1_CR1_T8);
	/**< Write the transmit data bit [8]  */
	UART1->CR1 |= (uint8_t) (((uint8_t) (Data >> 2)) & UART1_CR1_T8);
	/**< Write the transmit data bit [0:7] */
	UART1->DR = (uint8_t) (Data);
}

/**
  * @brief  Transmits break characters.
  * @param  None
  * @retval None
  */
inline void UART1_SendBreak(void)
{
	UART1->CR2 |= UART1_CR2_SBK;
}

/**
  * @brief  Sets the address of the UART1 node.
  * @param  UART1_Address: Indicates the address of the UART1 node.
  * @retval None
  */
inline void UART1_SetAddress(uint8_t UART1_Address)
{
	/*assert_param for UART1_Address*/
	assert_param(IS_UART1_ADDRESS_OK(UART1_Address));

	/* Clear the UART1 address */
	UART1->CR4 &= ((uint8_t) ~UART1_CR4_ADD);
	/* Set the UART1 address node */
	UART1->CR4 |= UART1_Address;
}

/**
  * @brief  Sets the specified UART guard time.
  * @note   SmartCard Mode should be Enabled
  * @param  UART1_GuardTime: specifies the guard time.
  * @retval None
  */
inline void UART1_SetGuardTime(uint8_t UART1_GuardTime)
{
	/* Set the UART1 guard time */
	UART1->GTR = UART1_GuardTime;
}

/**
  * @brief  Sets the system clock prescaler.
  * @note   IrDA Low Power mode or smartcard mode should be enabled
  * @note   This function is related to SmartCard and IrDa mode.
  * @param  UART1_Prescaler: specifies the prescaler clock.
  *         This parameter can be one of the following values:
  *         @par IrDA Low Power Mode
  *         The clock source is divided by the value given in the register (8 bits)
  *         - 0000 0000 Reserved
  *         - 0000 0001 divides the clock source by 1
  *         - 0000 0010 divides the clock source by 2
  *         - ...
  *        @par Smart Card Mode
  *        The clock source is divided by the value given in the register
  *        (5 significant bits) multiplied by 2
  *         - 0 0000 Reserved
  *         - 0 0001 divides the clock source by 2
  *         - 0 0010 divides the clock source by 4
  *         - 0 0011 divides the clock source by 6
  *         - ...
  * @retval None
  */
inline void UART1_SetPrescaler(uint8_t UART1_Prescaler)
{
	/* Load the UART1 prescaler value*/
	UART1->PSCR = UART1_Prescaler;
}

/**
  * @brief  Checks whether the specified UART1 flag is set or not.
  * @param  UART1_FLAG specifies the flag to check.
  *         This parameter can be any of the @ref UART1_Flag_TypeDef enumeration.
  * @retval FlagStatus (SET or RESET)
  */
inline FlagStatus UART1_GetFlagStatus(UART1_Flag_TypeDef UART1_FLAG)
{
	FlagStatus status = RESET;

	/* Check parameters */
	assert_param(IS_UART1_FLAG_OK(UART1_FLAG));


	/* Check the status of the specified UART1 flag*/
	if (UART1_FLAG == UART1_FLAG_LBDF) {
		if ((UART1->CR4 & (uint8_t) UART1_FLAG) != (uint8_t) 0x00) {
			/* UART1_FLAG is set*/
			status = SET;
		} else {
			/* UART1_FLAG is reset*/
			status = RESET;
		}
	} else if (UART1_FLAG == UART1_FLAG_SBK) {
		if ((UART1->CR2 & (uint8_t) UART1_FLAG) != (uint8_t) 0x00) {
			/* UART1_FLAG is set*/
			status = SET;
		} else {
			/* UART1_FLAG is reset*/
			status = RESET;
		}
	} else {
		if ((UART1->SR & (uint8_t) UART1_FLAG) != (uint8_t) 0x00) {
			/* UART1_FLAG is set*/
			status = SET;
		} else {
			/* UART1_FLAG is reset*/
			status = RESET;
		}
	}
	/* Return the UART1_FLAG status*/
	return status;
}

/**
  * @brief  Clears the UART1 flags.
  * @param  UART1_FLAG specifies the flag to clear
  *         This parameter can be any combination of the following values:
  *         - UART1_FLAG_LBDF: LIN Break detection flag.
  *         - UART1_FLAG_RXNE: Receive data register not empty flag.
  * @note
  *         - PE (Parity error), FE (Framing error), NE (Noise error),
  *         OR (OverRun error) and IDLE (Idle line detected) flags are
  *         cleared by software sequence: a read operation to UART1_SR register
  *         (UART1_GetFlagStatus())followed by a read operation to UART1_DR
  *         register(UART1_ReceiveData8() or UART1_ReceiveData9()).
  *
  *         - RXNE flag can be also cleared by a read to the UART1_DR register
  *         (UART1_ReceiveData8()or UART1_ReceiveData9()).
  *
  *         - TC flag can be also cleared by software sequence: a read operation
  *         to UART1_SR register (UART1_GetFlagStatus()) followed by a write
  *         operation to UART1_DR register (UART1_SendData8() or UART1_SendData9()).
  *
  *         - TXE flag is cleared only by a write to the UART1_DR register
  *         (UART1_SendData8() or UART1_SendData9()).
  *
  *         - SBK flag is cleared during the stop bit of break.
  * @retval None
  */
inline void UART1_ClearFlag(UART1_Flag_TypeDef UART1_FLAG)
{
	assert_param(IS_UART1_CLEAR_FLAG_OK(UART1_FLAG));

	/* Clear the Receive Register Not Empty flag */
	if (UART1_FLAG == UART1_FLAG_RXNE) {
		UART1->SR = (uint8_t) ~(UART1_SR_RXNE);
	}
		/* Clear the LIN Break Detection flag */
	else {
		UART1->CR4 &= (uint8_t) ~(UART1_CR4_LBDF);
	}
}

/**
  * @brief  Checks whether the specified UART1 interrupt has occurred or not.
  * @param  UART1_IT: Specifies the UART1 interrupt pending bit to check.
  *         This parameter can be one of the following values:
  *         - UART1_IT_LBDF:  LIN Break detection interrupt
  *         - UART1_IT_TXE:  Transmit Data Register empty interrupt
  *         - UART1_IT_TC:   Transmission complete interrupt
  *         - UART1_IT_RXNE: Receive Data register not empty interrupt
  *         - UART1_IT_IDLE: Idle line detection interrupt
  *         - UART1_IT_OR:  OverRun Error interrupt
  *         - UART1_IT_PE:   Parity Error interrupt
  * @retval The new state of UART1_IT (SET or RESET).
  */
inline ITStatus UART1_GetITStatus(UART1_IT_TypeDef UART1_IT)
{
	ITStatus pendingbitstatus = RESET;
	uint8_t itpos = 0;
	uint8_t itmask1 = 0;
	uint8_t itmask2 = 0;
	uint8_t enablestatus = 0;

	/* Check parameters */
	assert_param(IS_UART1_GET_IT_OK(UART1_IT));

	/* Get the UART1 IT index */
	itpos = (uint8_t) ((uint8_t) 1 << (uint8_t) ((uint8_t) UART1_IT & (uint8_t) 0x0F));
	/* Get the UART1 IT index */
	itmask1 = (uint8_t) ((uint8_t) UART1_IT >> (uint8_t) 4);
	/* Set the IT mask*/
	itmask2 = (uint8_t) ((uint8_t) 1 << itmask1);


	/* Check the status of the specified UART1 pending bit*/
	if (UART1_IT == UART1_IT_PE) {
		/* Get the UART1_IT enable bit status*/
		enablestatus = (uint8_t) ((uint8_t) UART1->CR1 & itmask2);
		/* Check the status of the specified UART1 interrupt*/

		if (((UART1->SR & itpos) != (uint8_t) 0x00) && enablestatus) {
			/* Interrupt occurred*/
			pendingbitstatus = SET;
		} else {
			/* Interrupt not occurred*/
			pendingbitstatus = RESET;
		}
	} else if (UART1_IT == UART1_IT_LBDF) {
		/* Get the UART1_IT enable bit status*/
		enablestatus = (uint8_t) ((uint8_t) UART1->CR4 & itmask2);
		/* Check the status of the specified UART1 interrupt*/
		if (((UART1->CR4 & itpos) != (uint8_t) 0x00) && enablestatus) {
			/* Interrupt occurred*/
			pendingbitstatus = SET;
		} else {
			/* Interrupt not occurred*/
			pendingbitstatus = RESET;
		}
	} else {
		/* Get the UART1_IT enable bit status*/
		enablestatus = (uint8_t) ((uint8_t) UART1->CR2 & itmask2);
		/* Check the status of the specified UART1 interrupt*/
		if (((UART1->SR & itpos) != (uint8_t) 0x00) && enablestatus) {
			/* Interrupt occurred*/
			pendingbitstatus = SET;
		} else {
			/* Interrupt not occurred*/
			pendingbitstatus = RESET;
		}
	}

	/* Return the UART1_IT status*/
	return pendingbitstatus;
}

/**
  * @brief  Clears the UART1 pending flags.
  * @param  UART1_IT specifies the pending bit to clear
  *         This parameter can be one of the following values:
  *         - UART1_IT_LBDF:  LIN Break detection interrupt
  *         - UART1_IT_RXNE: Receive Data register not empty interrupt.
  * @note
  *         - PE (Parity error), FE (Framing error), NE (Noise error),
  *           OR (OverRun error) and IDLE (Idle line detected) pending bits are
  *           cleared by software sequence: a read operation to UART1_SR register
  *           (UART1_GetITStatus()) followed by a read operation to UART1_DR register
  *           (UART1_ReceiveData8() or UART1_ReceiveData9()).
  *
  *         - RXNE pending bit can be also cleared by a read to the UART1_DR register
  *           (UART1_ReceiveData8() or UART1_ReceiveData9()).
  *
  *         - TC (Transmit complete) pending bit can be cleared by software
  *           sequence: a read operation to UART1_SR register (UART1_GetITStatus())
  *           followed by a write operation to UART1_DR register (UART1_SendData8()
  *           or UART1_SendData9()).
  *
  *         - TXE pending bit is cleared only by a write to the UART1_DR register
  *           (UART1_SendData8() or UART1_SendData9()).
  * @retval None
  */
inline void UART1_ClearITPendingBit(UART1_IT_TypeDef UART1_IT)
{
	assert_param(IS_UART1_CLEAR_IT_OK(UART1_IT));

	/* Clear the Receive Register Not Empty pending bit */
	if (UART1_IT == UART1_IT_RXNE) {
		UART1->SR = (uint8_t) ~(UART1_SR_RXNE);
	}
		/* Clear the LIN Break Detection pending bit */
	else {
		UART1->CR4 &= (uint8_t) ~(UART1_CR4_LBDF);
	}
}

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* __STM8S_UART1_H */
