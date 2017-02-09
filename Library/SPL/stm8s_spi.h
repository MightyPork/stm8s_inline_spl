/**
  ******************************************************************************
  * @file    stm8s_spi.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains all functions prototype and macros for the SPI peripheral.
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
#ifndef __STM8S_SPI_H
#define __STM8S_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */

/** @addtogroup SPI_Exported_Types
  * @{
  */

/**
  * @brief  SPI data direction mode
  * Warning: element values correspond to BDM, BDOE, RXONLY bits position
  */
typedef enum {
	SPI_DATADIRECTION_2LINES_FULLDUPLEX = (uint8_t) 0x00, /*!< 2-line uni-directional data mode enable */
	SPI_DATADIRECTION_2LINES_RXONLY = (uint8_t) 0x04, /*!< Receiver only in 2 line uni-directional data mode */
	SPI_DATADIRECTION_1LINE_RX = (uint8_t) 0x80, /*!< Receiver only in 1 line bi-directional data mode */
	SPI_DATADIRECTION_1LINE_TX = (uint8_t) 0xC0  /*!< Transmit only in 1 line bi-directional data mode */
} SPI_DataDirection_TypeDef;

/**
  * @brief  SPI Slave Select management
  * Warning: element values correspond to LSBFIRST bit position
  */
typedef enum {
	SPI_NSS_SOFT = (uint8_t) 0x02, /*!< Software slave management disabled */
	SPI_NSS_HARD = (uint8_t) 0x00  /*!< Software slave management enabled */
} SPI_NSS_TypeDef;


/**
  * @brief  SPI direction transmit/receive
  */

typedef enum {
	SPI_DIRECTION_RX = (uint8_t) 0x00, /*!< Selects Rx receive direction in bi-directional mode */
	SPI_DIRECTION_TX = (uint8_t) 0x01  /*!< Selects Tx transmission direction in bi-directional mode */
} SPI_Direction_TypeDef;

/**
  * @brief  SPI master/slave mode
  * Warning: element values correspond to MSTR bit position
  */
typedef enum {
	SPI_MODE_MASTER = (uint8_t) 0x04, /*!< SPI Master configuration */
	SPI_MODE_SLAVE = (uint8_t) 0x00  /*!< SPI Slave configuration */
} SPI_Mode_TypeDef;

/**
  * @brief  SPI BaudRate Prescaler
  * Warning: element values correspond to BR bits position
  */
typedef enum {
	SPI_BAUDRATEPRESCALER_2 = (uint8_t) 0x00, /*!< SPI frequency = frequency(CPU)/2 */
	SPI_BAUDRATEPRESCALER_4 = (uint8_t) 0x08, /*!< SPI frequency = frequency(CPU)/4 */
	SPI_BAUDRATEPRESCALER_8 = (uint8_t) 0x10, /*!< SPI frequency = frequency(CPU)/8 */
	SPI_BAUDRATEPRESCALER_16 = (uint8_t) 0x18, /*!< SPI frequency = frequency(CPU)/16 */
	SPI_BAUDRATEPRESCALER_32 = (uint8_t) 0x20, /*!< SPI frequency = frequency(CPU)/32 */
	SPI_BAUDRATEPRESCALER_64 = (uint8_t) 0x28, /*!< SPI frequency = frequency(CPU)/64 */
	SPI_BAUDRATEPRESCALER_128 = (uint8_t) 0x30, /*!< SPI frequency = frequency(CPU)/128 */
	SPI_BAUDRATEPRESCALER_256 = (uint8_t) 0x38  /*!< SPI frequency = frequency(CPU)/256 */
} SPI_BaudRatePrescaler_TypeDef;

/**
  * @brief  SPI Clock Polarity
  * Warning: element values correspond to CPOL bit position
  */
typedef enum {
	SPI_CLOCKPOLARITY_LOW = (uint8_t) 0x00, /*!< Clock to 0 when idle */
	SPI_CLOCKPOLARITY_HIGH = (uint8_t) 0x02  /*!< Clock to 1 when idle */
} SPI_ClockPolarity_TypeDef;

/**
  * @brief  SPI Clock Phase
  * Warning: element values correspond to CPHA bit position
  */
typedef enum {
	SPI_CLOCKPHASE_1EDGE = (uint8_t) 0x00, /*!< The first clock transition is the first data capture edge */
	SPI_CLOCKPHASE_2EDGE = (uint8_t) 0x01  /*!< The second clock transition is the first data capture edge */
} SPI_ClockPhase_TypeDef;

/**
  * @brief  SPI Frame Format: MSB or LSB transmitted first
  * Warning: element values correspond to LSBFIRST bit position
  */
typedef enum {
	SPI_FIRSTBIT_MSB = (uint8_t) 0x00, /*!< MSB bit will be transmitted first */
	SPI_FIRSTBIT_LSB = (uint8_t) 0x80  /*!< LSB bit will be transmitted first */
} SPI_FirstBit_TypeDef;

/**
  * @brief  SPI CRC Transmit/Receive
  */
typedef enum {
	SPI_CRC_RX = (uint8_t) 0x00, /*!< Select Tx CRC register */
	SPI_CRC_TX = (uint8_t) 0x01  /*!< Select Rx CRC register */
} SPI_CRC_TypeDef;

/**
  * @brief  SPI flags definition - Warning : FLAG value = mapping position register
  */
typedef enum {
	SPI_FLAG_BSY = (uint8_t) 0x80, /*!< Busy flag */
	SPI_FLAG_OVR = (uint8_t) 0x40, /*!< Overrun flag */
	SPI_FLAG_MODF = (uint8_t) 0x20, /*!< Mode fault */
	SPI_FLAG_CRCERR = (uint8_t) 0x10, /*!< CRC error flag */
	SPI_FLAG_WKUP = (uint8_t) 0x08, /*!< Wake-up flag */
	SPI_FLAG_TXE = (uint8_t) 0x02, /*!< Transmit buffer empty */
	SPI_FLAG_RXNE = (uint8_t) 0x01  /*!< Receive buffer empty */
} SPI_Flag_TypeDef;

/**
  * @brief  SPI_IT possible values
  * Elements values convention: 0xYX
  *   X: Position of the corresponding Interrupt
  *   Y: ITPENDINGBIT position
  */
typedef enum {
	SPI_IT_WKUP = (uint8_t) 0x34,  /*!< Wake-up interrupt*/
	SPI_IT_OVR = (uint8_t) 0x65,  /*!< Overrun interrupt*/
	SPI_IT_MODF = (uint8_t) 0x55,  /*!< Mode fault interrupt*/
	SPI_IT_CRCERR = (uint8_t) 0x45,  /*!< CRC error interrupt*/
	SPI_IT_TXE = (uint8_t) 0x17,  /*!< Transmit buffer empty interrupt*/
	SPI_IT_RXNE = (uint8_t) 0x06,   /*!< Receive buffer not empty interrupt*/
	SPI_IT_ERR = (uint8_t) 0x05   /*!< Error interrupt*/
} SPI_IT_TypeDef;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/

/** @addtogroup SPI_Private_Macros
  * @brief  Macros used by the assert_param function to check the different functions parameters.
  * @{
  */

/**
  * @brief  Macro used by the assert_param function in order to check the data direction mode values
  */
#define IS_SPI_DATA_DIRECTION_OK(MODE) (((MODE) == SPI_DATADIRECTION_2LINES_FULLDUPLEX) || \
                                        ((MODE) == SPI_DATADIRECTION_2LINES_RXONLY) || \
                                        ((MODE) == SPI_DATADIRECTION_1LINE_RX) || \
                                        ((MODE) == SPI_DATADIRECTION_1LINE_TX))

/**
  * @brief  Macro used by the assert_param function in order to check the mode 
  *         half duplex data direction values
  */
#define IS_SPI_DIRECTION_OK(DIRECTION) (((DIRECTION) == SPI_DIRECTION_RX) || \
                                        ((DIRECTION) == SPI_DIRECTION_TX))

/**
  * @brief  Macro used by the assert_param function in order to check the NSS 
  *         management values
  */
#define IS_SPI_SLAVEMANAGEMENT_OK(NSS) (((NSS) == SPI_NSS_SOFT) || \
                                        ((NSS) == SPI_NSS_HARD))


/**
  * @brief  Macro used by the assert_param function in order to check the different
  *         sensitivity values for the CRC polynomial
  */
#define IS_SPI_CRC_POLYNOMIAL_OK(POLYNOMIAL) ((POLYNOMIAL) > (uint8_t)0x00)

/**
  * @brief  Macro used by the assert_param function in order to check the SPI Mode values
  */
#define IS_SPI_MODE_OK(MODE) (((MODE) == SPI_MODE_MASTER) || \
                              ((MODE) == SPI_MODE_SLAVE))

/**
  * @brief  Macro used by the assert_param function in order to check the baudrate values
  */
#define IS_SPI_BAUDRATE_PRESCALER_OK(PRESCALER) (((PRESCALER) == SPI_BAUDRATEPRESCALER_2) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_4) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_8) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_16) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_32) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_64) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_128) || \
    ((PRESCALER) == SPI_BAUDRATEPRESCALER_256))

/**
  * @brief  Macro used by the assert_param function in order to check the polarity values
  */
#define IS_SPI_POLARITY_OK(CLKPOL) (((CLKPOL) == SPI_CLOCKPOLARITY_LOW) || \
                                    ((CLKPOL) == SPI_CLOCKPOLARITY_HIGH))

/**
  * @brief  Macro used by the assert_param function in order to check the phase values
  */
#define IS_SPI_PHASE_OK(CLKPHA) (((CLKPHA) == SPI_CLOCKPHASE_1EDGE) || \
                                 ((CLKPHA) == SPI_CLOCKPHASE_2EDGE))

/**
  * @brief  Macro used by the assert_param function in order to check the first 
  *         bit to be transmited values
  */
#define IS_SPI_FIRSTBIT_OK(BIT) (((BIT) == SPI_FIRSTBIT_MSB) || \
                                 ((BIT) == SPI_FIRSTBIT_LSB))

/**
  * @brief  Macro used by the assert_param function in order to check the CRC 
  *         Transmit/Receive
  */
#define IS_SPI_CRC_OK(CRC) (((CRC) == SPI_CRC_TX) || \
                            ((CRC) == SPI_CRC_RX))

/**
  * @brief  Macro used by the assert_param function in order to check the 
  *         different flags values
  */
#define IS_SPI_FLAGS_OK(FLAG) (((FLAG) == SPI_FLAG_OVR) || \
                               ((FLAG) == SPI_FLAG_MODF) || \
                               ((FLAG) == SPI_FLAG_CRCERR) || \
                               ((FLAG) == SPI_FLAG_WKUP) || \
                               ((FLAG) == SPI_FLAG_TXE) || \
                               ((FLAG) == SPI_FLAG_RXNE) || \
                               ((FLAG) == SPI_FLAG_BSY))

/**
  * @brief  Macro used by the assert_param function in order to check the 
  *         different sensitivity values for the flag that can be cleared 
  *         by writing 0
  */
#define IS_SPI_CLEAR_FLAGS_OK(FLAG) (((FLAG) == SPI_FLAG_CRCERR) || \
                                     ((FLAG) == SPI_FLAG_WKUP))

/**
  * @brief  Macro used by the assert_param function in order to check the 
  *        different sensitivity values for the Interrupts
  */
#define IS_SPI_CONFIG_IT_OK(Interrupt) (((Interrupt) == SPI_IT_TXE)  || \
                                        ((Interrupt) == SPI_IT_RXNE)  || \
                                        ((Interrupt) == SPI_IT_ERR) || \
                                        ((Interrupt) == SPI_IT_WKUP))

/**
  * @brief  Macro used by the assert_param function in order to check the 
  *         different sensitivity values for the pending bit
  */
#define IS_SPI_GET_IT_OK(ITPendingBit) (((ITPendingBit) == SPI_IT_OVR)  || \
                                        ((ITPendingBit) == SPI_IT_MODF) || \
                                        ((ITPendingBit) == SPI_IT_CRCERR) || \
                                        ((ITPendingBit) == SPI_IT_WKUP) || \
                                        ((ITPendingBit) == SPI_IT_TXE)  || \
                                        ((ITPendingBit) == SPI_IT_RXNE))

/**
  * @brief  Macro used by the assert_param function in order to check the 
  *         different sensitivity values for the pending bit that can be cleared
  *         by writing 0
  */
#define IS_SPI_CLEAR_IT_OK(ITPendingBit) (((ITPendingBit) == SPI_IT_CRCERR) || \
    ((ITPendingBit) == SPI_IT_WKUP))

/**
  * @}
  */

#if 0
/** @addtogroup SPI_Exported_Functions
  * @{
  */
void SPI_DeInit(void);

void SPI_Init(SPI_FirstBit_TypeDef FirstBit,
			  SPI_BaudRatePrescaler_TypeDef BaudRatePrescaler,
			  SPI_Mode_TypeDef Mode, SPI_ClockPolarity_TypeDef ClockPolarity,
			  SPI_ClockPhase_TypeDef ClockPhase,
			  SPI_DataDirection_TypeDef Data_Direction,
			  SPI_NSS_TypeDef Slave_Management, uint8_t CRCPolynomial);

void SPI_Cmd(FunctionalState NewState);

void SPI_ITConfig(SPI_IT_TypeDef SPI_IT, FunctionalState NewState);

void SPI_SendData(uint8_t Data);

uint8_t SPI_ReceiveData(void);

void SPI_NSSInternalSoftwareCmd(FunctionalState NewState);

void SPI_TransmitCRC(void);

void SPI_CalculateCRCCmd(FunctionalState NewState);

uint8_t SPI_GetCRC(SPI_CRC_TypeDef SPI_CRC);

void SPI_ResetCRC(void);

uint8_t SPI_GetCRCPolynomial(void);

void SPI_BiDirectionalLineConfig(SPI_Direction_TypeDef SPI_Direction);

FlagStatus SPI_GetFlagStatus(SPI_Flag_TypeDef SPI_FLAG);

void SPI_ClearFlag(SPI_Flag_TypeDef SPI_FLAG);

ITStatus SPI_GetITStatus(SPI_IT_TypeDef SPI_IT);

void SPI_ClearITPendingBit(SPI_IT_TypeDef SPI_IT);
#endif
/**
  * @}
  */


/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup SPI_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the SPI peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
inline void SPI_DeInit(void)
{
	SPI->CR1 = SPI_CR1_RESET_VALUE;
	SPI->CR2 = SPI_CR2_RESET_VALUE;
	SPI->ICR = SPI_ICR_RESET_VALUE;
	SPI->SR = SPI_SR_RESET_VALUE;
	SPI->CRCPR = SPI_CRCPR_RESET_VALUE;
}

/**
  * @brief  Initializes the SPI according to the specified parameters.
  * @param  FirstBit : This parameter can be any of the
  *         @ref SPI_FirstBit_TypeDef enumeration.
  * @param  BaudRatePrescaler : This parameter can be any of the
  *         @ref SPI_BaudRatePrescaler_TypeDef enumeration.
  * @param  Mode : This parameter can be any of the
  *         @ref SPI_Mode_TypeDef enumeration.
  * @param  ClockPolarity : This parameter can be any of the
  *         @ref SPI_ClockPolarity_TypeDef enumeration.
  * @param  ClockPhase : This parameter can be any of the
  *         @ref SPI_ClockPhase_TypeDef enumeration.
  * @param  Data_Direction : This parameter can be any of the
  *         @ref SPI_DataDirection_TypeDef enumeration.
  * @param  Slave_Management : This parameter can be any of the
  *         @ref SPI_NSS_TypeDef enumeration.
  * @param  CRCPolynomial : Configures the CRC polynomial.
  * @retval None
  */
inline void
SPI_Init(SPI_FirstBit_TypeDef FirstBit, SPI_BaudRatePrescaler_TypeDef BaudRatePrescaler, SPI_Mode_TypeDef Mode,
		 SPI_ClockPolarity_TypeDef ClockPolarity, SPI_ClockPhase_TypeDef ClockPhase,
		 SPI_DataDirection_TypeDef Data_Direction, SPI_NSS_TypeDef Slave_Management, uint8_t CRCPolynomial)
{
	/* Check structure elements */
	assert_param(IS_SPI_FIRSTBIT_OK(FirstBit));
	assert_param(IS_SPI_BAUDRATE_PRESCALER_OK(BaudRatePrescaler));
	assert_param(IS_SPI_MODE_OK(Mode));
	assert_param(IS_SPI_POLARITY_OK(ClockPolarity));
	assert_param(IS_SPI_PHASE_OK(ClockPhase));
	assert_param(IS_SPI_DATA_DIRECTION_OK(Data_Direction));
	assert_param(IS_SPI_SLAVEMANAGEMENT_OK(Slave_Management));
	assert_param(IS_SPI_CRC_POLYNOMIAL_OK(CRCPolynomial));

	/* Frame Format, BaudRate, Clock Polarity and Phase configuration */
	SPI->CR1 = (uint8_t) ((uint8_t) ((uint8_t) FirstBit | BaudRatePrescaler) |
						  (uint8_t) ((uint8_t) ClockPolarity | ClockPhase));

	/* Data direction configuration: BDM, BDOE and RXONLY bits */
	SPI->CR2 = (uint8_t) ((uint8_t) (Data_Direction) | (uint8_t) (Slave_Management));

	if (Mode == SPI_MODE_MASTER) {
		SPI->CR2 |= (uint8_t) SPI_CR2_SSI;
	} else {
		SPI->CR2 &= (uint8_t) ~(SPI_CR2_SSI);
	}

	/* Master/Slave mode configuration */
	SPI->CR1 |= (uint8_t) (Mode);

	/* CRC configuration */
	SPI->CRCPR = (uint8_t) CRCPolynomial;
}

/**
  * @brief  Enables or disables the SPI peripheral.
  * @param  NewState New state of the SPI peripheral.
  *         This parameter can be: ENABLE or DISABLE
  * @retval None
  */
inline void SPI_Cmd(FunctionalState NewState)
{
	/* Check function parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		SPI->CR1 |= SPI_CR1_SPE; /* Enable the SPI peripheral*/
	} else {
		SPI->CR1 &= (uint8_t) (~SPI_CR1_SPE); /* Disable the SPI peripheral*/
	}
}

/**
  * @brief  Enables or disables the specified interrupts.
  * @param  SPI_IT Specifies the SPI interrupts sources to be enabled or disabled.
  * @param  NewState: The new state of the specified SPI interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void SPI_ITConfig(SPI_IT_TypeDef SPI_IT, FunctionalState NewState)
{
	uint8_t itpos = 0;
	/* Check function parameters */
	assert_param(IS_SPI_CONFIG_IT_OK(SPI_IT));
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	/* Get the SPI IT index */
	itpos = (uint8_t) ((uint8_t) 1 << (uint8_t) ((uint8_t) SPI_IT & (uint8_t) 0x0F));

	if (NewState != DISABLE) {
		SPI->ICR |= itpos; /* Enable interrupt*/
	} else {
		SPI->ICR &= (uint8_t) (~itpos); /* Disable interrupt*/
	}
}

/**
  * @brief  Transmits a Data through the SPI peripheral.
  * @param  Data : Byte to be transmitted.
  * @retval None
  */
inline void SPI_SendData(uint8_t Data)
{
	SPI->DR = Data; /* Write in the DR register the data to be sent*/
}

/**
  * @brief  Returns the most recent received data by the SPI peripheral.
  * @param  None
  * @retval The value of the received data.
  */
inline uint8_t SPI_ReceiveData(void)
{
	return ((uint8_t) SPI->DR); /* Return the data in the DR register*/
}

/**
  * @brief  Configures internally by software the NSS pin.
  * @param  NewState Indicates the new state of the SPI Software slave management.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void SPI_NSSInternalSoftwareCmd(FunctionalState NewState)
{
	/* Check function parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		SPI->CR2 |= SPI_CR2_SSI; /* Set NSS pin internally by software*/
	} else {
		SPI->CR2 &= (uint8_t) (~SPI_CR2_SSI); /* Reset NSS pin internally by software*/
	}
}

/**
  * @brief  Enables the transmit of the CRC value.
  * @param  None
  * @retval None
  */
inline void SPI_TransmitCRC(void)
{
	SPI->CR2 |= SPI_CR2_CRCNEXT; /* Enable the CRC transmission*/
}

/**
  * @brief  Enables or disables the CRC value calculation of the transferred bytes.
  * @param  NewState Indicates the new state of the SPI CRC value calculation.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
inline void SPI_CalculateCRCCmd(FunctionalState NewState)
{
	/* Check function parameters */
	assert_param(IS_FUNCTIONALSTATE_OK(NewState));

	if (NewState != DISABLE) {
		SPI->CR2 |= SPI_CR2_CRCEN; /* Enable the CRC calculation*/
	} else {
		SPI->CR2 &= (uint8_t) (~SPI_CR2_CRCEN); /* Disable the CRC calculation*/
	}
}

/**
  * @brief  Returns the transmit or the receive CRC register value.
  * @param  SPI_CRC Specifies the CRC register to be read.
  * @retval The selected CRC register value.
  */
inline uint8_t SPI_GetCRC(SPI_CRC_TypeDef SPI_CRC)
{
	uint8_t crcreg = 0;

	/* Check function parameters */
	assert_param(IS_SPI_CRC_OK(SPI_CRC));

	if (SPI_CRC != SPI_CRC_RX) {
		crcreg = SPI->TXCRCR;  /* Get the Tx CRC register*/
	} else {
		crcreg = SPI->RXCRCR; /* Get the Rx CRC register*/
	}

	/* Return the selected CRC register status*/
	return crcreg;
}

/**
  * @brief  Reset the Rx CRCR and Tx CRCR registers.
  * @param  None
  * @retval None
  */
inline void SPI_ResetCRC(void)
{
	/* Rx CRCR & Tx CRCR registers are reset when CRCEN (hardware calculation)
	bit in SPI_CR2 is written to 1 (enable) */
	SPI_CalculateCRCCmd(ENABLE);

	/* Previous function disable the SPI */
	SPI_Cmd(ENABLE);
}

/**
  * @brief  Returns the CRC Polynomial register value.
  * @param  None
  * @retval The CRC Polynomial register value.
  */
inline uint8_t SPI_GetCRCPolynomial(void)
{
	return SPI->CRCPR; /* Return the CRC polynomial register */
}

/**
  * @brief  Selects the data transfer direction in bi-directional mode.
  * @param  SPI_Direction Specifies the data transfer direction in bi-directional mode.
  * @retval None
  */
inline void SPI_BiDirectionalLineConfig(SPI_Direction_TypeDef SPI_Direction)
{
	/* Check function parameters */
	assert_param(IS_SPI_DIRECTION_OK(SPI_Direction));

	if (SPI_Direction != SPI_DIRECTION_RX) {
		SPI->CR2 |= SPI_CR2_BDOE; /* Set the Tx only mode*/
	} else {
		SPI->CR2 &= (uint8_t) (~SPI_CR2_BDOE); /* Set the Rx only mode*/
	}
}

/**
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  SPI_FLAG : Specifies the flag to check.
  *         This parameter can be any of the @ref SPI_FLAG_TypeDef enumeration.
  * @retval FlagStatus : Indicates the state of SPI_FLAG.
  *         This parameter can be any of the @ref FlagStatus enumeration.
  */
inline FlagStatus SPI_GetFlagStatus(SPI_Flag_TypeDef SPI_FLAG)
{
	FlagStatus status = RESET;
	/* Check parameters */
	assert_param(IS_SPI_FLAGS_OK(SPI_FLAG));

	/* Check the status of the specified SPI flag */
	if ((SPI->SR & (uint8_t) SPI_FLAG) != (uint8_t) RESET) {
		status = SET; /* SPI_FLAG is set */
	} else {
		status = RESET; /* SPI_FLAG is reset*/
	}

	/* Return the SPI_FLAG status */
	return status;
}

/**
  * @brief  Clears the SPI flags.
  * @param  SPI_FLAG : Specifies the flag to clear.
  *         This parameter can be one of the following values:
  *         - SPI_FLAG_CRCERR
  *         - SPI_FLAG_WKUP
  * @note   - OVR (OverRun Error) interrupt pending bit is cleared by software
  *         sequence:
  *         a read operation to SPI_DR register (SPI_ReceiveData()) followed by
  *         a read operation to SPI_SR register (SPI_GetFlagStatus()).
  *         - MODF (Mode Fault) interrupt pending bit is cleared by software sequence:
  *         a read/write operation to SPI_SR register (SPI_GetFlagStatus()) followed by
  *         a write operation to SPI_CR1 register (SPI_Cmd() to enable the SPI).
  * @retval None
  */
inline void SPI_ClearFlag(SPI_Flag_TypeDef SPI_FLAG)
{
	assert_param(IS_SPI_CLEAR_FLAGS_OK(SPI_FLAG));
	/* Clear the flag bit */
	SPI->SR = (uint8_t) (~SPI_FLAG);
}

/**
  * @brief  Checks whether the specified interrupt has occurred or not.
  * @param  SPI_IT: Specifies the SPI interrupt pending bit to check.
  *         This parameter can be one of the following values:
  *         - SPI_IT_CRCERR
  *         - SPI_IT_WKUP
  *         - SPI_IT_OVR
  *         - SPI_IT_MODF
  *         - SPI_IT_RXNE
  *         - SPI_IT_TXE
  * @retval ITStatus : Indicates the state of the SPI_IT.
  *         This parameter can be any of the @ref ITStatus enumeration.
  */
inline ITStatus SPI_GetITStatus(SPI_IT_TypeDef SPI_IT)
{
	ITStatus pendingbitstatus = RESET;
	uint8_t itpos = 0;
	uint8_t itmask1 = 0;
	uint8_t itmask2 = 0;
	uint8_t enablestatus = 0;
	assert_param(IS_SPI_GET_IT_OK(SPI_IT));
	/* Get the SPI IT index */
	itpos = (uint8_t) ((uint8_t) 1 << ((uint8_t) SPI_IT & (uint8_t) 0x0F));

	/* Get the SPI IT mask */
	itmask1 = (uint8_t) ((uint8_t) SPI_IT >> (uint8_t) 4);
	/* Set the IT mask */
	itmask2 = (uint8_t) ((uint8_t) 1 << itmask1);
	/* Get the SPI_ITPENDINGBIT enable bit status */
	enablestatus = (uint8_t) ((uint8_t) SPI->SR & itmask2);
	/* Check the status of the specified SPI interrupt */
	if (((SPI->ICR & itpos) != RESET) && enablestatus) {
		/* SPI_ITPENDINGBIT is set */
		pendingbitstatus = SET;
	} else {
		/* SPI_ITPENDINGBIT is reset */
		pendingbitstatus = RESET;
	}
	/* Return the SPI_ITPENDINGBIT status */
	return pendingbitstatus;
}

/**
  * @brief  Clears the interrupt pending bits.
  * @param  SPI_IT: Specifies the interrupt pending bit to clear.
  *         This parameter can be one of the following values:
  *         - SPI_IT_CRCERR
  *         - SPI_IT_WKUP
  * @note   - OVR (OverRun Error) interrupt pending bit is cleared by software sequence:
  *         a read operation to SPI_DR register (SPI_ReceiveData()) followed by
  *         a read operation to SPI_SR register (SPI_GetITStatus()).
  *         - MODF (Mode Fault) interrupt pending bit is cleared by software sequence:
  *         a read/write operation to SPI_SR register (SPI_GetITStatus()) followed by
  *         a write operation to SPI_CR1 register (SPI_Cmd() to enable the SPI).
  * @retval None
  */
inline void SPI_ClearITPendingBit(SPI_IT_TypeDef SPI_IT)
{
	uint8_t itpos = 0;
	assert_param(IS_SPI_CLEAR_IT_OK(SPI_IT));

	/* Clear  SPI_IT_CRCERR or SPI_IT_WKUP interrupt pending bits */

	/* Get the SPI pending bit index */
	itpos = (uint8_t) ((uint8_t) 1 << (uint8_t) ((uint8_t) (SPI_IT & (uint8_t) 0xF0) >> 4));
	/* Clear the pending bit */
	SPI->SR = (uint8_t) (~itpos);

}

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#endif /* __STM8S_SPI_H */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
