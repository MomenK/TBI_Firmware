/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC2650_LAUNCHXL.h
 *
 *  @brief      CC2650 LaunchPad Board Specific header file.
 *
 *  NB! This is the board file for CC2650 LaunchPad PCB version 1.1
 *
 *  ============================================================================
 */
#ifndef __BTE_H__
#define __BTE_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];



/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

/* Discrete outputs */
#define Board_GLED                  IOID_14
#define Board_LED_ON                1
#define Board_LED_OFF               0

/* SPI Board */
#define Board_SPI0_MISO             IOID_5          /* RF1.20 */
#define Board_SPI0_MOSI             IOID_4          /* RF1.18 */
#define Board_SPI0_CLK              IOID_3         /* RF1.16 */
#define Board_SPI0_CSN              PIN_UNASSIGNED

/* I2C */
#define Board_I2C0_SCL0             IOID_0
#define Board_I2C0_SDA0             IOID_1


/*ACD */
#define Board_DIO7_ANALOG          IOID_7
#define Board_DIO8_ANALOG          IOID_8
#define Board_DIO9_ANALOG          IOID_9
#define Board_DIO10_ANALOG          IOID_10
#define Board_DIO11_ANALOG          IOID_11
#define Board_DIO12_ANALOG          IOID_12
#define Board_DIO13_ANALOG          IOID_13


/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             BTE_GPTIMER0A
#define Board_GPTIMER0B             BTE_GPTIMER0B
#define Board_GPTIMER1A             BTE_GPTIMER1A
#define Board_GPTIMER1B             BTE_GPTIMER1B
#define Board_GPTIMER2A             BTE_GPTIMER2A
#define Board_GPTIMER2B             BTE_GPTIMER2B
#define Board_GPTIMER3A             BTE_GPTIMER3A
#define Board_GPTIMER3B             BTE_GPTIMER3B



/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   BTE_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  BTE_SPI0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum BTE_I2CName {
    BTE_I2C0 = 0,

    BTE_I2CCOUNT
} BTE_I2CName;

/*!
 *  @def    CC2650_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum BTE_SPIName {
    BTE_SPI0 = 0,

    BTE_SPICOUNT
} BTE_SPIName;


/*!
 *  @def    CC2650_LAUNCHXL_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum BTE_UdmaName {
    BTE_UDMA0 = 0,

    BTE_UDMACOUNT
} BTE_UdmaName;


/*!
 *  @def    CC2650_LAUNCHXL_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum BTE_TRNGName {
    BTE_TRNG0 = 0,
    BTE_TRNGCOUNT
} BTE_TRNGName;

/*!
 *  @def    CC2650_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum BTE_ADCBufName {
    BTE_ADCBuf0 = 0,
    BTE_ADCBufCOUNT
} BTE_ADCBufName;



/*!
 *  @def   BTE_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum BTE_GPTimerName
{
    BTE_GPTIMER0A = 0,
    BTE_GPTIMER0B,
    BTE_GPTIMER1A,
    BTE_GPTIMER1B,
    BTE_GPTIMER2A,
    BTE_GPTIMER2B,
    BTE_GPTIMER3A,
    BTE_GPTIMER3B,
    BTE_GPTIMERPARTSCOUNT
} BTE_GPTimerName;

/*!
 *  @def   BTE_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum BTE_GPTimers
{
    BTE_GPTIMER0 = 0,
    BTE_GPTIMER1,
    BTE_GPTIMER2,
    BTE_GPTIMER3,
    BTE_GPTIMERCOUNT
} BTE_GPTimers;

/*!
 *  @def    BTE_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum BTE_ADCName {
    BTE_ADC0 = 0,
    BTE_ADC1,
    BTE_ADC2,
    BTE_ADC3,
    BTE_ADC4,
    BTE_ADC5,
    BTE_ADC6,
    BTE_ADCDCOUPL,
    BTE_ADCVSS,
    BTE_ADCVDDS,
    BTE_ADCCOUNT
}  BTE_ADCName;


#ifdef __cplusplus
}
#endif

#endif /* __BTE_H__ */
