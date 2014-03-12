/*  Copyright (c) 2014, Alan Barr
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __CHIBIOS_TSL2561__
#define __CHIBIOS_TSL2561__

#include "ch.h"
#include "hal.h"

/* TSL2641 Addresses */
#define TSL2561_ADDR_GND    0x29
#define TSL2561_ADDR_FLOAT  0x39
#define TSL2561_ADDR_VDD    0x49

/* Registers */
#define CONTROL             0x0
#define TIMING              0x1
#define THRESHLOWLOW        0x2
#define THRESHLOWHIGH       0x3
#define THRESHHIGHLOW       0x4
#define THRESHHIGHHIGH      0x5
#define INTERRUPT           0x6
//#define CRC                 0x8
#define ID                  0xA
#define DATA0LOW            0xC
#define DATA0HIGH           0xD
#define DATA1LOW            0xE
#define DATA1HIGH           0xF

/* Command Register */
#define CMD                 (1<<7)
#define CLEAR               (1<<6)
#define WORD                (1<<5)
#define BLOCK               (1<<4)
#define ADDRESS_MASK        0xF

/* Control Register */
#define POWER_MASK          (0x3)

/* Timing Register */
#define GAIN                (1<<4)
#define INTEG_MASK          0x3

/* Interrupt Control Register */
#define INTR_MASK           (0x3<<5)
#define PERSIST_MASK        0xF

/* ID Register */
#define PARTNO_MASK         (0xF<<7)
#define REVNO_MASK          0xF


typedef enum {
    POWER_DOWN = 0,
    POWER_UP   = 3
} powerMode;


typedef enum {
    INT_TIME_13_7_MS = 0x0,
    INT_TIME_101_MS  = 0x1,
    INT_TIME_402_MS  = 0x2,
    INT_TIME_MANUAL  = 0x3
} tslIntegrationTime;


typedef enum {
    GAIN_LOW_1X    = 0,
    GAIN_HIGH_16X  = 1,
} tslGain;


typedef enum {
    TYPE_TMB,
    TYPE_CS
} tslType;


msg_t tslReadRegister(I2CDriver * driver, uint8_t device,
                      uint8_t reg, uint8_t * byte);

msg_t tslReadRegisters(I2CDriver * driver, uint8_t device,
                       uint8_t reg, uint8_t * byte, uint8_t number);

msg_t tslWriteRegister(I2CDriver * driver, uint8_t device,
                       uint8_t reg, uint8_t byte);
    
msg_t tslSetIntegrationTime(I2CDriver * driver, uint8_t device,
                            tslIntegrationTime time);

msg_t tslSetGain(I2CDriver * driver, uint8_t device, tslGain gain);

msg_t tslSetPowerMode(I2CDriver * driver, uint8_t device,
                      powerMode mode);

msg_t tslReadChannels(I2CDriver * driver, uint8_t device, uint16_t * channel0,
                      uint16_t * channel1);

uint16_t calculateLux(tslGain iGain, tslIntegrationTime tInt,
                          uint16_t ch0, uint16_t ch1, tslType Type);

msg_t tslReadLuxConvertSleep(I2CDriver * driver, uint8_t device, uint16_t * lux);

#endif /*__CHIBIOS_TSL2561__*/

