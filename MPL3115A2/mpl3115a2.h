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

#ifndef __CHIBIOS_MPL3115A2__
#define __CHIBIOS_MPL3115A2__

/* Default Address (7 bit)*/
#define MPL3115A2_DEFAULT_ADDR  0x60 

/* Register Address Map */
#define STATUS              0x00
#define OUT_P_MSB           0x01
#define OUT_P_CSB           0x02
#define OUT_P_LSB           0x03
#define OUT_T_MSB           0x04
#define OUT_T_LSB           0x05
#define DR_STATUS           0x06
#define OUT_P_DELTA_MSB     0x07
#define OUT_P_DELTA_CSB     0x08
#define OUT_P_DELTA_LSB     0x09
#define OUT_T_DELTA_MSB     0x0A
#define OUT_T_DELTA_LSB     0x0B
#define WHO_AM_I            0x0C
#define F_STATUS            0x0D
#define F_DATA              0x0E
#define F_SETUP             0x0F
#define TIME_DLY            0x10
#define SYSMOD_REG          0x11 /* TODO rename */
#define INT_SOURCE          0x12
#define PT_DATA_CFG         0x13
#define BAR_IN_MSB          0x14
#define BAR_IN_LSB          0x15
#define P_TGT_MSB           0x16
#define P_TGT_LSB           0x17
#define T_TGT               0x18
#define P_WND_MSB           0x19
#define P_WND_LSB           0x1A
#define T_WND               0x1B
#define P_MIN_MSB           0x1C
#define P_MIN_CSB           0x1D
#define P_MIN_LSB           0x1E
#define T_MIN_MSB           0x1F
#define T_MIN_LSB           0x20
#define P_MAX_MSB           0x21
#define P_MAX_CSB           0x22
#define P_MAX_LSB           0x23
#define T_MAX_MSB           0x24
#define T_MAX_LSB           0x25
#define CTRL_REG1           0x26
#define CTRL_REG2           0x27
#define CTRL_REG3           0x28
#define CTRL_REG4           0x29
#define CTRL_REG5           0x2A
#define OFF_P               0x2B
#define OFF_T               0x2C
#define OFF_H               0x2D

/* DR_STATUS Register */
#define TDR                 (0x1<<1)
#define PDR                 (0x1<<2)
#define PTDR                (0x1<<3)
#define TOW                 (0x1<<5)
#define POW                 (0x1<<6)
#define PTOW                (0x1<<7)

/* F_STATUS Register */
#define F_CNTx_MASK         (0x3F)
#define F_WMRK_FLAG         (0x1<<6)
#define F_OVF               (0x1<<7)

/* F_SETUP Register */
#define F_WMRKx_MASK        (0x3F)
#define F_MODEx_MASK        (0x3<<6)
#define F_MODE_DISABLED     (0x0<<6)
#define F_MODE_CIRCULAR     (0x1<<6)
#define F_MODE_FULLSTOP     (0x2<<6)

/* SYSMOD Register */
#define SYSMOD              0x1

/* INT_SOURCE */
#define SRC_TCHG            0x1
#define SRC_PCHG            (0x1<<1)
#define SRC_TTH             (0x1<<2)
#define SRC_PTH             (0x1<<3)
#define SRC_TW              (0x1<<4)
#define SRC_PW              (0x1<<5)
#define SRC_FIFO            (0x1<<6)
#define SRC_DRDY            (0x1<<7)        

/* PT_DATA_CFG */
#define TDEFE               0x1
#define PDEFE               (0x1<<1)
#define DREM                (0x1<<2)

/* CTRL_REG1 Register */
#define SBYB                (0x1)
#define OST                 (0x1<<1)
#define RST                 (0x1<<2)
#define OSx_MASK            (0x7<<5)
#define RAW                 (0x1<<6)
#define ALT                 (0x1<<7)

/* CTRL_REG2 Register */
#define STx_MASK            (0xF)
#define ALARM_SEL           (0x1<<4)
#define LOAD_OUTPUT         (0x1<<5)

/* CTRL_REG3 Register */
#define PP_OD2              (0x1)
#define IPOL2               (0x1<<1)
#define PP_OD1              (0x1<<4)
#define IPOL1               (0x1<<5)

/* CTRL_REG4 Register */
#define INT_EN_TCHG         (0x1)
#define INT_EN_PCHG         (0x1<<1)
#define INT_EN_TTH          (0x1<<2)
#define INT_EN_PTH          (0x1<<3)
#define INT_EN_TW           (0x1<<4)
#define INT_EN_PW           (0x1<<5)
#define INT_EN_FIFO         (0x1<<6)
#define INT_EN_DRDY         (0x1<<7)

/* CTRL_REG5 Register */
#define INT_CFG_TCHG        (0x1)
#define INT_CFG_PCHG        (0x1<<1)
#define INT_CFG_TTH         (0x1<<2)
#define INT_CFG_PTH         (0x1<<3)
#define INT_CFG_TW          (0x1<<4)
#define INT_CFG_PW          (0x1<<5)
#define INT_CFG_FIFO        (0x1<<6)
#define INT_CFG_DRDY        (0x1<<7)

#include "stdint.h"
#include "ch.h"
#include "hal.h"

typedef enum
{
    STANDBY,
    ACTIVE
} mplPowerMode;

typedef enum
{
    BAROMETER,
    ALTIMETER
} mplMeasurementMode;


msg_t mplRegisterWrite(I2CDriver * driver, uint8_t devAddress,
                       uint8_t regAddress, uint8_t value);

msg_t mplRegisterRead(I2CDriver * driver, uint8_t devAddress,
                      uint8_t regAddress, uint8_t * readByte);

msg_t mplRegistersRead(I2CDriver * driver, uint8_t devAddress,
                       uint8_t regAddress, uint8_t * readBytes,
                       uint8_t readNumber);

msg_t mplSetPowerMode(I2CDriver * driver, uint8_t devAddress, mplPowerMode mode);

msg_t mplSetMeasurementMode(I2CDriver * driver, uint8_t devAddress,
                            mplMeasurementMode mode);

msg_t mplPerformOneShot(I2CDriver * driver, uint8_t devAddress);

msg_t mplReadTemperature(I2CDriver * driver, uint8_t devAddres,
                         float * temperature);

msg_t mplReadAltitude(I2CDriver * driver, uint8_t devAddres,
                      float * altitude);

msg_t mplReadPressure(I2CDriver * driver, uint8_t devAddres,
                      float * pressure);

msg_t mplOneShotReadBarometer(I2CDriver * driver, uint8_t devAddress,
                              float * pressure, float * temperature);

msg_t mplOneShotReadAll(I2CDriver * driver, uint8_t devAddress,
                        float * altitude, float * pressure,
                        float * temperature);


#endif /*__CHIBIOS_MPL3115A2__*/

