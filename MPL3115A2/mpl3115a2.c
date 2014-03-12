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

#include "mpl3115a2.h"

#define I2C_TIMEOUT MS2ST(1000)

msg_t mplRegisterWrite(I2CDriver * driver, uint8_t devAddress,
                       uint8_t regAddress, uint8_t value)
{
    uint8_t txBuf[2] = {regAddress,value};

    return i2cMasterTransmitTimeout(driver,
                                    devAddress,
                                    txBuf, sizeof(txBuf),
                                    NULL, 0,
                                    I2C_TIMEOUT);   
}


msg_t mplRegisterRead(I2CDriver * driver, uint8_t devAddress,
                      uint8_t regAddress, uint8_t * readByte)
{
    return i2cMasterTransmitTimeout(driver,
                                    devAddress,
                                    &regAddress, 1,
                                    readByte, 1,
                                    I2C_TIMEOUT);   
}


msg_t mplRegistersRead(I2CDriver * driver, uint8_t devAddress,
                       uint8_t regAddress, uint8_t * readBytes,
                       uint8_t readNumber)
{
    return i2cMasterTransmitTimeout(driver,
                                    devAddress,
                                    &regAddress, 1,
                                    readBytes, readNumber,
                                    I2C_TIMEOUT); 
}

msg_t mplSetPowerMode(I2CDriver * driver, uint8_t devAddress, mplPowerMode mode)
{
    uint8_t temp;
    msg_t msg;
    
    if (RDY_OK != (msg = mplRegisterRead(driver, devAddress, CTRL_REG1, &temp)))
    {
        return msg;
    }
    
    if (mode == ACTIVE)
    {
        temp |= SBYB;
    }
    else 
    {
        temp &= ~SBYB;
    }

    return mplRegisterWrite(driver, devAddress, CTRL_REG1, temp);
}

msg_t mplSetMeasurementMode(I2CDriver * driver, uint8_t devAddress,
                            mplMeasurementMode mode)
{
    uint8_t temp;
    msg_t msg;
    
    if (RDY_OK != (msg = mplRegisterRead(driver, devAddress, CTRL_REG1, &temp)))
    {
        return msg;
    }
    
    if (mode == ALTIMETER)
    {
        temp |= ALT;
    }
    else 
    {
        temp &= ~ALT;
    }

    return mplRegisterWrite(driver, devAddress, CTRL_REG1, temp);
}

msg_t mplPerformOneShot(I2CDriver * driver, uint8_t devAddress)
{
    uint8_t currentCtrlReg1;
    msg_t msg;

    if (RDY_OK != (msg = mplRegisterRead(driver, devAddress, CTRL_REG1,
                                         &currentCtrlReg1)))
    {
        return msg;
    }
   
    if (currentCtrlReg1 & OST)
    {
        currentCtrlReg1 &= ~OST;

        if (RDY_OK != (msg = mplRegisterWrite(driver, devAddress,
                                              CTRL_REG1, currentCtrlReg1)))
        {
            return msg;
        }
    }

    currentCtrlReg1 |= OST;

    return mplRegisterWrite(driver, devAddress, CTRL_REG1, currentCtrlReg1);

}


msg_t mplReadTemperature(I2CDriver * driver, uint8_t devAddress,
                         float * temperature)
{
    msg_t msg;
    uint8_t rxBuf[3]; 

    if (RDY_OK != (msg = mplRegistersRead(driver, devAddress,
                                          OUT_T_MSB, 
                                          rxBuf, 2)))
    {
        return msg;
    }

    *temperature = (float)(rxBuf[1] >> 4)/16.0;
    *temperature += (float)rxBuf[0];

    return msg;
}


msg_t mplReadPressure(I2CDriver * driver, uint8_t devAddress, float * pressure)
{
    msg_t msg;
    uint8_t rxBuf[3]; 

    if (RDY_OK != (msg = mplRegistersRead(driver, devAddress,
                                          OUT_P_MSB, 
                                          rxBuf, 3)))
    {
        return msg;
    }

    *pressure =  (float)((uint32_t)rxBuf[0]<<10);   /* MSB */
    *pressure += (float)(rxBuf[1]<<2);              /* CSB */
    *pressure += (float)((rxBuf[2]&0xC0)>>6);       /* LSB Integer */
    *pressure += (float)((rxBuf[2]&0x30)>>4)/4.0;   /* LSB Decimal */

    return msg;
}


msg_t mplReadAltitude(I2CDriver * driver, uint8_t devAddress, float * altitude)
{
    msg_t msg;
    uint8_t rxBuf[3]; 

    if (RDY_OK != (msg = mplRegistersRead(driver, devAddress,
                                          OUT_P_MSB, 
                                          rxBuf, 3)))
    {
        return msg;
    }

    *altitude =  (float)((int16_t)rxBuf[0]<<8);     /* MSB */
    *altitude += (float)((int16_t)rxBuf[1]);        /* CSB */
    *altitude += (float)((rxBuf[2]&0xF0)>>4)/16.0;  /* LSB Decimal */

    return msg;
}


msg_t mplOneShotReadBarometer(I2CDriver * driver, uint8_t devAddress,
                              float * pressure, float * temperature)
{
    msg_t msg;

    if (RDY_OK != (msg = mplSetPowerMode(driver, devAddress, STANDBY)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplSetMeasurementMode(driver, devAddress,
                                                    BAROMETER)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplPerformOneShot(driver, devAddress)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplReadTemperature(driver, devAddress,
                                                 temperature)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplReadPressure(driver, devAddress, pressure)))
    {
        return msg;
    }

    return msg;
}


msg_t mplOneShotReadAll(I2CDriver * driver, uint8_t devAddress,
                        float * altitude, float * pressure, float * temperature)
{
    msg_t msg;

    if (RDY_OK != (msg = mplOneShotReadBarometer(driver, devAddress,
                                                 pressure, temperature)))
    {
        return msg;
    }
    else if (RDY_OK != (msg = mplSetMeasurementMode(driver, devAddress,
                                                    ALTIMETER)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplPerformOneShot(driver, devAddress)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = mplReadAltitude(driver, devAddress, altitude)))
    {
        return msg;
    }

    return msg;
}


