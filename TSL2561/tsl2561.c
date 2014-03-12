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

#include "tsl2561.h"

#define I2C_TIMEOUT MS2ST(1000)

msg_t tslReadRegister(I2CDriver * driver, uint8_t device,
                      uint8_t reg, uint8_t * byte)
{
    uint8_t command = CMD | (ADDRESS_MASK & reg);
    return i2cMasterTransmitTimeout(driver,
                                    device,
                                    &command, 1,
                                    byte, 1,
                                    I2C_TIMEOUT);   
}


msg_t tslReadRegisters(I2CDriver * driver, uint8_t device,
                       uint8_t reg, uint8_t * byte, uint8_t number)
{
    uint8_t command = CMD | (ADDRESS_MASK & reg);
    return i2cMasterTransmitTimeout(driver,
                                    device,
                                    &command, 1,
                                    byte, number,
                                    I2C_TIMEOUT);   
}


msg_t tslWriteRegister(I2CDriver * driver, uint8_t device,
                       uint8_t reg, uint8_t byte)
{
    uint8_t txBuf[2] =  {CMD | (ADDRESS_MASK & reg),
                         byte};

    return i2cMasterTransmitTimeout(driver,
                                    device,
                                    txBuf, 2,
                                    NULL, 0,
                                    I2C_TIMEOUT);   
}


msg_t tslSetPowerMode(I2CDriver * driver, uint8_t device,
                      powerMode mode)
{
    if (mode == POWER_UP)
    {
        return tslWriteRegister(driver, device, CONTROL, POWER_MASK);
    }
    else
    {
        return tslWriteRegister(driver, device, CONTROL, 0x00);
    }
}


msg_t tslReadChannels(I2CDriver * driver, uint8_t device, uint16_t * channel0,
                      uint16_t * channel1)
{
    uint8_t channels[4];
    msg_t msg;

    if (RDY_OK != (msg = tslReadRegisters(driver, device,
                                          DATA0LOW, channels, 4)))
    {
        return msg;
    }

    *channel0 = channels[0] + (((uint16_t)channels[1])<<8);
    *channel1 = channels[2] + (((uint16_t)channels[3])<<8);

    return msg;
}


msg_t tslSetGain(I2CDriver * driver, uint8_t device, tslGain gain)
{
    msg_t msg;
    uint8_t timing;

    if (RDY_OK != (msg = tslReadRegister(driver, device, TIMING, &timing)))
    {
        return msg;
    }
        
    if (gain == GAIN_HIGH_16X)
    {
        timing |= GAIN;
    }
    else 
    {
        timing &= ~GAIN;
    }

    if (RDY_OK != (msg = tslWriteRegister(driver, device, TIMING, timing)))
    {
        return msg;
    }

    return msg;
}


msg_t tslSetIntegrationTime(I2CDriver * driver, uint8_t device,
                            tslIntegrationTime time)
{
    msg_t msg;
    uint8_t timing;

    if (RDY_OK != (msg = tslReadRegister(driver, device, TIMING, &timing)))
    {
        return msg;
    }

    timing &= ~INTEG_MASK;
            
    switch (time)
    {
        case INT_TIME_13_7_MS:
            timing |= INT_TIME_13_7_MS;
            break;
        case INT_TIME_101_MS:
            timing |= INT_TIME_101_MS;
            break;
        case INT_TIME_402_MS:
            timing |= INT_TIME_402_MS;
            break;
        case INT_TIME_MANUAL:
            timing |= INT_TIME_MANUAL;
            break;
    }

    if (RDY_OK != (msg = tslWriteRegister(driver, device, TIMING, timing)))
    {
        return msg;
    }

    return msg;
}


msg_t tslReadLuxConvertSleep(I2CDriver * driver, uint8_t device, uint16_t * lux)
{
    uint8_t msg;
    uint16_t channel0;
    uint16_t channel1;

    if (RDY_OK != (msg = tslSetIntegrationTime(driver, device, 
                                               INT_TIME_402_MS)))
    {
        return msg;
    }

    else if (RDY_OK != (msg = tslSetGain(driver, device, GAIN_HIGH_16X)))
    {
        return msg;
    }
            
    else if (RDY_OK != (msg = tslSetPowerMode(driver, device, POWER_UP)))
    {
        return msg;
    }

    chThdSleep(MS2ST(450));

    if (RDY_OK != (msg = tslReadChannels(driver, device, &channel0,
                                         &channel1)))
    {
        return msg;
    }

    if (RDY_OK != (msg = tslSetPowerMode(driver, device, POWER_DOWN)))
    {
        return msg;
    }

    *lux = calculateLux(GAIN_HIGH_16X, INT_TIME_402_MS, channel0, channel1, 
                        TYPE_TMB);
    return msg;
}


/* Using the code provided in the data sheet instead. It doesn't need floating
 * point numbers and saves using the math library. */
# if 0
/* can return -1 on error */
float calculateLuxFloat(uint16_t channel0, uint16_t channel1, tslGain gain, 
                        tslIntegrationTime time, tslType type)
{
    /* TODO sort out gain and integration time. */
    float ratio = (float)channel1/(float)channel0;
    float scale = 1.0;

    switch (time)
    {
        case INT_TIME_13_7_MS:
            scale = 0.034;
            break;
        case INT_TIME_101_MS:
            scale = 0.252;
            break;
        case INT_TIME_402_MS:
        case INT_TIME_MANUAL: /* ALAN TODO */
            scale = 1.0;
            break;
    }

    if (gain == GAIN_LOW_1X)
    {
        scale *= 16.0;
    }

    channel0 *= scale;
    channel1 *= scale;

    if (type == TYPE_TMB)
    {
        if (ratio < 0.5)
        {
            return 0.0304 * channel0 - 0.062 * channel0 * pow(ratio,1.4);
        }
        else if (ratio < 0.61)
        {
            return 0.0224 * channel0 - 0.031 * channel1;
        }
        else if (ratio < 0.80)
        {
            return 0.0128 * channel0 - 0.0153 * channel1;
        }
        else if (ratio < 1.30)
        {
            return 0.00146 * channel0 - 0.00112 * channel1;
        }
        else
        {
            return 0.0;
        }
    }
    else
    {
        if (ratio < 0.52)
        {
            return 0.0315 * channel0 - 0.0593 * channel0 * pow(ratio,1.4);
        }
        else if (ratio < 0.65)
        {
            return 0.0229 * channel0 - 0.0291 * channel1;
        }
        else if (ratio < 0.80)
        {
            return 0.0157 * channel0 - 0.0180 * channel1;
        }
        else if (ratio < 1.30)
        {
            return 0.00338 * channel0 - 0.00260 * channel1;
        }
        else
        {
            return 0.0;
        }
    }
}
#endif


