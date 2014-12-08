/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file HardWire.h
 * @author Trystan Jones <crenn6977@gmail.com>
 * @brief Wire library, uses the hardware I2C available in the Maple to
 *        interact with I2C slave devices.
 */

/*
 * Library created by crenn to use the new WireBase system and allow Arduino
 * users easy interaction with the I2C Hardware in a familiar method.
 */


#ifndef _HARDWIRE_H_
#define _HARDWIRE_H_

#include "i2c.h"
#include "wirish.h"
#include "WireBase.h"

class HardWire : public WireBase {
private:
    i2c_dev* sel_hard;
    uint8    dev_flags;
protected:
    /*
     * Processes the incoming I2C message defined by WireBase to the
     * hardware. If an error occured, restart the I2C device.
     */
    uint8 process();
public:
    /*
     * Check if devsel is within range and enable selected I2C interface with
     * passed flags
     */
    HardWire(uint8, uint8 = 0);

    /*
     * Disables the I2C device and remove the device address.
     */
    ~HardWire();
};

#endif // _HARDWIRE_H_
