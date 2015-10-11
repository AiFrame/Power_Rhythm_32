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
 * @file Wire.cpp
 * @author Trystan Jones <crenn6977@gmail.com>
 * @brief Wire library, uses the WireBase to create the primary interface
 *        while keeping low level interactions invisible to the user.
 */

/*
 * Library updated by crenn to follow new Wire system.
 * Code was derived from the original Wire for maple code by leaflabs and the
 * modifications by gke and ala42.
 */

#include "Wire.h"

#define I2C_WRITE 0
#define I2C_READ  1

/* low level conventions:
 * - SDA/SCL idle high (expected high)
 * - always start with i2c_delay rather than end
 */

void TwoWire::set_scl(bool state) {
    I2C_DELAY(this->i2c_delay);
    digitalWrite(this->scl_pin,state);
    //Allow for clock stretching - dangerous currently
    if (state == HIGH) {
        while(digitalRead(this->scl_pin) == 0);
    }
}

void TwoWire::set_sda(bool state) {
    I2C_DELAY(this->i2c_delay);
    digitalWrite(this->sda_pin, state);
}

void TwoWire::i2c_start() {
    set_sda(LOW);
    set_scl(LOW);
}

void TwoWire::i2c_stop() {
    set_sda(LOW);
    set_scl(HIGH);
    set_sda(HIGH);
}

bool TwoWire::i2c_get_ack() {
    set_scl(LOW);
    set_sda(HIGH);
    set_scl(HIGH);

    bool ret = !digitalRead(this->sda_pin);
    set_scl(LOW);
    return ret;
}

void TwoWire::i2c_send_ack() {
    set_sda(LOW);
    set_scl(HIGH);
    set_scl(LOW);
}

void TwoWire::i2c_send_nack() {
    set_sda(HIGH);
    set_scl(HIGH);
    set_scl(LOW);
}

uint8 TwoWire::i2c_shift_in() {
    uint8 data = 0;
    set_sda(HIGH);

    int i;
    for (i = 0; i < 8; i++) {
        set_scl(HIGH);
        data |= digitalRead(this->sda_pin) << (7-i);
        set_scl(LOW);
    }

    return data;
}

void TwoWire::i2c_shift_out(uint8 val) {
    int i;
    for (i = 0; i < 8; i++) {
        set_sda(!!(val & (1 << (7 - i)) ) );
        set_scl(HIGH);
        set_scl(LOW);
    }
}

uint8 TwoWire::process() {
    itc_msg.xferred = 0;

    uint8 sla_addr = (itc_msg.addr << 1);
    if (itc_msg.flags == I2C_MSG_READ) {
        sla_addr |= I2C_READ;
    }
    i2c_start();
    // shift out the address we're transmitting to
    i2c_shift_out(sla_addr);
    if (!i2c_get_ack()) {
        return ENACKADDR;
    }
    // Recieving
    if (itc_msg.flags == I2C_MSG_READ) {
        while (itc_msg.xferred < itc_msg.length) {
            itc_msg.data[itc_msg.xferred++] = i2c_shift_in();
            if (itc_msg.xferred < itc_msg.length) {
                i2c_send_ack();
            } else {
                i2c_send_nack();
            }
        }
    }
    // Sending
    else {
        for (uint8 i = 0; i < itc_msg.length; i++) {
            i2c_shift_out(itc_msg.data[i]);
            if (!i2c_get_ack()) {
                return ENACKTRNS;
            }
            itc_msg.xferred++;
        }
    }
    i2c_stop();
    return SUCCESS;
}

// TODO: Add in Error Handling if pins is out of range for other Maples
// TODO: Make delays more capable
TwoWire::TwoWire(uint8 scl, uint8 sda, uint8 delay) : i2c_delay(delay) {
    this->scl_pin=scl;
    this->sda_pin=sda;
}

void TwoWire::begin(uint8 self_addr) {
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;
    rx_buf_len = 0;
    pinMode(this->scl_pin, OUTPUT_OPEN_DRAIN);
    pinMode(this->sda_pin, OUTPUT_OPEN_DRAIN);
    set_scl(HIGH);
    set_sda(HIGH);
}

TwoWire::~TwoWire() {
    this->scl_pin=0;
    this->sda_pin=0;
}

// Declare the instance that the users of the library can use
TwoWire Wire();
