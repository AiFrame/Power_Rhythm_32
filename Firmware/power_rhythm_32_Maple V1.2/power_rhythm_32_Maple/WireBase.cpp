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
 * @file WireBase.cpp
 * @author Trystan Jones <crenn6977@gmail.com>
 * @brief Wire library, following the majority of the interface from Arduino.
 *        Provides a 'standard' interface to I2C (two-wire) communication for
 *        derived classes.
 */

/*
 * Library created by crenn to allow a system which would provide users the
 * 'standardised' Arduino method for interfacing with I2C devices regardless of
 * whether it is I2C hardware or emulating software.
 */

#include "WireBase.h"

void WireBase::begin(uint8 self_addr) {
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;
    rx_buf_len = 0;
}

void WireBase::beginTransmission(uint8 slave_address) {
    itc_msg.addr = slave_address;
    itc_msg.data = &tx_buf[tx_buf_idx];
    itc_msg.length = 0;
    itc_msg.flags = 0;
}

void WireBase::beginTransmission(int slave_address) {
    beginTransmission((uint8)slave_address);
}

uint8 WireBase::endTransmission(void) {
    if (tx_buf_overflow) {
        return EDATA;
    }
    process();
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    return SUCCESS;
}

//TODO: Add the ability to queue messages (adding a boolean to end of function
// call, allows for the Arduino style to stay while also giving the flexibility
// to bulk send
uint8 WireBase::requestFrom(uint8 address, int num_bytes) {
    if (num_bytes > WIRE_BUFSIZ) {
        num_bytes = WIRE_BUFSIZ;
    }
    itc_msg.addr = address;
    itc_msg.flags = I2C_MSG_READ;
    itc_msg.length = num_bytes;
    itc_msg.data = &rx_buf[rx_buf_idx];
    process();
    rx_buf_len += itc_msg.xferred;
    itc_msg.flags = 0;
    return rx_buf_len;
}

uint8 WireBase::requestFrom(int address, int numBytes) {
    return WireBase::requestFrom((uint8)address, numBytes);
}

void WireBase::send(uint8 value) {
    if (tx_buf_idx == WIRE_BUFSIZ) {
        tx_buf_overflow = true;
        return;
    }
    tx_buf[tx_buf_idx++] = value;
    itc_msg.length++;
}

void WireBase::send(uint8* buf, int len) {
    for (uint8 i = 0; i < len; i++) {
        send(buf[i]);
    }
}

void WireBase::send(int value) {
    send((uint8)value);
}

void WireBase::send(int* buf, int len) {
    send((uint8*)buf, (uint8)len);
}

void WireBase::send(char* buf) {
    uint8 *ptr = (uint8*)buf;
    while (*ptr) {
        send(*ptr);
        ptr++;
    }
}

uint8 WireBase::available() {
    return rx_buf_len - rx_buf_idx;
}

uint8 WireBase::receive() {
    if (rx_buf_idx == rx_buf_len) {
        rx_buf_idx = 0;
        rx_buf_len = 0;
        return 0;
    } else if (rx_buf_idx == (rx_buf_len-1)) {
        uint8 temp = rx_buf[rx_buf_idx];
        rx_buf_idx = 0;
        rx_buf_len = 0;
        return temp;
    }
    return rx_buf[rx_buf_idx++];
}
