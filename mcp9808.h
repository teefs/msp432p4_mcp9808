/*
 * mcp9808.h
 *
 *  Created on: Mar 6, 2019
 *      Author: Nicholas
 */

#ifndef MCP9808_H_
#define MCP9808_H_

#include "msp.h"
#include <stdint.h>

#define

enum mcp9808_registers {CONFIG = 0001, TUPPER = 0010, TLOWER = 0011, TCRIT = 0100, TA = 0101, MANUID = 0110, DEVID = 0111, RESOLUTION = 1000};

typedef struct MSP432P4_MCP9808 {
    uint8_t address;
    EUSCI_B_Type *i2cDevice;

    uint8_t hysteresis;
    uint8_t shutdown;
    uint8_t critTripLock;
    uint8_t alarmWinLock;
    uint8_t alertStatus;
    uint8_t outputCtrl;
    uint8_t critAlertOnly;
    uint8_t alertPolarity;
    uint8_t alertCompInt;
} MCP9808;

initMCP9808 (MCP9808* device, uint8_t addressBits);
int setAddressBits (MCP9808* device, uint8_t addressBits);
uint8_t getAddressBits(MCP9808* device);

void shutdownOrWakeup (MCP9808* device, uint8_t shtdwnOrWk);
void shutdown (MCP9808* device);
void wakeup (MCP9808* device);


uint16_t readMCP9808 (MCP9808* device, enum mcp9808_registers reg);
void writeMCP9808 (MCP9808* device, enum mcp9808_registers reg, uint16_t data);

#endif /* MCP9808_H_ */
