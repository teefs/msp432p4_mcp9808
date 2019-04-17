/*
 * mcp9808.c
 *
 *  Created on: Mar 6, 2019
 *      Author: Nicholas
 */
#include "mcp9808.h"
#include <stdint.h>
#include <stdlib.h>


/* Declaration for local functions and resources that do not need to be exposed to the user. */
#define NACKIV 0x4

static uint8_t read8 (MCP9808* device, enum mcp9808_registers reg);
static uint16_t read16 (volatile MCP9808* device, enum mcp9808_registers reg, uint16_t* buffer)
static int write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data)
static int write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data)
static uint16_t getConfig (MCP9808* device);
static int setMCP9808Config (MCP9808* device, uint16_t config);


int initMCP9808 (MCP9808* device, volatile EUSCI_B_Type *i2cPeripheral, uint8_t addressBits){
    device->i2cDevice = i2cPeripheral;
    if (setAddress (device, addressBits) == -1)
        return -1;

    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;
    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_A_CTLW0_MST;
    i2cPeripheral->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
    i2cPeripheral->IE = EUSCI_B_IE_RXIE0 | EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;

    // Should also read the Config register for the MCP9808 and populate the device->config variable. This will also error-check the slave address.
    return 0;
}

int setAddress (MCP9808* device, uint8_t address){
    if (address < 0x18 || address > 0x1F)
        return -1;

    device->address = address;
    return 0;
}

uint8_t getAddress(MCP9808* device){
    return device->address;
}

void shutdownOrWakeup (MCP9808* device, uint8_t shtdwn){
//    device->config = (device->config & 0xEF) | shtdwn;
//    write16 (device, MCP9808_CONFIG, device->config);
}

float getMCP9898Temp (MCP9808* device){
    uint16_t ta;
    if (read16 (device, MCP9808_TA, &ta) == -1)
        return -255;
    ta &= 0xFFF;
    float temp = (float) ta;
    temp /= 16.0;
    if (ta & 0x1000)
        temp -= 256;
    return temp;
}
uint8_t getMCP9808Res (MCP9808* device){
    return read8 (device, MCP9808_RESOLUTION);
}

void setMCP9808Res (MCP9808* device, enum mcp9808_resolution res){
    write8 (device, MCP9808_RESOLUTION, (uint8_t) res);
}

int getMCP9808Config (MCP9808* device){
//    if (read16 (device, MCP9808_MANUID) == -1)
//            return -1;
//    device->config = device->readBuf;
    return 0;
}

void setMCP9808Config (MCP9808* device, uint16_t config){

}

uint16_t getMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg){
    if (reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT){
//        return read16 (device, reg);
    }

    return 0xE000; // Impossible value since Tupper, Tlower, and Tcrit are 13-bit signed numbers.
}

void setMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg, uint16_t temp){
    if (reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT){
        write16 (device, reg, temp);
    }
}

static uint8_t read8 (MCP9808* device, enum mcp9808_registers reg);{
    return 0;
}

static uint16_t read16 (volatile MCP9808* device, enum mcp9808_registers reg, uint16_t* buffer){
    __disable_irq();
    if (buffer == NULL)
        return -1;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }

    device->i2cDevice->TXBUF = reg;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }
    device->i2cDevice->IFG = 0;


    device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }
    device->i2cDevice->IFG = 0;

    *buffer = EUSCI_B0->RXBUF << 8;

    while (device->i2cDevice->IV == 0);
    if (device->i2cDevice->IV & NACKIV){
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        device->i2cDevice->IFG = 0;
        return -1;
    }
    device->i2cDevice->IFG = 0;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    *buffer |= EUSCI_B0->RXBUF;

    __enable_irq();
    return 0;
}

static int write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data){
    __disable_irq();
    __enable_irq();
}

static int write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data){
    __disable_irq();
    __enable_irq();
}
