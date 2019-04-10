/*
 * mcp9808.c
 *
 *  Created on: Mar 6, 2019
 *      Author: Nicholas
 */
#include "mcp9808.h"

static int read8 (MCP9808* device, enum mcp9808_registers reg){
    __disable_irq();
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG){
        device->i2cDevice->IFG = 0;
        return -1;
    }
        device->i2cDevice->IFG = 0;

    device->i2cDevice->TXBUF = reg;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG){
        device->i2cDevice->IFG = 0;
        return -1;
    }
    device->i2cDevice->IFG = 0;


    device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG || device->i2cDevice->IFG & EUSCI_B_IFG_CLTOIFG){
        device->i2cDevice->IFG = 0;
        return -1;
    }
    device->i2cDevice->IFG = 0;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    uint16_t result = device->i2cDevice->RXBUF;

    device->readBuf = result;
    __enable_irq();
    return 0;
}

static int read16 (MCP9808* device, enum mcp9808_registers reg){
    __disable_irq();
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TR;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG){
        device->i2cDevice->IFG = 0;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return -1;
    }
        device->i2cDevice->IFG = 0;

    device->i2cDevice->TXBUF = reg;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG){
        device->i2cDevice->IFG = 0;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return -1;
    }
    device->i2cDevice->IFG = 0;


    device->i2cDevice->CTLW0 &= ~EUSCI_B_CTLW0_TR;
    device->i2cDevice->I2CSA = (uint16_t) device->address;
    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    while (device->i2cDevice->IFG == 0);
    if (device->i2cDevice->IFG & EUSCI_B_IFG_NACKIFG || device->i2cDevice->IFG & EUSCI_B_IFG_CLTOIFG){
        device->i2cDevice->IFG = 0;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return -1;
    }
    device->i2cDevice->IFG = 0;

    uint16_t result = device->i2cDevice->RXBUF << 8;

    while (device->i2cDevice->IFG == 0);
    if (!(device->i2cDevice->IFG & EUSCI_B_IFG_RXIFG0)){
        device->i2cDevice->IFG = 0;
        device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        return -1;
    }
    device->i2cDevice->IFG = 0;

    device->i2cDevice->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    result |= device->i2cDevice->RXBUF;

    device->readBuf = result;
    __enable_irq();
    return 0;
}

static void write8 (MCP9808* device, enum mcp9808_registers reg, uint8_t data){
    __disable_irq();
    __enable_irq();
}

static void write16 (MCP9808* device, enum mcp9808_registers reg, uint16_t data){
    __disable_irq();
    __enable_irq();
}


int initMCP9808 (MCP9808* device, EUSCI_B_Type *i2cPeripheral, uint8_t addressBits){
    device->i2cDevice = i2cPeripheral;
    if (setAddress (device, addressBits) == -1)
        return -1;

    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;
    i2cPeripheral->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_A_CTLW0_MST;
    i2cPeripheral->CTLW1 = EUSCI_B_CTLW1_CLTO_3;
    i2cPeripheral->IE = /*EUSCI_B_IE_CLTOIE |*/ EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_TXIFG0 | EUSCI_B_IE_NACKIE;
    i2cPeripheral->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;

    // Should also read the Config register for the MCP9808 and populate the device->config variable. This will also error-check the slave address.
    return 0;
}

int setAddress (MCP9808* device, uint8_t addressBits){
    if (addressBits & 0xF8)
        return -1;

    device->address = 0x18 | addressBits;
    return 0;
}

uint8_t getAddress(MCP9808* device){
    return device->address;
}

void shutdownOrWakeup (MCP9808* device, uint8_t shtdwn){
    device->config = (device->config & 0xEF) | shtdwn;
    write16 (device, MCP9808_CONFIG, device->config);
}
void shutdown (MCP9808* device){
    device->config = device->config | 0x10;
    write16 (device, MCP9808_CONFIG, device->config);
}
void wakeup (MCP9808* device){
    device->config = device->config & 0xEF;
    write16 (device, MCP9808_CONFIG, device->config);
}

float getMCP9898Temp (MCP9808* device){
    if (read16 (device, MCP9808_TA) == -1)
        return -255;
    uint16_t ta = device->readBuf;
    float temp = ta & 0x0FFF;
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

uint16_t getMCP9808Config (MCP9808* device){

}

void setMCP9808Config (MCP9808* device, uint16_t config){

}

uint16_t getMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg){
    if (reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT){
        return read16 (device, reg);
    }

    return 0xE000; // Impossible value since Tupper, Tlower, and Tcrit are 13-bit signed numbers.
}

void setMCP9808TempSent (MCP9808* device, enum mcp9808_registers reg, uint16_t temp){
    if (reg == MCP9808_TUPPER || reg == MCP9808_TLOWER || reg == MCP9808_TCRIT){
        write16 (device, reg, temp);
    }
}
