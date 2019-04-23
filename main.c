/*****************************************************************************
*
* Copyright (C) 2013 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the
*   distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of
*   its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* MSP432 empty main.c template
*
******************************************************************************/

#include "msp.h"
#include "mcp9808.h"
#include <stdint.h>
#include <stdio.h>


int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;

    //CS->KEY = CS_KEY_VAL;
    //CS->CTL1 |= CS_CTL1_DIVS__8;
    //CS->KEY &= ~CS_KEY_VAL;

    // Configure GPIO
    P1->SEL0 |= BIT6 | BIT7;                // I2C pins


    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC;  // Software reset enabled
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3 |          // I2C mode
    EUSCI_A_CTLW0_MST |             // Master mode
    EUSCI_B_CTLW0_SSEL__SMCLK;
    EUSCI_B0->BRW = 30;//30;
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;// Release eUSCI from reset state

    MCP9808 tempboi;
    initMCP9808 (&tempboi, EUSCI_B0, (uint8_t)24);
    setMCP9808Resolution (&tempboi, MCP9808_QUARTER_DEG);
    uint8_t res = getMCP9808Resolution (&tempboi);
    float temp = getMCP9808Temp (&tempboi);

    setMCP9808TempSentinel(&tempboi, MCP9808_TUPPER, temp);
    float temp2 = getMCP9808TempSentinel(&tempboi, MCP9808_TUPPER);
    //getMCP9808Config(&tempboi);
    //float temp = 0;
    //while (1){
    //    temp = getMCP9898Temp(&tempboi);
    //}
    //EUSCI_B0->IE |= EUSCI_B_IE_TXIE0 |      // Enable transmit interrupt
    //        EUSCI_B_IE_STPIE;               // Enable stop interrupt
    return 0;

}
