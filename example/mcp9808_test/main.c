#include "msp.h"
#include "mcp9808.h"
#include <stdint.h>
#include <stdio.h>

MCP9808 sensor;

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;


    // Configure P1.6 & 1.7 as I2C pins
    P1->SEL0 |= BIT6 | BIT7;

    // Configure the EUSCI B 0 as an I2C master
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SYNC; // Software reset enabled
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_A_CTLW0_MST | EUSCI_B_CTLW0_SSEL__SMCLK; // I2C mode, master mode, use SMCLK (3MHz default)
    EUSCI_B0->BRW = 30; // Clock prescaler. 3MHz / 30 = 100kHz
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    int code = initMCP9808(&sensor, EUSCI_B0, (uint8_t)0x18);

    if (code == -1)
        printf("Invalid MCP9808 slave address\n");
    else if (code == -2)
        printf("MCP9808 not found\n");
    else if (code == 3)
        printf("Cannot R/W over I2C bus. Check circuit for errors.\n");
    else
    {
        printf("Found MCP9808\n");
        // Setup systick timer to read temperature every 2 seconds
        SysTick->LOAD = 6000000; // 6,000,000 / 3MHz = 2s
        SysTick->VAL = 0;
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Start counting

        __enable_irq();
        while(1)
            __sleep();
    }
    return -1;
}

void SysTick_Handler (void){
    float temp = getMCP9808Temp (&sensor);
    if (temp == -65535)
        printf("Cannot R/W over I2C bus. Check circuit for errors.\n");
    else
        printf("Temperature: %.3f*C\n", temp);
}
