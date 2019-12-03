#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"

#define ADC_RESOLUTION 1024
char adcDone = 0, adcBusy; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int sensorRes[2][3], resIndex = 0, curSensor = 0;
char zoneState = 0;
uint8_t cliBuffer[20];                    /* CLI output buffer */
uint8_t cliIndex = 0;                                /* CLI buffer index */
char uartReceived = 0;
int threshold[2][2];
int enable[2][2];
/*
 * This project contains some code samples that may be useful.
 *
 */

/*
 * UART operation
 */


/* Tx to UART */
void uartDisplay(uint8_t *sendText, uint8_t length)
{
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */

    int i;
    for (i = 0 ; i < length ; i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, sendText[i]); /* send message */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    }

    if ((sendText[0] != '>') && (sendText[0] != '#') && (sendText[0] != ' ')) /* if not enter key or welcome message, it was command, make new line */
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '>'); /* send new prompt */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 32); /* send space*/
    }
}

void testCLIMessage() {
    uint8_t cliWelcome[100];
    int cliIndex;
    for (cliIndex = 0 ; cliIndex < 100 ; cliIndex++)
        cliWelcome[cliIndex]= 0; /* initialize welcome message */

    strcpy((char*) cliWelcome, " ECE 298 - Group 15  GreenHouse\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use e v 0 to enable the ventilation motor in zone 0\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use e v 1 to enable the ventilation motor in zone 1\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use d v 0 to disable the ventilation motor in zone 0\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use d v 1 to disable the ventilation motor in zone 0\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " replace v with i to enable/disable irrigation motor\r\n");

    strcpy((char*) cliWelcome, " use s v 0 value to set the threshold for ventilation more in zone 0\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, "> "); /* prompt */
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
}



/* UART communication */
void uartTransmit(void)
{
    uartReceived = 0;
    uint8_t txMsg[100]; /* UART TX message */

    int i;
    for (i = 0 ; i < 50 ; i++) /* initialize txMsg buffer */
        txMsg[i] = 0;

//    for (i = 0 ; i < 20 ; i++) /* initialize txMsg buffer */
//        txMsg[i] = cliBuffer[i];
//    displayScrollText(cliBuffer);

    if (! strcmp((char*)cliBuffer, "")) /* enter key*/
    {
        strcpy((char*) txMsg, "> "); /* prompt */
        uartDisplay(txMsg, strlen((char*) txMsg));
    }

    else if (cliBuffer[0] == 's') /* threshold set */
    {
        //Example:
        //s v 0 100

        int newThreshold = -1;
        if (isdigit(cliBuffer[8])) /* 3 digit value */
            newThreshold = (cliBuffer[6] - 48)*100 + (cliBuffer[7] - 48)*10 + (cliBuffer[8] - 48);
        else if (isdigit(cliBuffer[7])) /* 2 digit value */
            newThreshold = (cliBuffer[6] - 48)*10 + (cliBuffer[7] - 48);
        else if (isdigit(cliBuffer[6])) /* 1 digit value */
            newThreshold = (cliBuffer[6] - 48);


        if ((newThreshold >= 0) && (newThreshold <= 100)) {
            // check if it is valid
            int zone = cliBuffer[4] - '0';
            if (cliBuffer[2] == 'v') {
                threshold[zone][0] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for temperature motor was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            } else if (cliBuffer[2] == 'i') {
                threshold[zone][1] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for mositure motor was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
        }
        else /* threshold set error*/
        {
            strcpy((char*) txMsg, "ERROR: Invalid threshold!");
            uartDisplay(txMsg, strlen((char*) txMsg));
        }
    } else if (cliBuffer[0] == 'e') {
        // example:
        //e v 0
        int zone = cliBuffer[4] - '0';
        if (cliBuffer[2] == 'v') {
            enable[zone][0] = 1;
            strcpy((char*) txMsg, "enable ventilation motor");
            uartDisplay(txMsg, strlen((char*) txMsg));
        } else if (cliBuffer[2] == 'i') {
            enable[zone][1] = 1;
            strcpy((char*) txMsg, "enable irrigation motor");
            uartDisplay(txMsg, strlen((char*) txMsg));
        }
    }  else if (cliBuffer[0] == 'd') {
        // example:
        //d v 0
        int zone = cliBuffer[4] - '0';
        if (cliBuffer[2] == 'v') {
            enable[zone][0] = 0;
            disableVen(zone);
            strcpy((char*) txMsg, "disable ventilation motor");
            uartDisplay(txMsg, strlen((char*) txMsg));
        } else if (cliBuffer[2] == 'i') {
            enable[zone][1] = 0;
            disableIR(zone);
            strcpy((char*) txMsg, "disable irrigation motor");
            uartDisplay(txMsg, strlen((char*) txMsg));
        }
    }

    cliIndex = 0;
    for (i = 0 ; i < 20 ; i++) /* clear receive buffer */
        cliBuffer[i] = 0;
}

void parseResult(int sensor, int val, int zone) {
    uint8_t result = 0;
    if (sensor == 0) {
        // temp sensor
        result = 300*(((float)val/ADC_RESOLUTION));
        if (result > 99)
            result = 99;
    } else if (sensor == 1) {
        // mois sensor
        result = 160*(((float)val/ADC_RESOLUTION));
        if (result > 99)
            result = 99;
    } else if (sensor == 2) {
        // light sensor
        if (val >= 1015) {
            result = 1;
        }
    }
    sensorRes[zone][sensor] = result;
}

void readZone0(int sensorNum) {
    if (!sensorNum) {
        // LLL to read A0, temp sensor
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
    } else if (sensorNum == 1) {
        // HLL to read A1, mositure sensor
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
    } else if (sensorNum == 2) {
        // LHL to read A2, light sensor
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
    }
}

void readZone1(int sensorNum) {
    if (!sensorNum) {
        // LLL to read A7, temp sensor
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
    } else if (sensorNum == 1) {
        // HLL to read A5, mositure sensor
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
    } else if (sensorNum == 2) {
        // LHL to read A4, light sensor
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
    }
}

void enableVen(int zone) {
    if (zone == 0) {
        if (enable[0][0] == 1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
        }
    } else if (zone == 1) {
        if (enable[1][0] == 1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        }
    }
}

void disableVen(int zone) {
    if (zone == 0) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
    } else if (zone == 1) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    }
}

void enableIR(int zone) {
    if (zone == 0) {
        if (enable[0][1] == 1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
        }
    } else if (zone == 1) {
        if (enable[1][1] == 1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        }
    }
}

void disableIR(int zone) {
    if (zone == 0) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    } else if (zone == 1) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    }
}

void work_motor() {
    int zone = 0;
    for (zone = 0; zone < 2; ++zone) {
        if (sensorRes[zone][2] == 1) {
            // light, we will only need to check temperature sensor
            if (sensorRes[zone][0] <= threshold[zone][0]) {
                enableVen(zone);
            } else {
                disableVen(zone);
            }
            disableIR(zone);
        } else {
            // night, we will only need to check moisture sensor
            if (sensorRes[zone][1] <= threshold[zone][1]) {
                enableIR(zone);
            } else {
                disableIR(zone);
            }
            disableVen(zone);
        }
    }
}

void main(void)
{
    char buttonPressed = 0;
    char displayedZone = 0;
    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    threshold[0][0] = 71; // temperature
    threshold[1][0] = 71; // temperature
    threshold[0][1] = 5; // moisture
    threshold[1][1] = 5; // moisture
    enable[0][0] = 1;
    enable[0][1] = 1;
    enable[1][0] = 1;
    enable[1][1] = 1;
    testCLIMessage();

    displayScrollText("ECE298 GROUP15");
     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    __delay_cycles(500000);

    //All done initializations - turn interrupts back on.
    __enable_interrupt();
    while (1) {
        if (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0 && !buttonPressed) {
            buttonPressed = 1;
            if (displayedZone == 0) {
                displayedZone = 1;
            } else {
                displayedZone = 0;
            }
        } else {
            buttonPressed = 0;
        }

        if (!adcBusy) {
            if (zoneState == 0) {
                readZone0(curSensor);
            } else {
                readZone1(curSensor);
            }
            adcBusy = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }
        if (adcDone) {
            parseResult(curSensor, (int)ADCResult, zoneState);
            ++curSensor;
            if (curSensor >= 3) {
                curSensor -= 3;
                zoneState = zoneState == 1 ? 0 : 1;
            }
            adcDone = 0;
        }
        work_motor();
        display(sensorRes[displayedZone][0], sensorRes[displayedZone][1], sensorRes[displayedZone][2], displayedZone); //Put the previous result on the LCD display
        if (uartReceived) {
            uartTransmit();
        }
        __delay_cycles(200000);
    }
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);
    uint8_t data = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }

    if (data == '\r') {
        // enter key
        uartReceived = 1;
    } else if (data == 0x7f) {
        // backspace key
        // need to remove the last digit from the clibuffer if it exists
        if (cliIndex > 0) {
            --cliIndex;
            if (cliIndex < 20) {
                cliBuffer[cliIndex] = 0;
            }
        }
    } else if (cliIndex < 20) {
        if ((isalpha(tolower(data))) || (isdigit(data)) || (data == ' ')) /* legal keys */
            cliBuffer[cliIndex] = tolower(data); /* store key */
        ++cliIndex;
    } else {
        ++cliIndex;
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

void Init_ADC_MOIS(void) {
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT_MOIS, ADC_IN_PIN_MOIS, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL_MOIS,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        adcBusy = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
        adcDone = 1;
    }
}


