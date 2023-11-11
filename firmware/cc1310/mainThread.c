/*
 * Copyright (c) 2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* Example/Board Header files */
#include "Board.h"

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>

//Added from example
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
//#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/crypto/CryptoCC26XX.h>
#include "easylink/EasyLink.h"





#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/devices/DeviceFamily.h>

/* Driverlib APIs */
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

/* Application Header files */
#include "smartrf_settings/smartrf_settings.h"

/* Driverlib APIs */
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)




#define MAC_LENGTH 4
#define AAD_LENGTH 4
#define NONCE_LENGTH 12

const uint8_t key[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
char messageBuffer[128] = "Of First Disobedience and the Fruit Of that Forbidden Tree, whose mortal taste Brought Death into the World, and all our woe...\0";

uint32_t frequency = 0x0364; //868.00000 MHz
uint16_t DEBUGLEVEL = 9999; //Debug 0 shows no debugging; debug 9999 shows all
int8_t   txPower = 0xA73F;

static int32_t encryptMessage(char *messageBuffer, char *nonce, uint8_t msgLength, int32_t keyIndex, CryptoCC26XX_Handle cryptoHandle)
{
    uint8_t plainTextLength     = msgLength;
    uint8_t cipherTextLength    = MAC_LENGTH + plainTextLength;
    uint8_t header[AAD_LENGTH]  = {plainTextLength, cipherTextLength, MAC_LENGTH, NONCE_LENGTH};
    CryptoCC26XX_AESCCM_Transaction trans;

    /* Load key to the key register */
    CryptoCC26XX_loadKey(cryptoHandle, keyIndex, (const uint32_t *)key);

    /* Initialize encryption transaction */
    CryptoCC26XX_Transac_init((CryptoCC26XX_Transaction *)&trans, CRYPTOCC26XX_OP_AES_CCM_ENCRYPT);

    trans.keyIndex      = keyIndex;
    trans.authLength    = MAC_LENGTH;
    trans.nonce         = (char *) nonce;
    trans.header        = (char *) header;
    trans.fieldLength   = 15 - NONCE_LENGTH;
    trans.msgInLength   = plainTextLength;
    trans.headerLength  = AAD_LENGTH;
    trans.msgIn         = messageBuffer; /* Points to the plaintext */
    trans.msgOut        = (char *)&(messageBuffer[plainTextLength]); /* position of MAC field */

    /* Start the encryption transaction */
    return CryptoCC26XX_transact(cryptoHandle, (CryptoCC26XX_Transaction *)&trans);
}

/*
*   ======== messageEncrypter ========
*   Running function that will wait for input from the user and then encrypt the input message
*/
void messageEncrypter(CryptoCC26XX_Handle cryptoHandle, PIN_Handle ledPinHandle ) {
    int32_t             keyIndex;
    int32_t             status;
    uint8_t             nonce[NONCE_LENGTH];
    EasyLink_TxPacket   txPacket = {0};

    /* Store key and receive key index for the first avilable keylocation */
    keyIndex = CryptoCC26XX_allocateKey(cryptoHandle, CRYPTOCC26XX_KEY_ANY, (const uint32_t*)key);
    if(keyIndex == CRYPTOCC26XX_STATUS_ERROR) {
        /* key allocation failed */
        while(1);
    }

    /* generate new even nonce for every packet */
    generateNonce(nonce, NONCE_LENGTH, false);
    /* encrypt the user input with the newly generated nonce */
    status = encryptMessage(messageBuffer, (char *)nonce, 128, keyIndex, cryptoHandle);
    if(status == CRYPTOCC26XX_STATUS_SUCCESS){
        /* toggle led to indicate that the message has been successfully encrypted */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, !PIN_getOutputValue(Board_PIN_LED0));
    }

    /* Fill packet with encrypted data */
    fillPacket(&txPacket, messageBuffer, (char *)nonce, 128);

    status = EasyLink_transmit(&txPacket);
    if(status == EasyLink_Status_Success){
        /* Toggle led to indicate a packet has been successfully transmitted, otherwise ignore */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, !PIN_getOutputValue(Board_PIN_LED1));
    }

}



typedef enum
{
    RADIO_TEST_NOP,             /**< No test running.                                */
    RADIO_TEST_TXCC,            /**< TX constant carrier.                            */
    RADIO_TEST_TXCC_CCM,        /**< TX constant carrier with encryption.            */
    RADIO_TEST_TXCC_CCM_DELAY,  /**< TX constant carrier with encryption and delays. */
    RADIO_TEST_TXMC,            /**< TX modulated carrier.                           */
    RADIO_TEST_TXMC_CCM,        /**< TX modulated carrier with encryption.           */
    RADIO_TEST_TXSWEEP,         /**< TX sweep.                                       */
    RADIO_TEST_RXC,             /**< RX constant carrier.                            */
    RADIO_TEST_RXSWEEP,         /**< RX sweep.                                       */
    RADIO_TEST_NOISYOP,         /**< Some noisy op.                                  */
} radio_tests_t;

typedef struct {
    bool sweep;
    bool ccm;
} main_state_t;


UART_Handle _uart;


/** @brief Function for configuring all peripherals used in this example.
*/
static void init() {
    /* Call driver init functions */
    GPIO_init();
    UART_init();

    UART_Params uartParams;
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    _uart = UART_open(Board_UART0, &uartParams);
    if(_uart == NULL)
        { while(1); }

    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success){
        while(1);
    }

}

static void cleanup() {
    if(_uart) {
        UART_readCancel(_uart);
        UART_writeCancel(_uart);
        UART_close(_uart);
        _uart=NULL;
    }
}

static void printMessage(char* message, uint16_t messagelevel) {
    if(DEBUGLEVEL >= messagelevel ) {
        UART_write(_uart, message, strlen(message));
        UART_write(_uart, "\r\n", 2);
    }
}

void errorOut(char* message) {
    printMessage(message, 0);
}

void UART_read_string(char* string, uint16_t maxSize){
    char input = '\0';
    uint16_t curSize = 0;
    do {
        UART_read(_uart, &input, 1);
        if(input != '\r' && input != '\n' && input != '\0') {
            UART_write(_uart, &input, 1);
            string[curSize] = input;
            curSize = curSize + 1;
        }
    } while (input != '\r' && input != '\n' && input != '\0' && curSize < maxSize);
    string[curSize] = '\0';
    UART_write(_uart, "\r\n", 2);
}

/** @brief Function for outputting usage info to the serial port.
*/
static void help(void)
{
    printMessage("Usage:",0);
    printMessage("a: [NOT IMPLEMENTED] Enter start channel for sweep/channel for constant carrier",0);
    printMessage("b: [NOT IMPLEMENTED] Enter end channel for sweep",0);
    printMessage("c: Start unmodulated TX carrier",0);
    printMessage("d: [NOT IMPLEMENTED] Enter time on each channel (1ms-99ms)",0);
    printMessage("e: Cancel sweep/carrier",0);
    printMessage("f: [NOT IMPLEMENTED]  Toggle CCM power",0);
    printMessage("g: [NOT IMPLEMENTED]  Change CCM counter",0);
    printMessage("i: [NOT IMPLEMENTED]  Start (unmodulated) TX carrier with active CCM",0);
    printMessage("j: [NOT IMPLEMENTED]  Start (unmodulated) TX carrier with active CCM and delays",0);
    printMessage("l: [NOT IMPLEMENTED]  Test the crypto hardware",0);
    printMessage("m: [NOT IMPLEMENTED]  Enter data rate",0);
    printMessage("o: Start modulated TX carrier",0);
    printMessage("p: Enter output power",0);
    printMessage("q: Start modulated TX carrier with encryption",0);
    printMessage("s: Print current delay, channels and so on",0);
    printMessage("r: Start RX sweep",0);
    printMessage("t: Start TX sweep",0);
    printMessage("x: Start RX carrier",0);
    printMessage("y: Start noisy operation",0);
    printMessage("z: End noisy operation",0);
    printMessage("n: Enter tiny_aes_128 mode",0);
    printMessage("   p: Enter plaintext",0);
    printMessage("   k: Enter key",0);
    printMessage("   e: Encrypt",0);
    printMessage("   n: Set number of repetitions",0);
    printMessage("   r: Run repeated encryption",0);
    printMessage("   q: Quit tiny_aes_128 mode",0);
    printMessage("v: Enter simplified power analysis mode",0);
    printMessage("   m: Enter switching mask",0);
    printMessage("   s: Switch",0);
    printMessage("   q: Quit power analysis mode",0);
    printMessage("u: Enter hwcrypto mode",0);
    printMessage("   p: Enter plaintext",0);
    printMessage("   k: Enter key",0);
    printMessage("   e: Encrypt",0);
    printMessage("   o: Print encrypted ciphertext",0);
    printMessage("   q: Quit hwcrypto mode",0);
    printMessage("n: Enter hwcrypto ECB mode",0);
    printMessage("   p: Enter plaintext",0);
    printMessage("   k: Enter key",0);
    printMessage("   e: Encrypt",0);
    printMessage("   n: Set number of repetitions",0);
    printMessage("   r: Run repeated encryption",0);
    printMessage("   q: Quit hwcrypto ECB mode",0);
    printMessage("w: Enter aes_masked mode",0);
    printMessage("   0: Set mask mode to UNMASKED",0);
    printMessage("   1: Set mask mode to RIVAIN-PROUFF",0);
    printMessage("   2: Set mask mode to RIVAIN-PROUFF-SHARED",0);
    printMessage("   3: Set mask mode to RAND-TABLE",0);
    printMessage("   4: Set mask mode to RAND-TABLE-INC",0);
    printMessage("   5: Set mask mode to RAND-TABLE-WORD",0);
    printMessage("   6: Set mask mode to RAND-TABLE-WORD-INC",0);
    printMessage("   7: Set mask mode to RAND-TABLE-SHARED",0);
    printMessage("   8: Set mask mode to RAND-TABLE-SHARED-WORD",0);
    printMessage("   9: Set mask mode to RAND-TABLE-SHARED-WORD-INC",0);
    printMessage("   p: Enter plaintext",0);
    printMessage("   k: Enter key",0);
    printMessage("   e: Encrypt",0);
    printMessage("   o: Print encrypted ciphertext",0);
    printMessage("   n: Set number of repetitions",0);
    printMessage("   r: Run repeated encryption",0);
    printMessage("   q: Quit aes_masked mode",0);
}

void *mainThread(void *arg0)
{
    uint32_t err_code;
    radio_tests_t test     = RADIO_TEST_NOP;
    radio_tests_t cur_test = RADIO_TEST_NOP;
    main_state_t state     = {false, false};

    init();

    char        input = '\0';

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    printMessage("Whispering Electronics Firmware Loaded", 0);


    /* Loop forever echoing */
    char messageBuffer[255];
    while (1) {
        printMessage("Press key to select usage or press 'h' for usage information", 0);
        UART_read(_uart, &input, 1);
        UART_write(_uart, &input, 1);
        UART_write(_uart, "\r\n", 2);


        switch (input){
            case 'd':
                printMessage("WARNING: Not implemented", 0);
                //This option was used to specify the BlueTooth channel. ScreamingChannels focused on BlueTooth but this firmware considers other wireless protocols
                test = cur_test;
                break;
            case 'k': //this is to replace the "channel" option used in ScreaminChannels with a parameter for target frequency
                //uint freqInt = ; //convert Hz to MHz
                printMessage("Current frequency (MHz):", 100);
                printMessage(itoa(division_rounded(EasyLink_getFrequency(), (1000*1000)), messageBuffer, 10), 100);
                printMessage("Enter 3-4 digits for the frequency (in MHz) in the range 287-351, 359-439, 431-527, 718-878, or 861-1054:", 0);
                UART_read_string(messageBuffer, 5);
                frequency = atoi(messageBuffer); //TODO: should make sure that freq is in a supported range
                if(EasyLink_setFrequency(frequency * (1000*1000)) != EasyLink_Status_Success )  //Multiplication converts Hz to MHz
                    { errorOut("Unable to set frequency"); }
                break;
            case 'c':
                printMessage("Starting unmodulated TX", 100);
                EasyLink_setCtrl(EasyLink_Ctrl_Test_Tone, 0);
                test = RADIO_TEST_TXCC;
                break;
            case 'e':
                printMessage("Stopping current tests", 100);
                EasyLink_abort();
                cur_test = RADIO_TEST_NOP;
                break;
/*
            case 'a':
            case 'b':
            case 'f':
                NRF_CCM->ENABLE = NRF_CCM->ENABLE ? CCM_ENABLE_ENABLE_Disabled : CCM_ENABLE_ENABLE_Enabled;
            case 'g':
                ccm_config.counter[0]++;
            case 'i':
                test = RADIO_TEST_TXCC_CCM;
            case 'j':
                test = RADIO_TEST_TXCC_CCM_DELAY;
            case 'l':
                if (cur_test == RADIO_TEST_NOP && test == RADIO_TEST_NOP)
                    ccm_test_crypto();
            case 'm':
                get_datarate();
                test = cur_test;
            case 'w':
                aes_masked_mode();
            case 'n':
                tiny_aes_128_mode();
*/
            case 'o':
                EasyLink_setCtrl(EasyLink_Ctrl_Test_Signal, 0);
                test = RADIO_TEST_TXMC; //TX modulated carrier
                break;
            case 'p':
                EasyLink_getRfPower(&txPower);
                printMessage("Current transmission power (dBm): ", 100);
                printMessage(itoa(txPower, messageBuffer, 10), 100);
                test = cur_test;
                printMessage("Enter transmission power (dBm) in the range -10 to 14", 0);
                UART_read_string(messageBuffer, 4);
                EasyLink_setRfPower(atoi(messageBuffer));
                break;
/*            case 'q':
                test = RADIO_TEST_TXMC_CCM; //TX modulated carrier with encryption
                break;
            case 'r':
                test = RADIO_TEST_RXSWEEP; //RX Sweep
            case 's':
                print_parameters();
            case 't':
                test = RADIO_TEST_TXSWEEP; //TX Sweep
            case 'u':
                hwcrypto_mode();
            case 'U':
                hwcrypto_ecb_mode();
            case 'v':
                power_analysis_mode();
            case 'x':
                test = RADIO_TEST_RXC; //RX constant carrier
            case 'y':
                test = RADIO_TEST_NOISYOP; //Exercise the processor
            case 'z':
                stop_noisy_op();
                test = RADIO_TEST_NOP; //Relax the processor
                */
            case 'h':
                help();
        }
        input = '\0';
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
        sleep(1);
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
        sleep(1);
    }
















//    printf("RF Test\r\n");
//    NVIC_EnableIRQ(TIMER0_IRQn);
//    NVIC_EnableIRQ(TIMER1_IRQn);
//    NVIC_EnableIRQ(CCM_AAR_IRQn);











/*
    __enable_irq();
    while (true){
        scanf("%c",&input);
        switch (input){
            case 'a':
                scanf("%d",&channel_start_);
                test = cur_test;
            case 'b':
                scanf("%d",&channel_end_);
                test = cur_test;
            case 'c':
                test = RADIO_TEST_TXCC;
            case 'd':
                scanf("%d",&delayms_);
                test = cur_test;
            case 'e':
                radio_sweep_end();
                ccm_disable();
                cur_test = RADIO_TEST_NOP;
            case 'f':
                NRF_CCM->ENABLE = NRF_CCM->ENABLE ? CCM_ENABLE_ENABLE_Disabled : CCM_ENABLE_ENABLE_Enabled;
            case 'g':
                ccm_config.counter[0]++;
            case 'i':
                test = RADIO_TEST_TXCC_CCM;
            case 'j':
                test = RADIO_TEST_TXCC_CCM_DELAY;
            case 'l':
                if (cur_test == RADIO_TEST_NOP && test == RADIO_TEST_NOP)
                    ccm_test_crypto();
            case 'm':
                get_datarate();
                test = cur_test;
            case 'w':
                aes_masked_mode();
            case 'n':
                tiny_aes_128_mode();
            case 'o':
                test = RADIO_TEST_TXMC; //TX modulated carrier
            case 'p':
                get_power();
                test = cur_test;
            case 'q':
                test = RADIO_TEST_TXMC_CCM; //TX modulated carrier with encryption
            case 'r':
                test = RADIO_TEST_RXSWEEP; //RX Sweep
            case 's':
                print_parameters();
            case 't':
                test = RADIO_TEST_TXSWEEP; //TX Sweep
            case 'u':
                hwcrypto_mode();
            case 'U':
                hwcrypto_ecb_mode();
            case 'v':
                power_analysis_mode();
            case 'x':
                test = RADIO_TEST_RXC; //RX constant carrier
            case 'y':
                test = RADIO_TEST_NOISYOP; //Exercise the processor
            case 'z':
                stop_noisy_op();
                test = RADIO_TEST_NOP; //Relax the processor
            case 'h':
                help();
        }

        switch (test){
            case RADIO_TEST_TXCC:
                all_off(&state);
                radio_tx_carrier(txpower_, mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_TXCC_CCM:
                all_off(&state);
                state.ccm = true;
                ccm_radio_tx(txpower_, mode_, channel_start_, CCM_RADIO_TX_CW);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_TXCC_CCM_DELAY:
                all_off(&state);
                state.ccm = true;
                ccm_radio_tx(txpower_, mode_, channel_start_, CCM_RADIO_TX_CW_WITH_DELAY);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_TXMC:
                all_off(&state);
                radio_modulated_tx_carrier(txpower_, mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_TXMC_CCM:
                all_off(&state);
                state.ccm = true;
                ccm_radio_tx(txpower_, mode_, channel_start_, CCM_RADIO_TX_TRANSMIT_RESULT);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_TXSWEEP:
                all_off(&state);
                radio_tx_sweep_start(txpower_, mode_, channel_start_, channel_end_, delayms_);
                state.sweep = true;
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_RXC:
                all_off(&state);
                radio_rx_carrier(mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_RXSWEEP:
                radio_rx_sweep_start(mode_, channel_start_, channel_end_, delayms_);
                state.sweep = true;
                cur_test = test;
                test     = RADIO_TEST_NOP;
            case RADIO_TEST_NOISYOP:
                timer1_init();
                start_noisy_op();
                state.sweep    = false;
                cur_test = test;
                test     = RADIO_TEST_NOP;
        }
    }
*/
    cleanup();
}
