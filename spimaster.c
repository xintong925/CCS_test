/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/*
 *  ======== spimaster.c ========
 */

#include <inttypes.h>


/* Example/Board Header files */
#include "Board.h"


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/PIN.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <ti/drivers/PWM.h>

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>
#include <ti/devices/cc13x0/driverlib/gpio.h>

/* IMU */
#include "Adafruit_LSM6DS_new.h"

/* Example/Board Header files */
#include "smartrf_settings/smartrf_settings.h" // generated from smartrf application: 500 GB/s -> 915 MHz -> infinite 3512 code export; save; in the smartrf folder
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
/* Packet TX Configuration */
#define PAYLOAD_LENGTH      20  //remember to update for multi packets! lengh of tx buffer, larger than data sending; one packet

void send_databuffer(const void* buffer,int buffer_size);
int Communicate_IMU(void);
//void send_SPI_to_IMU(uint8_t *tx_buffer, uint8_t *rx_buffer, int word_count);


int transmit_imu_data(const uint64_t NUM_SAMPLES);

int init_SPI_IMU(void);
int SPI_write_data(void);

int Standby_MCU(void);
int32_t platform_read(void *handle, uint16_t reg, uint8_t *bufp, uint16_t len);
int32_t platform_write(void *handle, uint16_t reg, uint16_t data, uint16_t len);
int32_t platform_read_multiple(void *handle, uint16_t reg, uint8_t *bufp, uint16_t len);

uint32_t Temperature_raw_get(void *handle, uint16_t reg, uint16_t len);
int32_t Acceleration_raw_get(void *handle, uint16_t reg, uint16_t len);
int32_t Angular_Rate_raw_get(void *handle, uint16_t reg, uint16_t len);

void Tap_Detection(void *handle, uint16_t reg, uint16_t len);
int Activity_Detection(void *handle, uint16_t reg, uint16_t len);
void wakeUpCallback(void);
uint16_t concatenateHexNumbers(uint16_t num1, uint16_t num2);


#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (30)
//#define MASTER_MSG      ("Hello from master, msg#: ")

#define MAX_LOOP        (10)
#define SEND_DEBUG_INFO (0)
#define SEND_DATA (1)

static uint32_t raw_temp;
static uint32_t raw_accel[3];
static uint32_t raw_angular[3];
static float accel_g[3];
static float angular_mdps[3];




/***** Variable declarations for RF*****/
RF_Object rfObject;
RF_Handle rfHandle;
uint8_t packet[PAYLOAD_LENGTH+2];

static Display_Handle display;
static uint16_t seqNumber;

int16_t masterRxBuffer[SPI_MSG_LENGTH];
int16_t masterTxBuffer[SPI_MSG_LENGTH];
SPI_Transaction transaction;
SPI_Handle      masterSpi;
SPI_Params      spiParams;
uint8_t         data_ready = 0;

/* Semaphore to block master until slave is ready for transfer */
sem_t masterSem;

/* Led pin table */
PIN_Config LedPinTable[] =
{
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    PIN_TERMINATE                                                                      /* Terminate list */
};

/*
 *  ======== slaveReadyFxn ========
 *  Callback function for the GPIO interrupt on Board_SPI_SLAVE_READY.
 */
void slaveReadyFxn(uint_least8_t index)
{
    data_ready = 1;
    //sem_post(&masterSem);
}

void send_databuffer(const void* buffer, int buffer_size) // buffer: size of the data element is undefined
{

    RF_cmdPropTx.pktLen = buffer_size+2;

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    //uint8_t j;
    //for (j = 0; j < buffer_size/PAYLOAD_LENGTH; ++j)
    //{
        /* Create packet with incrementing sequence number and random payload */
        uint8_t *a=(((uint8_t*)buffer)/*+(PAYLOAD_LENGTH)*j*/); //rf always work in 8-bit packet, buffer[0] -> a[0] & a[1], 16 - into half
        packet[0] = (uint8_t)(seqNumber >> 8); //shift 8 bits, upper 8 bits
        packet[1] = (uint8_t)(seqNumber++); // lower 8 bits

        uint8_t i;
        for (i = 2; i < buffer_size+2 /*PAYLOAD_LENGTH+2*/; i++) // extra 2 bits for the length
        {
            packet[i]=a[i-2]; // load the whole array
            //packet[i] = rand();
//            packet[i] = ((uint8_t*)buffer)+(PAYLOAD_LENGTH)*j+i /* check pointer math */
        }

        /* Send packet */ // 01 23 - 23 01// data flipped
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

        /* Power down the radio */
        RF_yield(rfHandle);
}


/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
//e.g. platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy, 1);
int32_t platform_read(void *handle, uint16_t reg, uint8_t *bufp, uint16_t len)
{
    int32_t ret;
    int16_t read_bit = 0x8000;
    reg |= read_bit;

    uint8_t* tx_Address = &reg; /*Address of the register to write to IMU */

    transaction.count = len;
    transaction.txBuf = tx_Address;/*!< void * to a buffer with data to be transmitted */
    transaction.rxBuf = bufp;/*< void * to a buffer to receive data */

    bool spitransferOK;

    spitransferOK = SPI_transfer(handle, &transaction);

    if (spitransferOK) {
      //  printf("SPI transfer successful\n");
      //  printf("Transmitted: 0x%04X\n", reg);

    }
    else {
        printf("SPI transfer failed\n");
        printf("Transmitted: 0x%04X\n", reg);
        while(1);
    }

    ret = *bufp;
    printf("Received: 0x%04X\n", ret);

    return ret;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  data      data to write to reg
 * @param  len       number of consecutive register to write
 *
 */

//e.g. platform_write(masterSpi, LSM6DSOX_CTRL1_XL, data_to_write, 1);
int32_t platform_write(void *handle, uint16_t reg, uint16_t data, uint16_t len)
{
    int32_t ret;
    bool spitransferOK_message;

    uint16_t tx_Data = reg|data;
    uint16_t* tx_Address = &tx_Data; /*Address of the combined data of register & data to be transferred */

    transaction.count = len;
    transaction.txBuf = tx_Address;/*!< void * to a buffer with data to be transmitted */

    spitransferOK_message = SPI_transfer(handle, &transaction);

    if (spitransferOK_message) {
     //   printf("SPI transfer successful\n");
     //   printf("Transmitted: 0x%04X\n", tx_Data);
    }
    else {
        printf("SPI transfer failed\n");
        printf("Transmitted: 0x%04X\n", reg);
        while(1);
    }

    ret = tx_Data;

    return ret;
}


/**
  * @brief  Temperature data output register (r).
  *         L and H registers together express a 16-bit word in two's
  *         complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval             interface status (MANDATORY: return 0 -> no Error)
  *
  */
/*
uint32_t Temperature_raw_get(void *handle, uint16_t reg, uint16_t len) {

    int32_t ret;
    int32_t retL;
    int32_t retH;

    uint16_t dummy_temp;
    uint16_t temp_bit = 0x0004;
    uint8_t temp_buff[2];

    printf("Temperature buffer 1 before: 0x%02X\n", temp_buff[0]);
    printf("Temperature buffer 2  before: 0x%02X\n", temp_buff[1]);

    ret = platform_read(handle, reg, &dummy_temp, len);

    bool check_temp_aval = ((ret&temp_bit) == temp_bit); // if true, the temp data is ready

    printf("Is it ready to read T data? %d\n", check_temp_aval);

    if(check_temp_aval) {

        retL = platform_read(masterSpi, LSM6DSOX_OUT_TEMP_L, temp_buff, 1);
        retH = platform_read(masterSpi, LSM6DSOX_OUT_TEMP_H, temp_buff, 1);


        printf("temp L after: 0x%02X\n", retL);
        printf("temp H after: 0x%02X\n", retH);

        retH <<= 8;
        raw_temp = retL | retH;

        printf("Received temperature: 0x%X\n", raw_temp);

        float converted_temp = ((float_t)raw_temp / 256.0f) + 25.0f;
        printf("Measured T is %f\n", converted_temp);

    }
    return raw_temp;
}

*/

int32_t Acceleration_raw_get(void *handle, uint16_t reg, uint16_t len) {

    int32_t ret;

    uint16_t data_XL;
    uint16_t data_XH;
    uint16_t data_YL;
    uint16_t data_YH;
    uint16_t data_ZL;
    uint16_t data_ZH;

    uint16_t dummy_XL;
    uint16_t XL_bit = 0x0001;

    uint8_t XL_buff_X[2];
    uint8_t XL_buff_Y[2];
    uint8_t XL_buff_Z[2];

    ret = platform_read(handle, reg, &dummy_XL, len); // Check OUTX_L_A to see if data is ready

    bool check_XL_aval = ((ret&XL_bit) == XL_bit); // if true, the temp data is ready

    printf("Accelerometer data ready? %d\n", check_XL_aval);

    if(check_XL_aval) {

        data_XL = platform_read(masterSpi, LSM6DSOX_OUTX_L_A, XL_buff_X, 1);
        data_XH = platform_read(masterSpi, LSM6DSOX_OUTX_H_A, XL_buff_X, 1);
        data_YL = platform_read(masterSpi, LSM6DSOX_OUTY_L_A, XL_buff_Y, 1);
        data_YH = platform_read(masterSpi, LSM6DSOX_OUTY_H_A, XL_buff_Y, 1);
        data_ZL = platform_read(masterSpi, LSM6DSOX_OUTZ_L_A, XL_buff_Z, 1);
        data_ZH = platform_read(masterSpi, LSM6DSOX_OUTZ_H_A, XL_buff_Z, 1);

        data_XH <<= 8;
        data_YH <<= 8;
        data_ZH <<= 8;

        raw_accel[0] = data_XL | data_XH;
        raw_accel[1] = data_YL | data_YH;
        raw_accel[2] = data_ZL | data_ZH;

        accel_g[0] = ((float_t)raw_accel[0]) * 0.061f/1000; // Refer Adafruit_LSM6DS.cpp
        accel_g[1] = ((float_t)raw_accel[1]) * 0.061f/1000;
        accel_g[2] = ((float_t)raw_accel[2]) * 0.061f/1000;

        printf("Acceleration [g]:%4.2f\t%4.2f\t%4.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);


   }
    return raw_accel;
}


int32_t Angular_Rate_raw_get(void *handle, uint16_t reg, uint16_t len) {

    int32_t ret;

    uint16_t data_XL;
    uint16_t data_XH;
    uint16_t data_YL;
    uint16_t data_YH;
    uint16_t data_ZL;
    uint16_t data_ZH;

    uint16_t dummy_G;
    uint16_t G_bit = 0x0002;

    uint8_t G_buff[6];


    ret = platform_read(handle, reg, &dummy_G, len); // Check OUTX_L_G to see if data is ready

    bool check_G_aval = ((ret&G_bit) == G_bit); // if true, the data is ready

    printf("Gyro data ready? %d\n", check_G_aval);

    if(check_G_aval) {

        data_XL = platform_read(masterSpi, LSM6DSOX_OUTX_L_G, G_buff, 1);
        data_XH = platform_read(masterSpi, LSM6DSOX_OUTX_H_G, G_buff, 1);
        data_YL = platform_read(masterSpi, LSM6DSOX_OUTY_L_G, G_buff, 1);
        data_YH = platform_read(masterSpi, LSM6DSOX_OUTY_H_G, G_buff, 1);
        data_ZL = platform_read(masterSpi, LSM6DSOX_OUTZ_L_G, G_buff, 1);
        data_ZH = platform_read(masterSpi, LSM6DSOX_OUTZ_H_G, G_buff, 1);

        data_XH <<= 8;
        data_YH <<= 8;
        data_ZH <<= 8;

        raw_angular[0] = data_XL | data_XH;
        raw_angular[1] = data_YL | data_YH;
        raw_angular[2] = data_ZL | data_ZH;

        angular_mdps[0] = ((float_t)raw_angular[0]) * 35.0f/1000;
        angular_mdps[1] = ((float_t)raw_angular[1]) * 35.0f/1000;
        angular_mdps[2] = ((float_t)raw_angular[2]) * 35.0f/1000;

        printf("Angular rate [dps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_mdps[0], angular_mdps[1], angular_mdps[2]);

   }
    return raw_angular;
}


/* reg: LSM6DSOX_ALL_INT_SRC */
/* Tap detection
void Tap_Detection(void *handle, uint16_t reg, uint16_t len) {

    int32_t ret;

    uint16_t dummy_tap;
    uint16_t single_tap_bit = 0x0004;
    uint16_t double_tap_bit = 0x0008;

    ret = platform_read(handle, reg, &dummy_tap, len); // Check OUTX_L_G to see if data is ready

    bool check_single_tap = ((ret&single_tap_bit) == single_tap_bit); // if true, the temp data is ready
    bool check_double_tap = ((ret&double_tap_bit) == double_tap_bit); // if true, the temp data is ready

    if(check_single_tap) {
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!! Single tap detected !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }

    if(check_double_tap) {
        printf("************************** Double tap detected ********************************\n");
    }

}
*/

int Activity_Detection(void *handle, uint16_t reg, uint16_t len) {

    int32_t ret;

    int data_status;

    uint16_t dummy_act;
    uint16_t activity_bit = 0x0010;


    ret = platform_read(handle, reg, &dummy_act, len);

    bool check_activity = ((ret&activity_bit) == activity_bit); // if true, there's change in activity status

    if(check_activity) {
        printf("zzzzzzzzzzzzzzzzzzzzzzz Sleeping zzzzzzzzzzzzzzzzzzzzzzzzzz\n");
        data_status = 0;
    }
    else{
        printf("!!!!!!!!!!!!!!!!!!!! Active !!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        data_status = 1;
    }
    printf("status is %d\n", data_status);

    return data_status;

}

// Function to introduce a delay in microseconds

int init_SPI_IMU(void) {
//
    /* Enable G sleep mode and XL ultra-low power mode */
 //   uint16_t CTRL4_C_data_to_write = 0x0000; // CTRL4_C (13h): data to write 0b01000000 [Gyro sleep mode: enabled] (once enabled, no data is available for Gyro)
//    uint16_t CTRL5_C_data_to_write = 0x0080; // CTRL5_C (14h): data to write 0b10000000 [XL ultra-low-power mode enable: enabled]
//
//    /* Disable I3C */
//    uint16_t CTRL9_XL_data_to_write = 0x00E2; // CTRL9_XL (18h): data to write 0b11100010 [I3C disabled]
//

        /* Routing of functions including activity/sleep, tap and wake up on INT1 and INT2 */
    //    uint16_t MD1_CFG_data_to_write = 0x00C0; // MD1_CFG (5Eh): 0b11000000 routing of activity/inactivity, single tap recognition event on INT1 enabled
    //    uint16_t MD2_CFG_data_to_write = 0x0028; // MD2_CFG (5Fh): 0b00101000 routing of wake-up and double tap event on INT2 enabled
//
//    /* Function access setting */
//    uint16_t FUNC_CFG_ACCESS_data_to_write = 0x0000; //FUNC_CFG_ACCESS (01h): bit[1:5] must be set to 0 for the correct operation of the device, the rest unsure
//
    /* INT pin configuration */
//    uint16_t INT1_CTRL_data_to_write = 0x0003; //INT1_CTRL (0Dh): enables gyro and accel data-ready interrupt on INT1 pin.
 //   uint16_t INT2_CTRL_data_to_write = 0x0003; //INT2_CTRL (0Eh): enables gyro and accel data-ready interrupt on INT2 pin.
//

    // CTRL6_C, CTRL7_G, CTRL8_XL: ask

       /* Initialize SPI parameters */

    SPI_Params_init(&spiParams);            //spiParams is a global (TODO change eventually)
    spiParams.frameFormat = SPI_POL1_PHA1; // Mode 3
    spiParams.bitRate = 1000000;
    //spiParams.mode = SPI_MASTER;
    spiParams.dataSize = 16;
    spiParams.transferMode = SPI_MODE_BLOCKING;
        //GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

        /* Open SPI */
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);    //masterSPI is a global (TODO change eventually)
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        printf("Error initializing master SPI\n");
        while (1);
    }
    else {
        printf("Master SPI initialized\n");
    }

//    /* Write data to register */
//    /* Set ODR and scale for XL & G */
 //   int32_t new_data_XL = platform_write(masterSpi, LSM6DSOX_CTRL1_XL, 0x0050, 1);
 //   int32_t new_data_G = platform_write(masterSpi, LSM6DSOX_CTRL2_G, 0x0048, 1);
 //   int32_t WakeUpDur = platform_write(masterSpi, LSM6DSOX_WAKE_UP_DUR,  0x0064, 1);
 //   int32_t WakeUpTHS = platform_write(masterSpi, LSM6DSOX_WAKE_UP_THS, 0x0002, 1);

//
//    int32_t Func_Access = platform_write(masterSpi, LSM6DSOX_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_data_to_write, 1);
//
//        /* Enable G to sleep */
//    //    int32_t SleepEnable_G = platform_write(masterSpi, LSM6DSOX_CTRL4_C, CTRL4_C_data_to_write, 1);
//    int32_t LowPower_Enable_XL = platform_write(masterSpi, LSM6DSOX_CTRL5_C, CTRL5_C_data_to_write, 1);
//
//        /* Enable interrupt and tap recognition */
//    int32_t Tap_Enable = platform_write(masterSpi, LSM6DSOX_TAP_CFG0, 0x0000, 1);
//    int32_t InterruptEnable = platform_write(masterSpi, LSM6DSOX_TAP_CFG2, 0x00E0, 1);
//    int32_t Tap_Enable = platform_write(masterSpi, LSM6DSOX_TAP_CFG0, 0x0000, 1);
//     //   int32_t Tap_yThre = platform_write(masterSpi, LSM6DSOX_TAP_CFG1, TAP_CFG1_data_to_write, 1);
//     //   int32_t Tap_zThre = platform_write(masterSpi, LSM6DSOX_TAP_THS_6D, TAP_THS_6D_data_to_write, 1);
//     //   int32_t Tap_Dur = platform_write(masterSpi, LSM6DSOX_INT_DUR2, INT_DUR2_data_to_write, 1);

//
//
//    int32_t I3C_Disable = platform_write(masterSpi, LSM6DSOX_CTRL9_XL, CTRL9_XL_data_to_write, 1);
//    int32_t INT1_Routing = platform_write(masterSpi, LSM6DSOX_MD1_CFG, 0x0080, 1);
//      //  int32_t INT2_Routing = platform_write(masterSpi, LSM6DSOX_MD2_CFG, MD2_CFG_data_to_write, 1);
//    int32_t INT1_Control = platform_write(masterSpi, LSM6DSOX_INT1_CTRL, INT1_CTRL_data_to_write, 1);
//     //   int32_t INT2_Control = platform_write(masterSpi, LSM6DSOX_INT2_CTRL, INT2_CTRL_data_to_write, 1);



        /* Read data from register */
     //   int32_t rx_Data_XL = platform_read(masterSpi, LSM6DSOX_CTRL7_G, &dummy_read_XL, 1);
      //  int32_t rx_Data_G = platform_read(masterSpi, LSM6DSOX_CTRL5_C, &dummy_read_G, 1);
      //  int32_t rx_SleepEnable_G = platform_read(masterSpi, LSM6DSOX_CTRL4_C, &dummy_read_G, 1);
      //  int32_t rx_WakeUp = platform_read(masterSpi, LSM6DSOX_CTRL1_XL, &dummy_read_G, 1);
     //   int32_t rx_InterEnable = platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy_read_G, 1);

        /* Enable interrupt pin */
    GPIO_setConfig(Board_DIO12, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
 //   GPIO_setCallback(Board_DIO12, slaveReadyFxn);
    GPIO_enableInt(Board_DIO12);  /* INT1 */

    GPIO_setConfig(Board_DIO15, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_enableInt(Board_DIO15);  /* INT2 */

    return 0;

}

int SPI_write_data(void) {
    int32_t new_data_XL = platform_write(masterSpi, LSM6DSOX_CTRL1_XL, 0x0050, 1);
    int32_t new_data_G = platform_write(masterSpi, LSM6DSOX_CTRL2_G, 0x0048, 1);
    int32_t WakeUpDur = platform_write(masterSpi, LSM6DSOX_WAKE_UP_DUR,  0x0068, 1); // active time set to F - 37s
    int32_t WakeUpTHS = platform_write(masterSpi, LSM6DSOX_WAKE_UP_THS, 0x0002, 1);
           /* Enable interrupt and tap recognition */
    int32_t Tap_Enable = platform_write(masterSpi, LSM6DSOX_TAP_CFG0, 0x0000, 1);
    int32_t InterruptEnable = platform_write(masterSpi, LSM6DSOX_TAP_CFG2, 0x00E0, 1);
    int32_t INT1_Routing = platform_write(masterSpi, LSM6DSOX_MD1_CFG, 0x0080, 1);

    printf("SPI initialized successfully and IMU has been waken up\n");

    return 0;

}

int Standby_MCU(void) {
    printf("Entering sleeping mode! x1");
    SPI_close(masterSpi);
    printf("Entering sleeping mode! x2");
    sleep(3);

    printf("Entering sleeping mode! x3");
    init_SPI_IMU();
    printf("Waking up!");

    return 0;
}

//void intCallbackFxn(PIN_Handle handle, PIN_Id pinId)
void wakeUpCallback(void)
{
    // Add your wake-up tasks or data processing here
    // For example, toggle an LED to indicate wake-up
    printf("MCU is waken up!");
}



int Communicate_IMU(void) {

    uint8_t dummy_read_XL; //read data - to verify if transmit successfully
    uint8_t dummy_read_G;

    int check_status;

    /* Turn on XL and Gyro */
    uint16_t CTRL1_XL_data_to_write = 0x0050; // CTRL1_XL (10h): data to write 0b00000101 [High performance; ODR: 208Hz; Full scale selection: 2g]
    uint16_t CTRL2_G_data_to_write = 0x0048; // CTRL2_G (11h): data to write 0b00011000 [Hi performance; ODR: 104Hz; Gyro UI chain full-scale selction: 1000dps]

    /* Set duration for Inactivity detection */
    /* Select Activity/Inactivity threshold resolution and duration */
    uint16_t WAKE_UP_DUR_data_to_write = 0x0064; // WAKE_UP_DUR (5Ch): 0b00000100 [wake_dur: 14.4 ms; sleep_dur: 9.8 s - if no activity, enter sleeping mode].

    /* Set Activity/Inactivity threshold resolution and duration */
    uint16_t WAKE_UP_THS_data_to_write = 0x0002; // both single and double are enabled; Activity threshold is 1.5g.

    /* Select sleep-change notification */
    /* Select slope filter */
    uint16_t TAP_CFG0_data_to_write = 0x0000; // TAP_CFG0 (56h): 0b00000000; Slope filter applied.

    /* Enable interrupt */
    /* Inactivity configuration: accelerometer to 12.5Hz, Gyro to power down */
    uint16_t TAP_CFG2_data_to_write = 0x00E0; //  old: C9 TAP_CFG2 (58h): 0b11100000; [XL ODR 12.5 Hz; G power down] - enables interrupt and inactivity functions

    /* Activity/Inactivity interrupt driven in INT1 pin */
    uint16_t MD1_CFG_data_to_write = 0x0080; // MD1_CFG (5Eh): 0b10000000 routing of activity/inactivity to INT1 enabled

    /* Enable G sleep mode and XL ultra-low power mode */
    uint16_t CTRL4_C_data_to_write = 0x0000; // CTRL4_C (13h): data to write 0b01000000 [Gyro sleep mode: enabled] (once enabled, no data is available for Gyro)
    uint16_t CTRL5_C_data_to_write = 0x0080; // CTRL5_C (14h): data to write 0b10000000 [XL ultra-low-power mode enable: enabled]

    /* Disable I3C */
    uint16_t CTRL9_XL_data_to_write = 0x00E2; // CTRL9_XL (18h): data to write 0b11100010 [I3C disabled]


    /* Routing of functions including activity/sleep, tap and wake up on INT1 and INT2 */
//    uint16_t MD1_CFG_data_to_write = 0x00C0; // MD1_CFG (5Eh): 0b11000000 routing of activity/inactivity, single tap recognition event on INT1 enabled
//    uint16_t MD2_CFG_data_to_write = 0x0028; // MD2_CFG (5Fh): 0b00101000 routing of wake-up and double tap event on INT2 enabled

    /* Function access setting */
    uint16_t FUNC_CFG_ACCESS_data_to_write = 0x0000; //FUNC_CFG_ACCESS (01h): bit[1:5] must be set to 0 for the correct operation of the device, the rest unsure

    /* INT pin configuration */
    uint16_t INT1_CTRL_data_to_write = 0x0003; //INT1_CTRL (0Dh): enables gyro and accel data-ready interrupt on INT1 pin.
    uint16_t INT2_CTRL_data_to_write = 0x0003; //INT2_CTRL (0Eh): enables gyro and accel data-ready interrupt on INT2 pin.


// CTRL6_C, CTRL7_G, CTRL8_XL: ask

   /* Initialize SPI parameters */

    SPI_Params_init(&spiParams);            //spiParams is a global (TODO change eventually)
    spiParams.frameFormat = SPI_POL1_PHA1; // Mode 3
    spiParams.bitRate = 1000000;
    spiParams.mode = SPI_MASTER;
    spiParams.dataSize = 16;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    //GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

    /* Open SPI */
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);    //masterSPI is a global (TODO change eventually)
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        printf("Errorrrrrrr initializing master SPI\n");
        while (1);
    }
    else {
        printf("Masterrrrrrrr SPI initialized\n");
    }

    /* Write data to register */
    /* Set ODR and scale for XL & G */
    int32_t new_data_XL = platform_write(masterSpi, LSM6DSOX_CTRL1_XL, CTRL1_XL_data_to_write, 1);
    int32_t new_data_G = platform_write(masterSpi, LSM6DSOX_CTRL2_G, CTRL2_G_data_to_write, 1);

    int32_t Func_Access = platform_write(masterSpi, LSM6DSOX_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_data_to_write, 1);

    /* Enable G to sleep */
//    int32_t SleepEnable_G = platform_write(masterSpi, LSM6DSOX_CTRL4_C, CTRL4_C_data_to_write, 1);
    int32_t LowPower_Enable_XL = platform_write(masterSpi, LSM6DSOX_CTRL5_C, CTRL5_C_data_to_write, 1);

    /* Enable interrupt and tap recognition */
    int32_t Tap_Enable = platform_write(masterSpi, LSM6DSOX_TAP_CFG0, TAP_CFG0_data_to_write, 1);
    int32_t InterruptEnable = platform_write(masterSpi, LSM6DSOX_TAP_CFG2, TAP_CFG2_data_to_write, 1);
 //   int32_t Tap_yThre = platform_write(masterSpi, LSM6DSOX_TAP_CFG1, TAP_CFG1_data_to_write, 1);
 //   int32_t Tap_zThre = platform_write(masterSpi, LSM6DSOX_TAP_THS_6D, TAP_THS_6D_data_to_write, 1);
 //   int32_t Tap_Dur = platform_write(masterSpi, LSM6DSOX_INT_DUR2, INT_DUR2_data_to_write, 1);
    int32_t WakeUpTHS = platform_write(masterSpi, LSM6DSOX_WAKE_UP_THS, WAKE_UP_THS_data_to_write, 1);


    int32_t I3C_Disable = platform_write(masterSpi, LSM6DSOX_CTRL9_XL, CTRL9_XL_data_to_write, 1);
    int32_t INT1_Routing = platform_write(masterSpi, LSM6DSOX_MD1_CFG, MD1_CFG_data_to_write, 1);
  //  int32_t INT2_Routing = platform_write(masterSpi, LSM6DSOX_MD2_CFG, MD2_CFG_data_to_write, 1);
    int32_t INT1_Control = platform_write(masterSpi, LSM6DSOX_INT1_CTRL, INT1_CTRL_data_to_write, 1);
 //   int32_t INT2_Control = platform_write(masterSpi, LSM6DSOX_INT2_CTRL, INT2_CTRL_data_to_write, 1);
    int32_t WakeUpDur = platform_write(masterSpi, LSM6DSOX_WAKE_UP_DUR, WAKE_UP_DUR_data_to_write, 1);


    /* Read data from register */
 //   int32_t rx_Data_XL = platform_read(masterSpi, LSM6DSOX_CTRL7_G, &dummy_read_XL, 1);
  //  int32_t rx_Data_G = platform_read(masterSpi, LSM6DSOX_CTRL5_C, &dummy_read_G, 1);
  //  int32_t rx_SleepEnable_G = platform_read(masterSpi, LSM6DSOX_CTRL4_C, &dummy_read_G, 1);
  //  int32_t rx_WakeUp = platform_read(masterSpi, LSM6DSOX_CTRL1_XL, &dummy_read_G, 1);
 //   int32_t rx_InterEnable = platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy_read_G, 1);

    /* Enable interrupt pin */
//    GPIO_setConfig(Board_DIO12, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
//    // Set up the callback function for the wake-up ISR
//    GPIO_setCallback(Board_DIO12, wakeUpCallback);
//    GPIO_enableInt(Board_DIO12);  /* INT1 */

//    GPIO_setConfig(Board_DIO15, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
//    GPIO_enableInt(Board_DIO15);  /* INT2 */

  //  /*  Enable GPIO 3 (index 3 in gpioPinCongfigs in LAUNCHXL.c) for nDRDY Interupt */
  //  GPIO_setConfig(3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);//GPIO_CFG_IN_PU, rising in INT, go into slave ready - to 1
  //  GPIO_setCallback(3, slaveReadyFxn);
  //  GPIO_enableInt(3);

    /* Read T value */
  //  int32_t tempdata = Temperature_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);
    int counter = 0;

    /* Read XL value - acceleration */
    while (1) {

        int32_t XL_data = Acceleration_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);

        int32_t G_data = Angular_Rate_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);
        check_status = Activity_Detection(masterSpi, LSM6DSOX_WAKE_UP_SRC, 1);
        counter += 1;
        printf("now is %d\n time\n", counter);

        if(counter==20){
            printf("Entering sleeping mode since number is %d\n", counter);
            SPI_close(masterSpi);

            sleep(3);

            printf("Entering sleeping mode!");
            init_SPI_IMU();
            printf("Waking up!");
            check_status = 1;

        }
    }

        /* Read current output value for all pins */
   //     currentOutputVal =  PIN_getPortOutputValue(hPin);

        /* Toggle the LEDs, configuring all LEDs at once */
     //   PIN_setPortOutputValue(hPin, ~currentOutputVal);

    //    init_SPI_IMU();


        /* Sleep, to let the power policy transition the device to standby */
    //    sleep(standbyDuration);

     //   init_SPI_IMU();

        /* Read current output value for all pins */
   //     currentOutputVal =  PIN_getPortOutputValue(hPin);

        /* Toggle the LEDs, configuring all LEDs at once */
    //    PIN_setPortOutputValue(hPin, ~currentOutputVal);


    /* Read Gyro value - angular rate */
  //  int32_t G_data = Angular_Rate_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);

  //  memset()


    /* Close SPI */
    SPI_close(masterSpi);

    return 0;

}



/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{
    int32_t         status;
    PIN_State       pinState;
    PIN_Handle      hPin;
    uint32_t        currentOutputVal;
    uint32_t        standbyDuration = 3;


    /* Allocate LED pins */
    hPin = PIN_open(&pinState, LedPinTable);

    /*
     * Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY are GPIO pins connected
     * between the master & slave.  These pins are used to synchronize
     * the master & slave applications via a small 'handshake'.  The pins
     * are later used to synchronize transfers & ensure the master will not
     * start a transfer until the slave is ready.  These pins behave
     * differently between spimaster & spislave examples:
     *
     * spimaster example:
     *     * Board_SPI_MASTER_READY is configured as an output pin.  During the
     *       'handshake' this pin is changed from low to high output.  This
     *       notifies the slave the master is ready to run the application.
     *       Afterwards, the pin is used by the master to notify the slave it
     *       has opened Board_SPI_MASTER.  When Board_SPI_MASTER is opened, this
     *       pin will be pulled low.
     *
     *     * Board_SPI_SLAVE_READY is configured as an input pin. During the
     *       'handshake' this pin is read & a high value will indicate the slave
     *       ready to run the application.  Afterwards, a falling edge interrupt
     *       will be configured on this pin.  When the slave is ready to perform
     *       a transfer, it will pull this pin low.
     *
     * Below we set Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY initial
     * conditions for the 'handshake'.
     */
    /* INIT */
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH+2; // 2: index sent every time
    RF_cmdPropTx.pPkt = packet; //global variable
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW; //send immediately
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams); //global defined, call in other places

    uint8_t test_buffer[2];
    test_buffer[0] = 0x09;
    test_buffer[1] = 0x25;

    send_databuffer(test_buffer,sizeof(test_buffer));
 //   send_databuffer(test_startup_buffer2,sizeof(test_startup_buffer2));//confirm Tx OK; send the data wirelessly; test_star..; size of the buffer as well

    Power_enablePolicy(); //how to sleep

    /*
     * Handshake - Set Board_SPI_MASTER_READY high to indicate master is ready
     * to run.  Wait Board_SPI_SLAVE_READY to be high.
     */
 //   GPIO_write(Board_SPI_MASTER_READY, 1);
  //  while (GPIO_read(Board_SPI_SLAVE_READY) == 0) {}

    /* Handshake complete; now configure interrupt on Board_SPI_SLAVE_READY */
  //  GPIO_setConfig(Board_SPI_SLAVE_READY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
 //   GPIO_setCallback(Board_SPI_SLAVE_READY, slaveReadyFxn);
  //  GPIO_enableInt(Board_SPI_SLAVE_READY);

    /*
     * Create synchronization semaphore; the master will wait on this semaphore
     * until the slave is ready.
     */

 //   status = sem_init(&masterSem, 0, 0);
 //   if (status != 0) {
      //  Display_printf(display, 0, 0, "Error creating masterSem\n");

   //     while(1);
  //  }


    /*
     * Master has opened Board_SPI_MASTER; set Board_SPI_MASTER_READY high to
     * inform the slave.
     */
 //   GPIO_write(Board_SPI_MASTER_READY, 0);

    /* Communicate with IMU */

    init_SPI_IMU();
    SPI_write_data();

    /* Check IMU ID */
 //   uint8_t dummy_read_XL;
 //   int32_t rx_Data_XL = platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy_read_XL, 1);


    while (1) {

        int32_t XL_data = Acceleration_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);
        int32_t G_data = Angular_Rate_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1);

      //  uint32_t XL_data_XY = (uint32_t)XL_data[0];

      //  XL_data_XY <<= 16;
      //  XL_data_XY |= XL_data[1];

     //   printf("XL_data_XY: 0x%048\n", XL_data_XY);


     //   send_databuffer(XL_data,sizeof(XL_data));
    //    send_databuffer(XL_data[1],sizeof(XL_data[1]));


   //     int check_status = Activity_Detection(masterSpi, LSM6DSOX_WAKE_UP_SRC, 1);

        /*transmit through RF*/

   //     if(check_status==0){

     //       sleep(3);

            /* Read current output value for all pins */
       //     currentOutputVal =  PIN_getPortOutputValue(hPin);

            /* Toggle the LEDs, configuring all LEDs at once */
         //   PIN_setPortOutputValue(hPin, ~currentOutputVal);

     //   }
    }

    return (NULL);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;


    /* Call driver init functions. */
    Display_init();
    Board_init();
    GPIO_init();
    SPI_init();
 //   Power_init();
 //   Power_enablePolicy();


    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    Display_printf(display, 0, 0, "Starting the SPI master example");
    Display_printf(display, 0, 0, "This example requires external wires to be "
        "connected to the header pins. Please see the Board.html for details.\n");

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL); // call master thread here
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    return (NULL);
}
