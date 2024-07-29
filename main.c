#include <xc.h>
#include <stdio.h>

#include "canlib/canlib.h"
#include "common.h"
#include "fxls8974cf.h"
#include "slf3s4000b.h"
#include "rocketlib/include/i2c.h"

#include "mcc_generated_files/mcc.h"
#include "adcc.h"
#include "mcc_generated_files/fatfs/ff.h"

#define STATUS_TIME_DIF 500
#define MESSAGE_TIME_DIF 500

static inline void SET_ACCEL_I2C_ADDR(uint8_t addr) {
    LATC2 = (addr == 0x19); // SA0 pin of FXLS set LSB of 7-bit I2C Address
}

FATFS FatFs; /* FatFs work area needed for each volume */
FIL Fil;

// memory pool for the CAN tx buffer
uint8_t tx_pool[100];
volatile bool seen_can_message = false;

UINT bw;

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);
static void send_status_err(void);

void main(void) {
    // Initialize the device and adc
    SYSTEM_Initialize();
    ADCC_Initialize();

    // Enable global interrupts
    INTCON0bits.GIE = 1;

    // Set up timer 0 for millis
    timer0_init();
    uint32_t last_status_millis = millis();
    uint32_t last_message_millis = millis();

    // Set up CAN TX
    TRISC1 = 0;
    RC1PPS = 0x33;

    // Set up CAN RX
    TRISC0 = 1;
    ANSELC0 = 0;
    CANRXPPS = 0b00010000;

    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);
    // set up CAN tx buffer
    txb_init(tx_pool, sizeof (tx_pool), can_send, can_send_rdy);
    //set up can message
    can_msg_t msg;

    //init flow sensor and accel
    SET_ACCEL_I2C_ADDR(FXLS_I2C_ADDR);
    i2c_init(0b000); // I2C at 100 kHz
    fxls_init();

    // Flow sensor interrupt pin (IRQn)
    TRISC6 = 0; // Flow sensor output from PIC
    LATC6 = 1; // Active low

    uint8_t whoami = fxls_get_whoami();
    uint8_t prod_rev = fxls_get_prod_rev();

    // Accelerometer interrupt pin (INT2)
    TRISB3 = 0; // Accelerometer output from PIC
    LATB3 = 0; // Active high

    TRISA2 = 0; // Blue LED output enable
    TRISA3 = 0; // Green LED output enable
    TRISA4 = 0; // Red LED output enable
    TRISC2 = 0; // Accelerometer I2C select pin output enable

    if (f_mount(&FatFs, "", 1) == FR_OK) { /* Mount SD */
        if (f_open(&Fil, "Data.txt", FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK) {
            f_write(&Fil, "BEGIN LOG\r\n", 10, &bw);
            f_close(&Fil);
        }
    } else {
        while (f_mount(&FatFs, "", 1) != FR_OK) {
            if (millis() - last_status_millis > STATUS_TIME_DIF) {
                last_status_millis = millis();
                send_status_err();
            }
        }
    }


    while (1) {
        CLRWDT();

        //adc_result_t adc_value = ADCC_GetSingleConversion(channel_ANA0);

        if (millis() - last_status_millis > STATUS_TIME_DIF) {
            last_status_millis = millis();
            send_status_ok();
        }

        /*
                // Read accelerometer data
                //BROCKEN
        
                LATB3 = 1;
                while (!data_ready()) {
                    for (int i = 0; i < 5; i++) {

                    }
                }
                LATB3 = 0;

                // Variables to hold accelerometer data
                int16_t x = 0;
                int16_t y = 0;
                int16_t z = 0;

                // Read accelerometer data
         *                 fxls_read_accel_data(&x, &y, &z);

                build_analog_data_msg(millis(), SENSOR_MAG_2, x, &msg);
                can_send(&msg);
                while (!can_send_rdy()) {
                }
                for (int i = 0; i < 525; ++i);
                build_analog_data_msg(millis(), SENSOR_VELOCITY, y, &msg);
                can_send(&msg);
                while (!can_send_rdy()) {
                }
                for (int i = 0; i < 525; ++i);
                build_analog_data_msg(millis(), SENSOR_ARM_BATT_1, z, &msg);
                can_send(&msg);
                while (!can_send_rdy()) {
                }
                for (int i = 0; i < 525; ++i);
         */

        /*
        // Read liquid flow sensor data
        //BROCKEN
        LATC6 = 0;
        uint16_t flow = 0;
        uint16_t temperature = 0;
        read_flow_sensor_data(&flow, &temperature);
        LATC6 = 1;

        build_analog_data_msg(millis(), SENSOR_MAG_2, flow, &msg);
        can_send(&msg);
        //never true?
        while (!can_send_rdy()) {
        }
        for (int i = 0; i < 525; ++i);
        build_analog_data_msg(millis(), SENSOR_VELOCITY, temperature, &msg);
        can_send(&msg);
        while (!can_send_rdy()) {
        }

        if (f_open(&Fil, "Data.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK) {
            f_write(&Fil, "Hello world!\r\n", 14, &bw);
        }
        f_close(&Fil);
         */
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);
    int dest_id = -1;
    int cmd_type = -1;
    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    switch (msg_type) {
        case MSG_LEDS_ON:
            BLUE_LED_ON();
            GREEN_LED_ON();
            RED_LED_ON();
            break;

        case MSG_LEDS_OFF:
            BLUE_LED_OFF();
            GREEN_LED_OFF();
            RED_LED_OFF();
            break;

        case MSG_RESET_CMD:
            dest_id = get_reset_board_id(msg);
            if (dest_id == BOARD_UNIQUE_ID || dest_id == 0) {
                RESET();
            }
            break;

            // all the other ones - do nothing
        default:
            break;
    }
}

// Send a CAN message with nominal status

static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);
    can_send(&board_stat_msg);

    uint32_t time = millis();
    char text[32];

    sprintf(text, "%d\r\n", time);

    if (f_open(&Fil, "Data.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK) {
        //fix
        f_write(&Fil, text, sizeof (text), &bw);
        f_close(&Fil);

        BLUE_LED_TOGGLE();
    }
}

static void send_status_err(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_MISSING_CRITICAL_BOARD, NULL, 0, &board_stat_msg);
    can_send(&board_stat_msg);

    RED_LED_TOGGLE();
}

static void __interrupt() interrupt_handler(void) {
    if (PIR5) {
        can_handle_interrupt();
    }

    // Timer0 has overflowed - update millis() function
    // This happens approximately every 500us
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        timer0_handle_interrupt();
        PIR3bits.TMR0IF = 0;
    }
}