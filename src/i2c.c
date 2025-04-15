/***********************************************************************
 * @file      i2c.c
 * @version   1.0
 * @brief      Implementation file for I2C communication with Si7021 temperature sensor.
 *
 * @author    Bhavya Saravanan, bhsa3618@colorado.edu
 * @date      20-02-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Lecture slides
 ***********************************************************************/

#include "i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "src/timer.h"
#include "src/oscillator.h"
#include "src/scheduler.h"
#include "src/i2c.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "sl_i2cspm.h"
#include "src/timer.h"

static I2C_TransferSeq_TypeDef transferSequence;
static uint8_t temp_buffer[2];  // Buffer to hold temperature data
static uint16_t raw_temperature;  // Store temperature globally for processing

#define SI7021_I2C_ADDR        (0x40)   // I2C address of Si7021 sensor
#define MEASURE_TEMP_NO_HOLD   (0xF3)   // Command to measure temperature
#define TEMP_CONV_DELAY_US     (10800)  // Polling delay for conversion (~10ms)
#define SENSOR_ENABLE_PORT  gpioPortD
#define SENSOR_ENABLE_PIN   15
#define I2C_SCL_PORT gpioPortC
#define I2C_SCL_PIN  10
#define I2C_SDA_PORT gpioPortC
#define I2C_SDA_PIN  11

/**
 * @brief Initializes the I2C interface.
 *
 * This function enables clocks for I2C and GPIO and configures
 * the SCL and SDA pins. It then initializes the I2C peripheral
 * using default parameters for communication.
 *
 * @param None
 * @return None
 */
void i2c_init(void)
{
    // Enabling clocks for I2C and GPIO
    CMU_ClockEnable(cmuClock_I2C0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Configuring I2C SDA and SCL pins with wired-and pull-up mode
    GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAndPullUp, 1);

    // Initializing I2C with standard parameters
    I2CSPM_Init_TypeDef i2cInit = {
        .port = I2C0,
        .sclPort = I2C_SCL_PORT,
        .sclPin = I2C_SCL_PIN,
        .sdaPort = I2C_SDA_PORT,
        .sdaPin = I2C_SDA_PIN,
        .portLocationScl = 14,
        .portLocationSda = 16,
        .i2cRefFreq = 0,  // Default reference clock
        .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,  // 100 kHz Standard I2C speed
        .i2cClhr = i2cClockHLRStandard,
    };

    I2CSPM_Init(&i2cInit);
}

/**
 * @brief Powers the Si7021 temperature sensor ON/OFF.
 *
 * @param enable Boolean flag to turn sensor power ON (true) or OFF (false).
 * @return None
 */
void si7021_power_control(bool enable)
{
    if (enable) {
        GPIO_PinOutSet(SENSOR_ENABLE_PORT, SENSOR_ENABLE_PIN);  // Turn ON power

    } else {
        GPIO_PinOutClear(SENSOR_ENABLE_PORT, SENSOR_ENABLE_PIN); // Turn OFF power
    }
}

/**
 * @brief Sends a temperature measurement command to the Si7021 sensor.
 *
 * This function initiates a temperature measurement by writing the
 * measurement command to the sensor over I2C.
 *
 * @param None
 * @return None
 */
void si7021_send_measurement_command(void)
{
    static uint8_t cmd_data = MEASURE_TEMP_NO_HOLD;

    transferSequence.addr = SI7021_I2C_ADDR << 1;
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &cmd_data;
    transferSequence.buf[0].len = 1;
    NVIC_ClearPendingIRQ(I2C0_IRQn);
    // Enable I2C IRQ before starting the transfer
    NVIC_EnableIRQ(I2C0_IRQn);

    if (I2C_TransferInit(I2C0, &transferSequence) != i2cTransferInProgress) {
        LOG_ERROR("I2C Write Failed: Unable to Send Measurement Command");
    }

}

/**
 * @brief Reads temperature data from the Si7021 sensor.
 *
 * This function initiates an I2C read operation to retrieve
 * the measured temperature data from the sensor.
 *
 * @param None
 * @return None
 */
void si7021_read_temperature(void)
{
    transferSequence.addr = SI7021_I2C_ADDR << 1;
    transferSequence.flags = I2C_FLAG_READ;
    transferSequence.buf[0].data = temp_buffer;//storing the data to my temp_buffer
    transferSequence.buf[0].len = 2;

    // Clear and enable I2C IRQ
    NVIC_ClearPendingIRQ(I2C0_IRQn);//clearing the NVIC
    NVIC_EnableIRQ(I2C0_IRQn); // Enable I2C IRQ before starting the transfer

    if (I2C_TransferInit(I2C0, &transferSequence) != i2cTransferInProgress) {
        LOG_ERROR("I2C Read Failed: Unable to Read Temperature");
    }

}

/**
 * @brief Processes the temperature reading and converts it to Celsius.
 *
 * @param None
 * @return temperature value after the calculated formula for the celsius value
 */
float si7021_process_temperature(void)
{
    raw_temperature = (temp_buffer[0] << 8) | temp_buffer[1]; // Combine two bytes
    float temp_celsius = ((175.72 * raw_temperature) / 65536) - 46.85;//conversion formula

    return temp_celsius;
}

/**
 * @brief I2C interrupt handler.
 *
 * the transfer status and triggering the appropriate event.
 *
 * @param None
 * @return None
 */
void I2C0_IRQHandler(void)
{

    I2C_TransferReturn_TypeDef transferStatus = I2C_Transfer(I2C0);

    if (transferStatus == i2cTransferDone) {
        schedulerSetEventI2CComplete();  // Notify completion event

    }else if(transferStatus<0){
        LOG_INFO("I2C Transfer Failed! Error Code: %d", transferStatus);
    }
}
