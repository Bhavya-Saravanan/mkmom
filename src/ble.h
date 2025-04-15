/***********************************************************************
 * @file      ble.h
 * @version   1.0
 * @brief     Header file for Bluetooth Low Energy (BLE) communication.
 *
 * This file provides function prototypes and data structures for BLE
 * initialization, event handling, temperature notification, and IEEE formatting.
 *
 * @author    Bhavya Saravanan
 * @date      23-03-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course     ECEN 5823: IoT Embedded Firmware
 * @instructor Chris Choi
 *
 * @resources  Silicon Labs Bluetooth API, Lecture slides.
 ***********************************************************************/

#ifndef BLE_H_
#define BLE_H_

#include "sl_bt_api.h"
#include <stdint.h>
#include <stdbool.h>

// Define BLE Service and Characteristic UUIDs
#define HEALTH_THERMOMETER_SERVICE_UUID  0x1809  ///< UUID for Health Thermometer Service
#define TEMPERATURE_MEASUREMENT_CHAR_UUID 0x2A1C ///< UUID for Temperature Measurement Characteristic

#define BUTTON_SERVICE_UUID  0x0000000138c8433e87ec652a2d136289 ///< UUID for button service
#define BUTTON_CHAR_UUID 0x0000000238c8433e87ec652a2d136289 ///< UUID for button Characteristic
/**
 * @brief BLE Data Structure.
 *
 * This structure holds BLE-related information such as connection state,
 * advertising parameters, and indication status.
 */
typedef struct {
    bd_addr       myAddress;                  ///< Device Bluetooth address
    uint8_t       myAddressType;              ///< Bluetooth address type
    uint8_t       advertisingSetHandle;       ///< BLE advertising set handle
    bool          connection_open;            ///< True when a BLE connection is established
    bool          ok_to_send_htm_indications; ///< True when indications are enabled
    bool          indication_in_flight;       ///< True when an indication is in-flight
    uint8_t       connection_handle;          ///< Handle for the active BLE connection
    uint32_t      received_service_handle;     ///< Stores discovered GATT Service handle
    uint32_t      received_characteristic_handle; ///< Stores discovered Characteristic handle
    bool          pb0_indication;              ///< True if PB0 indication is enabled
    uint32_t      received_button_service_handle;     ///< Stores discovered GATT Service handle
    uint32_t      received_button_characteristic_handle;
    bool  toggle_indication;
} ble_data_struct_t;

/**
 * @brief Initializes the BLE stack and starts advertising.
 *
 * This function sets up the BLE module, creates an advertising set,
 * configures advertising intervals, and starts BLE advertising.
 *
 * @param None
 * @return None
 */
void ble_init(void);

/**
 * @brief Handles BLE events and updates system state.
 *
 * This function processes various BLE events such as system boot,
 * connection opened, connection closed, and GATT characteristic status updates.
 *
 * @param evt Pointer to the Bluetooth event structure (`sl_bt_msg_t`).
 * @return None
 */
void handle_ble_event(sl_bt_msg_t *evt);

/**
 * @brief Sends a temperature notification to the connected BLE client.
 *
 * This function formats the temperature value, writes it to the GATT database,
 * and sends an indication to the BLE client if allowed.
 *
 * @param temperature_value The temperature in degrees Celsius.
 * @return None
 */
void send_temperature_notification(float temperature_value);

/**
 * @brief Converts a temperature value to IEEE-11073 32-bit floating point format.
 *
 * This function scales the temperature, extracts the exponent and mantissa,
 * and packs them into an IEEE-11073 format.
 *
 * @param temp_value The temperature value in degrees Celsius.
 * @return The formatted IEEE-11073 32-bit floating point value.
 */
uint32_t IEEEformating(float temp_value);

/**
 * @brief Converts IEEE-11073 formatted temperature data to Celsius.
 *
 * This function extracts the mantissa and exponent from a BLE characteristic value
 * formatted in IEEE-11073 format and converts it into a readable Celsius temperature.
 *
 * @param buffer_ptr Pointer to the raw temperature data buffer.
 * @return Converted temperature value in degrees Celsius.
 */
float FLOAT_TO_CELSIUS(const uint8_t *buffer_ptr);

/**
 * @brief Returns a pointer to the BLE data structure.
 *
 * This function provides access to the global BLE data structure
 * containing BLE state and connection information.
 *
 * @param None
 * @return Pointer to `ble_data_struct_t`
 */
ble_data_struct_t* getBleDataPtr(void);

/**
 * @brief Resets the BLE transmission queue.
 *
 * This function resets the circular buffer queue by setting the write pointer (`wptr`)
 * and read pointer (`rptr`) to zero and clearing the full-queue flag (`isflag`).
 * It ensures that the queue is empty and ready for new entries.
 *
 * @param None
 * @return None
 */
void reset_queue (void);

/**
 * @brief Processes queued BLE indications and sends them.
 *
 * This function reads queued indications from a circular buffer
 * and sends them if no indication is currently in flight.
 *
 * @param None
 * @return None
 */
void process_indications(void);

/**
 * @brief Adds a data entry to the BLE transmission queue.
 *
 * This function inserts a new BLE data entry into a circular buffer
 * for processing indications.
 *
 * @param charHandle Characteristic handle for the data.
 * @param bufLength Length of the data buffer.
 * @param buffer Pointer to the data buffer.
 * @return `false` on success, `true` if queue is full or invalid input.
 */
bool write_queue(uint16_t charHandle, uint32_t bufLength, uint8_t *buffer);

/**
 * @brief Reads a data entry from the BLE transmission queue.
 *
 * This function retrieves an entry from the circular buffer for processing.
 *
 * @param charHandle Pointer to store the characteristic handle.
 * @param bufLength Pointer to store the data length.
 * @param buffer Pointer to store the data.
 * @return `false` on success, `true` if queue is empty or invalid input.
 */
bool read_queue(uint16_t *charHandle, uint32_t *bufLength, uint8_t *buffer);

/**
 * @brief Retrieves the number of elements currently in the BLE transmission queue.
 *
 * This function calculates the number of data entries currently present
 * in the circular buffer queue. It checks whether the queue is full,
 * empty, or partially filled, and returns the corresponding depth.
 *
 * @param None
 * @return uint32_t - The number of elements in the queue.
 */
uint32_t get_queue_depth(void);

#endif // BLE_H_


