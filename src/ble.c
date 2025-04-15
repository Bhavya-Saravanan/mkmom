/***********************************************************************
 * @file      ble.c
 * @version   1.0
 * @brief     Implementation file for Bluetooth Low Energy (BLE) communication.
 *
 * This file contains functions for initializing BLE, handling BLE events,
 * sending temperature notifications, and formatting temperature values
 * in IEEE-11073 format.
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
#include "em_letimer.h"
#include "sl_bt_api.h"
#include "ble.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#include "sl_power_manager.h"
#include "i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include <stdint.h>
#include "ble_device_type.h"
#include "lcd.h"
#include "timer.h"
#include "sl_bt_api.h"
#include <math.h>
#include"scheduler.h"
#include"gpio.h"
#include <string.h>
#include <stdbool.h>

// Variables for BLE connection parameters
uint16_t interval;
uint16_t latency;
uint16_t timeout;
// Declare circular buffer and pointers
uint32_t         wptr = 0;
uint32_t         rptr = 0;
bool             isflag = false;

#define SL_BT_SM_AUTHENTICATION_BONDING_FLAG 0x6F //Setting up the security manager configuration flag
#define QUEUE_DEPTH 16  // Circular buffer size
#define MAX_BUFFER_LENGTH 5


typedef struct {
    uint16_t charHandle;  // Characteristic handle
    size_t bufferLength;  // Length of data
    uint8_t buffer[MAX_BUFFER_LENGTH];  // Data buffer
} queue_struct_t;

// Structure to store BLE connection data
typedef struct {
    uint8_t connection_handle;
    bool bonded;
} ble_data_t;


// Global BLE data structure
static ble_data_struct_t ble_data;
static uint32_t pairing_passkey = 0;  // Stores the passkey for numeric comparison
static bool waiting_for_passkey_confirmation = false;  // Tracks if waiting for PB0 confirmation
static bool pb0_indication_in_flight = false;  // Tracks PB0 indication status
static queue_struct_t my_queue[QUEUE_DEPTH]; // Circular buffer
ble_data_t ble_datas = {0};
bool pairing_in_progress = false;  // Track if pairing is ongoing



#define TEMPERATURE_BUFFER_SIZE 5  // Buffer size for temperature notifications
#define MANTISSA_MASK 0x00FFFFFF  // Mask for 24-bit mantissa
#define ADVERTISING_INTERVAL_MIN  400    // Minimum advertising interval (250ms * 1.6 = 400 units)
#define ADVERTISING_INTERVAL_MAX  400    // Maximum advertising interval (250ms * 1.6 = 400 units)
#define CONNECTION_INTERVAL_MIN   60     // Minimum connection interval (75ms / 1.25ms)
#define CONNECTION_INTERVAL_MAX   60     // Maximum connection interval (75ms / 1.25ms)
#define SLAVE_LATENCY             4      // Slave latency (300ms / 75ms)
#define SUPERVISION_TIMEOUT       76     // Supervision timeout
#define MIN_CONNECTION_LENGTH     0      // Minimum connection length
#define MAX_CONNECTION_LENGTH     65535  // Maximum connection length
//client
#define DEFAULT_MIN_CONN_INTERVAL       60    // 60 * 1.25ms = 75ms
#define DEFAULT_MAX_CONN_INTERVAL       60    // 60 * 1.25ms = 75ms
#define DEFAULT_SLAVE_LATENCY           4     // Number of skipped connection intervals
#define DEFAULT_SUPERVISION_TIMEOUT     83    // 83 * 10ms = 830ms
#define DEFAULT_MIN_CE_LENGTH           0     // Typically unused
#define DEFAULT_MAX_CE_LENGTH           4     // Typical default

#define DEFAULT_SCAN_MODE               1     // 1 → Passive scanning
#define DEFAULT_SCAN_INTERVAL           80    // Scan interval (units of 0.625 ms)
#define DEFAULT_SCAN_WINDOW             40    // Scan window (must be ≤ scan interval)

#define Encryption_error             0x110F
uint8_t button_service_uuid[] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00 ,0x00 };
uint8_t button_char_uuid[] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00 ,0x00 };
uint8_t htm_service_uuid[2] = { 0x09, 0x18 };
uint8_t temp_char_uuid[2] = { 0x1C, 0x2A };
/**
 * @brief     Returns a pointer to the BLE data structure.
 *
 * This function provides access to the global BLE data structure.
 *
 * @param     None
 * @return    Pointer to `ble_data_struct_t`
 */
ble_data_struct_t* getBleDataPtr() {
    return (&ble_data);
}

/**
 * @brief     Sends a temperature notification to the connected BLE client.
 *
 * This function formats the temperature value, writes it to the GATT database,
 * and sends an indication to the BLE client if allowed.
 *
 * @param     temperature_value  The temperature to send in degrees Celsius.
 * @return    None
 */
void send_temperature_notification(float temperature_value)
{
     sl_status_t sc;

     uint8_t htm_temperature_buffer[5] = {0}; // Buffer to store temperature data

     uint32_t temp = IEEEformating(temperature_value);
     // Populate the buffer with temperature data
     htm_temperature_buffer[0] = 0x00; // Flags: 0x00 for Celsius, no timestamp
     htm_temperature_buffer[1] = (uint8_t)(temp & 0xFF);
     htm_temperature_buffer[2] = (uint8_t)((temp >> 8) & 0xFF);
     htm_temperature_buffer[3] = (uint8_t)((temp >> 16) & 0xFF);
     htm_temperature_buffer[4] = (uint8_t)((temp >> 24) & 0xFF);

     sc = sl_bt_gatt_server_write_attribute_value(
            gattdb_temperature_measurement, 0, TEMPERATURE_BUFFER_SIZE, htm_temperature_buffer);

    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed by writting to local database: 0x%x", sc);
    }

    // Send indication if connection is open and allowed
    if (ble_data.connection_open && ble_data.ok_to_send_htm_indications)
    {
        write_queue(gattdb_temperature_measurement, TEMPERATURE_BUFFER_SIZE, htm_temperature_buffer);
        process_indications();
        displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %.1f ",temperature_value);
    }

}
/**
 * @brief     Converts a temperature value to IEEE-11073 32-bit floating point format.
 *
 * This function scales the temperature, extracts the exponent and mantissa,
 * and packs them into an IEEE-11073 format.
 *
 * @param     temp_value  The temperature value in degrees Celsius.
 * @return    The formatted IEEE-11073 32-bit floating point value.
 */

uint32_t IEEEformating(float temp_value)
{
    union {
        struct {
            uint32_t mantissa : 24; // 24-bit mantissa field
            uint32_t exponent : 8;  // 8-bit exponent field
        };
        uint32_t ieee_value;
    } ieee_temp;

    ieee_temp.exponent = 0xFE; // Exponent for scaling (divide by 100)

    //  mantissa is positive and uses 24 bits
    int32_t scaled_mantissa = (int32_t)(temp_value * 100); // Scale temperature to 2 decimal places
    ieee_temp.mantissa = (uint32_t)(scaled_mantissa & MANTISSA_MASK);// Mask mantissa to 24 bits

    return ieee_temp.ieee_value;
}
/**
 * @brief     Converts IEEE-11073 formatted temperature data to Celsius.
 *
 * This function extracts the mantissa and exponent from a BLE characteristic value
 * formatted in IEEE-11073 format and converts it into a readable Celsius temperature.
 *
 * @param     buffer_ptr  Pointer to the raw temperature data buffer.
 * @return    Converted temperature value in degrees Celsius.
 */
float FLOAT_TO_CELSIUS(const uint8_t *buffer_ptr)
{
    // Extract the exponent (8-bit signed integer)
    int8_t exponent = (int8_t)buffer_ptr[4];

    // Extract the mantissa (24-bit signed integer)
    int32_t mantissa = (int32_t)(
        (buffer_ptr[1] << 0)  |
        (buffer_ptr[2] << 8)  |
        (buffer_ptr[3] << 16)
    );

    // Sign extend the mantissa if negative (check the sign bit)
    if (buffer_ptr[3] & 0x80) {
        mantissa |= 0xFF000000;  // Extend sign for 32-bit integer
    }

    // Compute the temperature in Celsius
    float temperature = mantissa * pow(10, exponent);

    return temperature;  // Return float value
}
/**
 * @brief     Initializes the BLE stack and sets up advertising.
 *
 * This function retrieves the Bluetooth address, creates an advertising set,
 * configures advertising parameters, and starts advertising.
 *
 * @param     None
 * @return    None
 */

void ble_init(void)
{
    sl_status_t sc;
    uint8_t initial_value = 0x00;
    // Get Bluetooth Address
    sc = sl_bt_system_get_identity_address(&ble_data.myAddress,&ble_data.myAddressType );
    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed to get identity address: 0x%04X\n", sc);
    }
    else {

            displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                          ble_data.myAddress.addr[0],
                          ble_data.myAddress.addr[1],
                          ble_data.myAddress.addr[2],
                          ble_data.myAddress.addr[3],
                          ble_data.myAddress.addr[4],
                          ble_data.myAddress.addr[5]);
        }

    // Create an advertising set
    sc = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed to create advertiser set: 0x%04X\n", sc);
    }

    // Generate data for advertising (General Discoverable Mode)
    sc = sl_bt_legacy_advertiser_generate_data(ble_data.advertisingSetHandle,
                                               sl_bt_advertiser_general_discoverable);
    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed to generate advertising data: 0x%04X\n", sc);
    }

    // Configure advertising interval to 250ms
    sc = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle,
                                     ADVERTISING_INTERVAL_MIN,  // Min interval
                                     ADVERTISING_INTERVAL_MAX,  // Max interval
                                     0, 0);  // Duration & max events (infinite advertising)

    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed to set advertiser timing: 0x%04X\n", sc);
    }

    // Start advertising
    sc = sl_bt_legacy_advertiser_start(ble_data.advertisingSetHandle,
                                       sl_bt_advertiser_connectable_scannable);
    if (sc != SL_STATUS_OK) {
        LOG_ERROR("Failed to start advertising: 0x%04X\n", sc);
    }

    sl_status_t status = sl_bt_gatt_server_write_attribute_value(
        gattdb_button_state,  // GATT characteristic handle
        0,                    // Offset
        1,                    // Length
        &initial_value         // Data
    );

    if (status != SL_STATUS_OK) {
        LOG_ERROR("GATT Init Failed! Status: %d", status);
    }
    sl_bt_sm_delete_bondings();
    sl_bt_sm_configure(SL_BT_SM_AUTHENTICATION_BONDING_FLAG,
                       sl_bt_sm_io_capability_displayyesno);

}

/**
 * @brief : Handles Bluetooth events and updates system state.
 *  It distinguishes between the Client and Server roles
 * using the `DEVICE_IS_BLE_SERVER` macro and handles events accordingly.
 *
 * This function processes various BLE events such as connection, disconnection,
 * and GATT characteristic status updates.
 *
 * @param     evt  Pointer to the Bluetooth event structure.
 * @return    None
 */
void handle_ble_event(sl_bt_msg_t *evt)
{

 #if DEVICE_IS_BLE_SERVER
    sl_status_t sc;
    uint32_t signal;

    switch (SL_BT_MSG_ID(evt->header))
    {
        case sl_bt_evt_system_boot_id:
          displayInit();
          displayPrintf(DISPLAY_ROW_NAME, "%s", BLE_DEVICE_TYPE_STRING);
            ble_init();
            displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
            displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A9");
            displayPrintf(DISPLAY_ROW_9, "Button Released");
            break;

        case sl_bt_evt_connection_opened_id:

            displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
            // Update BLE connection state
            ble_data.connection_open = true;
            ble_data.ok_to_send_htm_indications = false;
            ble_data.indication_in_flight = false;
            ble_data.connection_handle = evt->data.evt_connection_opened.connection;

            // Set connection parameters as per the assignment requirements:
            sc = sl_bt_connection_set_parameters(
                ble_data.connection_handle,      // Connection handle
                CONNECTION_INTERVAL_MIN,         // Min connection interval
                CONNECTION_INTERVAL_MAX,         // Max connection interval
                SLAVE_LATENCY,                   // Slave latency
                SUPERVISION_TIMEOUT,             // Supervision timeout
                MIN_CONNECTION_LENGTH,           // Min connection length
                MAX_CONNECTION_LENGTH            // Max connection length
            );
            if (sc != SL_STATUS_OK) {
                LOG_ERROR("Failed to set connection parameters: 0x%04X\n", sc);
            }

            break;


        case sl_bt_evt_connection_closed_id:
            displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
            ble_data.connection_open = false;
            ble_data.ok_to_send_htm_indications = false;
            ble_data.indication_in_flight = false;
            gpioLed1SetOff();
            gpioLed0SetOff();
            // Generate new advertising data
            sc = sl_bt_legacy_advertiser_generate_data(ble_data.advertisingSetHandle,
                                                       sl_bt_advertiser_general_discoverable);
            if (sc != SL_STATUS_OK) {
                LOG_ERROR("Failed to generate advertising data: 0x%04X\n", sc);

            }

            // Restart advertising after client has disconnected
            sc = sl_bt_legacy_advertiser_start(ble_data.advertisingSetHandle,
                                               sl_bt_advertiser_connectable_scannable);

            if (sc != SL_STATUS_OK) {
                LOG_ERROR("Failed to restart advertising: 0x%04X\n", sc);
            }
            displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
            reset_queue();
            sl_bt_sm_delete_bondings();
            break;


        /*case sl_bt_evt_connection_parameters_id:

          interval = evt->data.evt_connection_parameters.interval;
          latency = evt->data.evt_connection_parameters.latency;
          timeout = evt->data.evt_connection_parameters.timeout;
          LOG_INFO("\r\nInterval:%d ms", (interval * 125) / 100);
          LOG_INFO("\r\nLatency: %d ", latency);
          LOG_INFO("\r\nSupervision Timeout: %d ms",timeout / 10);

          break;*/

        case sl_bt_evt_gatt_server_characteristic_status_id:
            if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config) {
                if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication) {

                    if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement) {
                        // Enable HTM Indications
                        ble_data.ok_to_send_htm_indications = true;

                        // Initialize timers and I2C for temperature measurement
                        TimerInit();
                        i2c_init();

                        NVIC_ClearPendingIRQ(LETIMER0_IRQn);
                        NVIC_EnableIRQ(LETIMER0_IRQn);
                        gpioLed0SetOn();
                    }
                    else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state) {
                        // Enable PB0 Indications (Set LED1)
                        gpioLed1SetOn();
                        ble_data.pb0_indication = true;
                    }
             }
            else {
                if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement) {
                    // Disable HTM Indications
                    displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
                    ble_data.indication_in_flight = false;
                    LETIMER_Enable(LETIMER0, false);
                    gpioLed0SetOff();
                }
                else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state) {
                    // Disable PB0 Indications (Clear LED1)
                    ble_data.pb0_indication = false;
                    gpioLed1SetOff();
                    }
                }
            }
           else if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation) {
                // Clear indication in-flight flag after confirmation
                ble_data.indication_in_flight = false;
                process_indications();
            }
            break;

        case sl_bt_evt_system_soft_timer_id:
           displayUpdate();
           break;

        case sl_bt_evt_gatt_server_indication_timeout_id:
          ble_data.indication_in_flight = false;
          break;

        case sl_bt_evt_sm_confirm_bonding_id:
          sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
          break;

        case sl_bt_evt_sm_confirm_passkey_id:
            pairing_passkey = evt->data.evt_sm_confirm_passkey.passkey;
            waiting_for_passkey_confirmation = true;

            displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey: %06lu", pairing_passkey);
            displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
            break;


        case sl_bt_evt_sm_bonded_id:
            ble_datas.bonded = true;
            waiting_for_passkey_confirmation = false;
            displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
            displayPrintf(DISPLAY_ROW_PASSKEY,  " ");
            displayPrintf(DISPLAY_ROW_ACTION, " ");
            break;

        case sl_bt_evt_sm_bonding_failed_id:
            pairing_passkey = 0;
            sl_bt_sm_delete_bondings();
            waiting_for_passkey_confirmation = false;
            displayPrintf(DISPLAY_ROW_CONNECTION, "Bonding Failed");
            break;

       case sl_bt_evt_system_external_signal_id:
           // Extract external signal
           signal = evt->data.evt_system_external_signal.extsignals;

           if (signal & PB0_EVENT) {
               if (waiting_for_passkey_confirmation) {
                   sl_bt_sm_passkey_confirm(ble_data.connection_handle, 1); // Confirm pairing
                   waiting_for_passkey_confirmation = false;
               } else {
                   uint8_t button_value = 0x01;
                   sl_bt_gatt_server_write_attribute_value(gattdb_button_state, 0, 1, &button_value);

                   if (ble_datas.bonded) { // Send indication only if bonded
                       write_queue(gattdb_button_state, 1, &button_value);
                       process_indications();
                   }
                   displayPrintf(DISPLAY_ROW_9, "Button Pressed");
               }
           }

           if (signal & PB0_RELEASED) {
               uint8_t button_value = 0x00;
               sl_bt_gatt_server_write_attribute_value(gattdb_button_state, 0, 1, &button_value);

               if (ble_datas.bonded) { // Send indication only if bonded
                   write_queue(gattdb_button_state, 1, &button_value);
                   process_indications();
               }
               displayPrintf(DISPLAY_ROW_9, "Button Released");
           }
           break;

        default:
            break;
    }
#else
   sl_status_t sc;
switch (SL_BT_MSG_ID(evt->header))
{
  case sl_bt_evt_system_boot_id:

    displayInit();
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A9");
    displayPrintf(DISPLAY_ROW_NAME, "Client");
    // Get Bluetooth Address
      sc = sl_bt_system_get_identity_address(&ble_data.myAddress,&ble_data.myAddressType );
      if (sc != SL_STATUS_OK)
          LOG_ERROR("Failed to get identity address: 0x%04X\n", sc);

      else {
              displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                            ble_data.myAddress.addr[0],
                            ble_data.myAddress.addr[1],
                            ble_data.myAddress.addr[2],
                            ble_data.myAddress.addr[3],
                            ble_data.myAddress.addr[4],
                            ble_data.myAddress.addr[5]);
          }

      displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
      // Set the default connection parameters
      sc = sl_bt_connection_set_default_parameters(
            DEFAULT_MIN_CONN_INTERVAL,
            DEFAULT_MAX_CONN_INTERVAL,
            DEFAULT_SLAVE_LATENCY,
            DEFAULT_SUPERVISION_TIMEOUT,
            DEFAULT_MIN_CE_LENGTH,
            DEFAULT_MAX_CE_LENGTH
      );
      if (sc != SL_STATUS_OK)
          LOG_ERROR("Failed to set connection parameters, error: 0x%04X\n", sc);


      // Configure BLE scanner parameters
     sc = sl_bt_scanner_set_parameters(
              DEFAULT_SCAN_MODE,
              DEFAULT_SCAN_INTERVAL,
              DEFAULT_SCAN_WINDOW
      );
     if (sc != SL_STATUS_OK)
                LOG_ERROR("Failed to set scanner parameters, error: 0x%04X\n", sc);

      sc |= sl_bt_scanner_start(sl_bt_gap_phy_1m, sl_bt_scanner_discover_generic);

      if (sc != SL_STATUS_OK)
          LOG_ERROR("Failed to start scanning, error: 0x%04X\n", sc);

      sc= sl_bt_sm_delete_bondings();
      if (sc != SL_STATUS_OK)
              LOG_ERROR("Failed delete bonding, error: 0x%04X\n", sc);

      sc = sl_bt_sm_configure(0x0F, sl_bt_sm_io_capability_displayyesno);
      if (sc != SL_STATUS_OK)
        LOG_ERROR("Failed to set configuration, error: 0x%04X\n", sc);

      break;


 case sl_bt_evt_scanner_legacy_advertisement_report_id:
   {
      bd_addr server_address = SERVER_BT_ADDRESS;

      // Extract advertisement event flags
      uint8_t adv_flags = evt->data.evt_scanner_legacy_advertisement_report.event_flags;

      // Check if the advertisement is connectable before proceeding
      if (adv_flags & 0x03) {
          if (memcmp(evt->data.evt_scanner_legacy_advertisement_report.address.addr,
                     server_address.addr, sizeof(bd_addr)) == 0) {

              sl_bt_scanner_stop();
              sc = sl_bt_connection_open(
                  evt->data.evt_scanner_legacy_advertisement_report.address,
                  evt->data.evt_scanner_legacy_advertisement_report.address_type,
                  sl_bt_gap_phy_1m,
                  &ble_data.connection_handle
              );

              if (sc == SL_STATUS_OK) {
                  // Print the connected server address on row DISPLAY_ROW_BTADDR2
                  displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X",
                                server_address.addr[0], server_address.addr[1],
                                server_address.addr[2], server_address.addr[3],
                                server_address.addr[4], server_address.addr[5]);
              } else {
                  LOG_ERROR("Failed to open connection, error: 0x%04X\n", sc);
              }
          }
      }
    }
    break;


  case sl_bt_evt_connection_opened_id:

     displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
     // Storing  connection handle
      ble_data.connection_handle = evt->data.evt_connection_opened.connection;
    break;

  case sl_bt_evt_gatt_service_id:
      if (memcmp(evt->data.evt_gatt_service.uuid.data, htm_service_uuid, sizeof(htm_service_uuid))== 0)

          // Storing the service handle
          ble_data.received_service_handle = evt->data.evt_gatt_service.service;

      if (memcmp(evt->data.evt_gatt_service.uuid.data, button_service_uuid, sizeof(button_service_uuid))== 0)

          // Storing the button  service handle
          ble_data.received_button_service_handle = evt->data.evt_gatt_service.service;

      break;

 case sl_bt_evt_gatt_characteristic_id:

     if (memcmp(evt->data.evt_gatt_characteristic.uuid.data, temp_char_uuid, sizeof(temp_char_uuid))== 0)

         ble_data.received_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;

      if (memcmp(evt->data.evt_gatt_characteristic.uuid.data, button_char_uuid, sizeof(button_char_uuid))== 0)

          ble_data.received_button_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;

      break;

 case sl_bt_evt_system_external_signal_id:

      // If PB1 was pressed, attempt to read `button_state`

      if (evt->data.evt_system_external_signal.extsignals & PB1_EVENT_PRESSED)
      {
        if (!pairing_in_progress)  // To Prevent duplicate pairing attempts
              sl_bt_gatt_read_characteristic_value(ble_data.connection_handle, ble_data.received_button_characteristic_handle);

          if(GPIO_PinInGet(PB0_PORT, PB0_PIN)==0)
          {
              if(ble_data.toggle_indication)
              {

                sc = sl_bt_gatt_set_characteristic_notification(
                ble_data.connection_handle,
                ble_data.received_button_characteristic_handle,
                sl_bt_gatt_disable);
                if (sc != SL_STATUS_OK)
                    LOG_ERROR("Failed to set NOTIFICATION: 0x%04X\n", sc);
                ble_data.toggle_indication = false;
              }

              else if(!ble_data.toggle_indication)
              {
                   sc = sl_bt_gatt_set_characteristic_notification(
                      ble_data.connection_handle,
                      ble_data.received_button_characteristic_handle,
                      sl_bt_gatt_indication);
                   ble_data.toggle_indication = true;
              }

          }
          else if(!ble_data.toggle_indication)
             sl_bt_gatt_read_characteristic_value(ble_data.connection_handle, ble_data.received_button_characteristic_handle);
      }
     if ((evt->data.evt_system_external_signal.extsignals & PB0_EVENT) && waiting_for_passkey_confirmation)
     {
          sl_bt_sm_passkey_confirm(ble_data.connection_handle, 1);
          waiting_for_passkey_confirmation = false;

     }
      break;

  case sl_bt_evt_gatt_procedure_completed_id:

      if (evt->data.evt_gatt_procedure_completed.result == Encryption_error) {
         if (!pairing_in_progress) {
             pairing_in_progress = true;  // Mark pairing in progress
             sl_bt_sm_increase_security(ble_data.connection_handle);
         }
     }
     break;

 case sl_bt_evt_sm_confirm_passkey_id:

     pairing_passkey = evt->data.evt_sm_confirm_passkey.passkey;
     displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey: %06lu", pairing_passkey);
     displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
     waiting_for_passkey_confirmation = true;
     break;

 case sl_bt_evt_sm_bonded_id:

     waiting_for_passkey_confirmation = false;
     displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
     displayPrintf(DISPLAY_ROW_PASSKEY, " ");
     displayPrintf(DISPLAY_ROW_ACTION, " ");
     break;

 case sl_bt_evt_sm_bonding_failed_id:

    pairing_passkey = 0;
    sl_bt_sm_delete_bondings();
    waiting_for_passkey_confirmation = false;
    displayPrintf(DISPLAY_ROW_CONNECTION, "Bonding Failed");
    break;


 case sl_bt_evt_gatt_characteristic_value_id:

   if (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication) {

      if (evt->data.evt_gatt_characteristic_value.characteristic == ble_data.received_characteristic_handle)
      {
         // Convert received BLE data into temperature
         float temperature = FLOAT_TO_CELSIUS(evt->data.evt_gatt_characteristic_value.value.data);


         // Display the temperature with one decimal place
        displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %.1f ", temperature);


         // Send confirmation for indication
         sl_bt_gatt_send_characteristic_confirmation(ble_data.connection_handle);
      }
      else
      {
          if (evt->data.evt_gatt_characteristic_value.value.data[0])
             displayPrintf(DISPLAY_ROW_9, "Button Pressed");
          else
               displayPrintf(DISPLAY_ROW_9, "Button Released");

          // Send confirmation for indication
          sl_bt_gatt_send_characteristic_confirmation(ble_data.connection_handle);
       }
   }
  else if (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response) {
       if (evt->data.evt_gatt_characteristic_value.characteristic == ble_data.received_button_characteristic_handle)
            {
              if (evt->data.evt_gatt_characteristic_value.value.data[0])
                   displayPrintf(DISPLAY_ROW_9, "Button Pressed");
              else
                     displayPrintf(DISPLAY_ROW_9, "Button Released");

                // Send confirmation for indication
                sl_bt_gatt_send_characteristic_confirmation(ble_data.connection_handle);
             }
      }

      break;

  case sl_bt_evt_connection_closed_id:

    displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
    displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
    displayPrintf(DISPLAY_ROW_BTADDR2, " ");
    displayPrintf(DISPLAY_ROW_9, " ");
    displayPrintf(DISPLAY_ROW_PASSKEY, " ");
    displayPrintf(DISPLAY_ROW_ACTION, " ");

    waiting_for_passkey_confirmation = false;
    pb0_indication_in_flight = false;
    ble_data.connection_handle = 0;

      // Restart scanning
      sc = sl_bt_scanner_start(sl_bt_gap_phy_1m, sl_bt_scanner_discover_generic);
      if (sc != SL_STATUS_OK)
          LOG_ERROR("Failed to restart scanning, error: 0x%04X\n", sc);


      sc= sl_bt_sm_delete_bondings();
      if (sc != SL_STATUS_OK)
          LOG_ERROR("Failed to delete the bonding , error: 0x%04X\n", sc);

      break;

  case sl_bt_evt_system_soft_timer_id:

        displayUpdate();
        break;

  default:
      break;
}
#endif
}
/**
 * @brief Computes the next pointer position in the circular buffer.
 *
 * This function calculates the next pointer index in a circular buffer.
 *
 * @param ptr Current pointer index.
 * @return Next pointer index in the circular buffer.
 */

static uint32_t nextPtr(uint32_t ptr)
{
    return (ptr + 1) % QUEUE_DEPTH;
}

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

bool write_queue(uint16_t charHandle, uint32_t bufLength, uint8_t *buffer)
{
    if (buffer == NULL || bufLength == 0 || bufLength > MAX_BUFFER_LENGTH) {
        return true;  // Invalid input
    }

    if (wptr == rptr && isflag) {
        return true;  // Queue full, cannot write
    }

    my_queue[wptr].charHandle = charHandle;
    my_queue[wptr].bufferLength = bufLength;
    memcpy(my_queue[wptr].buffer, buffer, bufLength);

    wptr = nextPtr(wptr);

    if (wptr == rptr) {
        isflag = true; // Queue is now full
    }

    return false;  // Successfully added to queue
}

/**
 * @brief Reads an indication from the circular buffer.
 *
 * Retrieves the next queued indication if available.
 *
 * @param charHandle Pointer to store characteristic handle.
 * @param bufLength Pointer to store buffer length.
 * @param buffer Pointer to store read data.
 * @return True if queue is empty, false otherwise.
 */

bool read_queue(uint16_t *charHandle, uint32_t *bufLength, uint8_t *buffer)
{
    if (charHandle == NULL || bufLength == NULL || buffer == NULL) {
        return true;
    }

    if (wptr == rptr && !isflag) {
        return true;  // Queue empty, cannot read
    }

    *charHandle = my_queue[rptr].charHandle;
    *bufLength = my_queue[rptr].bufferLength;
    memcpy(buffer, my_queue[rptr].buffer, *bufLength);

    rptr = nextPtr(rptr);
    isflag = false; // Since we removed an entry, queue cannot be full

    return false;  // Successfully read from queue
}

/**
 * @brief     Retrieves the number of elements currently in the BLE transmission queue.
 *
 * This function calculates the number of data entries currently present
 * in the circular buffer queue. It checks whether the queue is full,
 * empty, or partially filled, and returns the corresponding depth.
 *
 * @param     None
 * @return    uint32_t - The number of elements in the queue.
 */

uint32_t get_queue_depth()
{
    if (isflag) return QUEUE_DEPTH;
    if (wptr == rptr) return 0;
    return (wptr > rptr) ? (wptr - rptr) : (QUEUE_DEPTH - rptr + wptr);
}
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
void reset_queue (void)
{
  wptr = 0;
  rptr = 0;
  isflag = false;

} // reset_queue()
/**
 * @brief Processes queued BLE indications and sends them.
 *
 * Reads queued indications and sends them if no indication is currently in flight.
 *
 * @return None
 */

void process_indications()
{
    if (!ble_data.indication_in_flight && get_queue_depth() > 0) {
        uint16_t charHandle;
        uint32_t bufLength;
        uint8_t buffer[MAX_BUFFER_LENGTH];

        if (!read_queue(&charHandle, &bufLength, buffer)) {
            sl_status_t status = sl_bt_gatt_server_send_indication(
                ble_data.connection_handle,
                charHandle,
                bufLength,
                buffer
            );

            if (status == SL_STATUS_OK) {
                pb0_indication_in_flight = true;  // Mark indication as in-flight
            }
        }
    }
}



