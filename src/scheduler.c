/***********************************************************************
 * @file      scheduler.c
 * @version   2.4
 * @brief     Event-driven scheduler with a state machine for handling system events.
 *
 * @author    Bhavya Saravanan
 * @date      23-03-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Lecture slides, Silicon Labs Peripheral Libraries.
 ***********************************************************************/
#include "scheduler.h"
#include "em_core.h"
#include "src/timer.h"
#include "src/i2c.h"
#include "sl_bt_api.h"
#include "src/log.h"
#include "sl_bt_api.h"
#include "em_core.h"
#include "app.h"
#include "ble.h"
#include "lcd.h"
#include"timer.h"
#include "gatt_db.h"  // Include GATT database
#include <string.h>
#include "gpio.h"
#define INCLUDE_LOG_DEBUG 1
int flagg = 0;// Flag for COMP1 tracking
ble_data_struct_t *bleDataPtr; // Get BLE status
ble_data_struct_t *ble;
static state_t currentState = STATE_IDLE;  // Global variable to track the current state
static discovery_state_t discovery_state = SCANNING;

/**
 * @brief     Sets the event for LETIMER0 Underflow.
 *
 * This function is called when the LETIMER0 underflow occurs. It signals the BLE stack
 * to indicate a temperature measurement event.
 *
 * @param     None
 * @return    None
 */
void schedulerSetEventUF(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(TEMPERATURE_EVENT);
    CORE_EXIT_CRITICAL();

}

/**
 * @brief     Sets the event for LETIMER0 COMP1 (used for timer delays).
 *
 * This function is used to signal when a timer delay has expired. The first time it
 * is called, it enables the event flag; subsequent calls trigger the event.
 *
 * @param     None
 * @return    None
 */

void schedulerSetEventCOMP1(void)
{
  if(flagg){
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(TIMER_EVENT); // Notify Bluetooth stack
    CORE_EXIT_CRITICAL();

  }else{
     //LOG_INFO("D");
      flagg=1;// Set flag for first-time use
  }
}

/**
 * @brief     Sets the event when I2C transaction is complete.
 *
 * This function is triggered when an I2C transaction (reading temperature data) completes.
 * It sends a signal to indicate that the data is ready for processing.
 *
 * @param     None
 * @return    None
 */
void schedulerSetEventI2CComplete(void)
{

    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(I2C_EVENT);// Signal I2C completion event
    CORE_EXIT_CRITICAL();
}
/**
 * @brief Schedules an event when PB0 is pressed.
 *
 * This function is called when the PB0 button is detected as pressed. It signals
 * the BLE stack by sending an external event (`PB0_EVENT`), which can be handled
 * in the BLE event loop.
 *
 * @param None
 * @return None
 */

void schedulerSetEventPB0Pressed(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(PB0_EVENT); // Notify BLE stack if pressed
    CORE_EXIT_CRITICAL();
}
/**
 * @brief Schedules an event when PB0 is released.
 *
 * This function is called when the PB0 button is detected as released. It signals
 * the BLE stack by sending an external event (`PB0_RELEASED`), which can be handled
 * in the BLE event loop.
 *
 * @param None
 * @return None
 */

void schedulerSetEventPB0Released(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(PB0_RELEASED);//if released
    CORE_EXIT_CRITICAL();
}
/**
 * @brief Schedules an event when PB1 is pressed.
 *
 * This function is called when the PB1 button is detected as pressed. It signals
 * the BLE stack by sending an external event (`PB1_EVENT_PRSSED`), which can be handled
 * in the BLE event loop.
 *
 * @param None
 * @return None
 */

void schedulerSetEventPB1Pressed(void) {
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(PB1_EVENT_PRESSED);
    CORE_EXIT_CRITICAL();
}
/**
 * @brief Schedules an event when PB1 is released.
 *
 * This function is called when the PB1 button is detected as released. It signals
 * the BLE stack by sending an external event (`PB1_EVENT_RELEASED`), which can be handled
 * in the BLE event loop.
 *
 * @param None
 * @return None
 */
void schedulerSetEventPB1Released(void) {
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(PB1_EVENT_RELEASED);
    CORE_EXIT_CRITICAL();
}
/**
 * @brief     Implements the state machine for temperature measurement.
 *
 * This function manages the system's state transitions for reading the temperature
 * sensor and sending the temperature over BLE. The state machine operates based
 * on events received from the scheduler.
 *
 * @param     evt  Pointer to the Bluetooth event structure (sl_bt_msg_t).
 * @return    None
 */
void state_machine(sl_bt_msg_t *evt)
{

  bleDataPtr = getBleDataPtr(); // Get BLE status

if (bleDataPtr->connection_open && bleDataPtr->ok_to_send_htm_indications){
    if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {

        return;
    }

    // Extract external signal from evt
    uint32_t signal = evt->data.evt_system_external_signal.extsignals;

    switch (currentState)
    {
        case STATE_IDLE:
            if (signal & TEMPERATURE_EVENT) {
              //si7021_power_control(true);
              timerWaitUs_irq(80000);  // Wait for sensor power-up
              currentState = STATE_POWER_ON_SENSOR;
            }
            break;

        case STATE_POWER_ON_SENSOR:
            if (signal & TIMER_EVENT) {
                sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                si7021_send_measurement_command(); // Start temperature measurement
                currentState = STATE_I2C_WRITE_COMPLETE;
            }
            break;

        case STATE_I2C_WRITE_COMPLETE:
            if (signal & I2C_EVENT) {
                sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                NVIC_DisableIRQ(I2C0_IRQn);
                timerWaitUs_irq(10800); // Wait ~10.8ms for temperature conversion
                currentState = STATE_WAIT_FOR_MEASUREMENT;
            }

            break;

        case STATE_WAIT_FOR_MEASUREMENT:
            if (signal & TIMER_EVENT) {
                sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                si7021_read_temperature(); // Read temperature data
                currentState = STATE_READ_TEMPERATURE;
            }
            break;

        case STATE_READ_TEMPERATURE:
            if (signal & I2C_EVENT) {
              NVIC_DisableIRQ(I2C0_IRQn);
               sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                float raw = si7021_process_temperature();
                 send_temperature_notification(raw);
                //si7021_power_control(false); // Power off sensor
                currentState = STATE_IDLE;

            }
            break;

        default:
            //app_log_warning("Unknown state, resetting to IDLE\n");
            currentState = STATE_IDLE;
            break;
    }
  }

}
/**
 * @brief Handles the BLE discovery state machine for service and characteristic discovery.
 *
 * This function transitions through various states to establish a connection, discover
 * the Health Thermometer Service (HTM), identify the temperature characteristic, enable
 * notifications, and handle disconnections and also do handling the enabling the custom button .
 * service its characteristics and enable notification
 *
 *
 * @param evt Pointer to the Bluetooth event structure (`sl_bt_msg_t`).
 *
 * @return None (void). Updates the discovery state and configures BLE services.
 */
void discovery_state_machine(sl_bt_msg_t *evt) {
  sl_status_t sc;
  ble_data_struct_t *bleDataPtr = getBleDataPtr();  // Get the BLE structure pointer

  switch (discovery_state) {

    case SCANNING:
      if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id) {


          uint8_t htm_service_uuid[2] = { 0x09, 0x18 };
          sc = sl_bt_gatt_discover_primary_services_by_uuid(
              bleDataPtr->connection_handle,
              sizeof(htm_service_uuid),
              htm_service_uuid
          );

          if (sc != SL_STATUS_OK) {
              LOG_ERROR("Service Discovery Failed: 0x%04X\n", sc);
          }
          discovery_state = DISCOVER_SERVICE;

      }
        break;

    case DISCOVER_SERVICE:
      if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) {
          if (bleDataPtr->received_service_handle == 0) {
              LOG_ERROR("HTM Service Not Found\n");
              break;
          }
          uint8_t temp_char_uuid[2] = { 0x1C, 0x2A };
          sc = sl_bt_gatt_discover_characteristics_by_uuid(
              bleDataPtr->connection_handle,
              bleDataPtr->received_service_handle,
              sizeof(temp_char_uuid),
              temp_char_uuid
          );

          if (sc != SL_STATUS_OK) {
              LOG_ERROR("Characteristic Discovery Failed: 0x%04X\n", sc);
          }

          discovery_state = DISCOVER_CHARACTERISTIC;

      }
      break;

    case DISCOVER_CHARACTERISTIC:
      if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) {
          if (bleDataPtr->received_characteristic_handle == 0) {
              LOG_ERROR("Temperature Characteristic Not Found\n");
              break;
          }
          sc = sl_bt_gatt_set_characteristic_notification(
              bleDataPtr->connection_handle,
              bleDataPtr->received_characteristic_handle,
              sl_bt_gatt_indication
              );
          if (sc != SL_STATUS_OK) {
              LOG_ERROR("Notification Failed:");
          }
          discovery_state =  SCANNING_BUTTON;

      }
    break;
    case SCANNING_BUTTON:
          if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) {


              uint8_t button_service_uuid[] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00 ,0x00 };
              sc = sl_bt_gatt_discover_primary_services_by_uuid(
                  bleDataPtr->connection_handle,
                  sizeof(button_service_uuid),
                  button_service_uuid
              );

              if (sc != SL_STATUS_OK) {
                  LOG_ERROR("Service Button Discovery Failed: 0x%04X\n", sc);
              }
              discovery_state = DISCOVER_SERVICE_BUTTON;

          }
            break;

        case DISCOVER_SERVICE_BUTTON:
          if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) {
              if (bleDataPtr-> received_button_service_handle == 0) {
                  LOG_ERROR("Button service Not Found\n");
                  break;
              }
             uint8_t button_char_uuid[] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00 ,0x00 };
              sc = sl_bt_gatt_discover_characteristics_by_uuid(
                  bleDataPtr->connection_handle,
                  bleDataPtr-> received_button_service_handle,
                  sizeof(button_char_uuid),
                  button_char_uuid
              );

              if (sc != SL_STATUS_OK) {
                  LOG_ERROR("Characteristic button Discovery Failed: 0x%04X\n", sc);
              }

              discovery_state = DISCOVER_CHARACTERISTIC_BUTTON;

          }
          break;
        case DISCOVER_CHARACTERISTIC_BUTTON:
          if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id) {
              if (bleDataPtr->received_button_characteristic_handle == 0) {
                  LOG_ERROR("Temperature Characteristic Not Found\n");
                  break;
              }
              sc = sl_bt_gatt_set_characteristic_notification(
                  bleDataPtr->connection_handle,
                  bleDataPtr->received_button_characteristic_handle,
                  sl_bt_gatt_indication
                  );
              if (sc != SL_STATUS_OK) {
                  LOG_ERROR("Notification Failed:");
              }


              bleDataPtr->toggle_indication = true;
              displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
              discovery_state =  DISCONNECTED;

          }
        break;
     case DISCONNECTED:
        if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id) {
            discovery_state = SCANNING;
        }
        break;
  }
}





