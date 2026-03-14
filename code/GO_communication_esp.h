/************************************************************************************//**
* \file         GO_communication_esp.h
* \brief        STM32H to ESP32 communication driver (IoT platform only).
*               Implements a framed UART protocol with CRC-16/CCITT integrity.
*               Frame: [SOF0][SOF1][MSG_ID][LEN_LO][LEN_HI][PAYLOAD][CRC_LO][CRC_HI]
*               CRC-16/CCITT: polynomial 0x1021, init 0xFFFF, over MSG_ID+LEN+PAYLOAD.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2024 (c) by GOcontroll      http://www.gocontroll.com   All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* Permission is hereby granted, free of charge, to any person obtaining a copy of this
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* \endinternal
****************************************************************************************/

#ifndef GO_COMMUNICATION_ESP_H
#define GO_COMMUNICATION_ESP_H

/****************************************************************************************
* IoT platform only (STM32H)
****************************************************************************************/
#ifdef GOCONTROLL_IOT

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32h5xx_hal.h"

/*==============================================================================================
** Protocol constants — keep identical in stminterface.h on the ESP32 side.
**
** Frame layout: [SOF0][SOF1][MSG_ID][LEN_LO][LEN_HI][PAYLOAD 0..512 bytes][CRC_LO][CRC_HI]
** CRC-16/CCITT: polynomial 0x1021, init 0xFFFF, MSB-first, over MSG_ID+LEN_LO+LEN_HI+PAYLOAD.
==============================================================================================*/

#define ESPIF_SOF0               0xAAu  /**< First start-of-frame byte  */
#define ESPIF_SOF1               0x55u  /**< Second start-of-frame byte */

/** Frame overhead: 2 SOF + 1 MSG_ID + 2 LEN + 2 CRC = 7 bytes. */
#define ESPIF_FRAME_OVERHEAD     7u

/** Maximum payload size. Must match STMIF_MAX_PAYLOAD_LEN on the ESP32 side. */
#define ESPIF_MAX_PAYLOAD_LEN    512u

/*==============================================================================================
** Message identifiers (MSG_ID field, 1 byte).
**
**   0x01–0x0F  Bidirectional utility
**   0x10–0x1F  STM32H → ESP  controller telemetry
**   0x20–0x2F  STM32H → ESP  modem / MQTT / GPS control
**   0x30–0x3F  ESP → STM32H  MQTT events
**   0x40–0x4F  ESP → STM32H  GPS data
**   0x50–0x5F  ESP → STM32H  modem / LTE status
==============================================================================================*/
typedef enum
{
    /* Bidirectional */
    ESPIF_MSG_HEARTBEAT         = 0x01u,  /**< Keep-alive. Payload: none.                     */
    ESPIF_MSG_ACK               = 0x02u,  /**< Acknowledge. Payload: 1 byte (MSG_ID ack'd).   */

    /* STM32H → ESP */
    ESPIF_MSG_STATIC_INFO       = 0x10u,  /**< One-time identity/version snapshot. Payload: 17 bytes fixed. */
    ESPIF_MSG_CYCLIC_INFO       = 0x11u,  /**< Cyclic telemetry (every 200 ms). Payload: 21 bytes fixed.    */
    ESPIF_MSG_MODEM_CONFIG      = 0x20u,  /**< APN + SIM PIN. Payload: variable.              */
    ESPIF_MSG_LTE_ENABLE        = 0x21u,  /**< Start/stop LTE. Payload: 1 byte (0/1).         */
    ESPIF_MSG_MQTT_CONFIG       = 0x22u,  /**< Broker credentials. Payload: variable.         */
    ESPIF_MSG_MQTT_ENABLE       = 0x23u,  /**< Start/stop MQTT. Payload: 1 byte (0/1).        */
    ESPIF_MSG_MQTT_PUBLISH      = 0x24u,  /**< Outgoing topic+payload. Payload: variable.     */
    ESPIF_MSG_GPS_ENABLE        = 0x25u,  /**< Start/stop GPS. Payload: 1 byte (0/1).         */
    ESPIF_MSG_MQTT_SUBSCRIBE    = 0x26u,  /**< Subscribe to topic. Payload: qos(u8)+topic_len(u8)+topic. */
    ESPIF_MSG_MQTT_UNSUBSCRIBE  = 0x27u,  /**< Unsubscribe from topic. Payload: topic_len(u8)+topic.    */

    /* ESP → STM32H */
    ESPIF_MSG_MQTT_STATUS       = 0x30u,  /**< Broker state. Payload: 1 byte (0/1/2).         */
    ESPIF_MSG_MQTT_RECEIVED     = 0x31u,  /**< Incoming topic+payload. Payload: variable.     */
    ESPIF_MSG_TIME_SYNC         = 0x32u,  /**< RTC time sync. Payload: 7 bytes (year LE + month + day + hour + minute + second). */
    ESPIF_MSG_GPS_DATA          = 0x40u,  /**< GPS fix. Payload: 25 bytes fixed.              */
    ESPIF_MSG_MODEM_STATUS      = 0x50u,  /**< Modem state + IP. Payload: variable.           */
} EspInterface_MsgId_t;

/*==============================================================================================
** MQTT status values (ESPIF_MSG_MQTT_STATUS payload byte).
==============================================================================================*/
#define ESPIF_MQTT_STATUS_DISCONNECTED   0u
#define ESPIF_MQTT_STATUS_CONNECTING     1u
#define ESPIF_MQTT_STATUS_CONNECTED      2u

/*==============================================================================================
** Modem/LTE status values (ESPIF_MSG_MODEM_STATUS payload byte 0).
==============================================================================================*/
#define ESPIF_MODEM_STATUS_OFF           0u
#define ESPIF_MODEM_STATUS_CONNECTING    1u
#define ESPIF_MODEM_STATUS_CONNECTED     2u

/*==============================================================================================
** Fixed-format payload structs (packed, little-endian).
** Use memcpy to populate — never dereference a packed pointer to a misaligned buffer.
==============================================================================================*/

/** ESPIF_MSG_STATIC_INFO payload — exactly 17 bytes. Sent once after startup. */
typedef struct __attribute__((packed))
{
    uint32_t slot1;           /**< Module article number in slot 1 (0 = empty)        */
    uint8_t  slot1_sw_major;  /**< Slot 1 module software major version               */
    uint8_t  slot1_sw_minor;  /**< Slot 1 module software minor version               */
    uint8_t  slot1_sw_patch;  /**< Slot 1 module software patch version               */
    uint32_t slot2;           /**< Module article number in slot 2 (0 = empty)        */
    uint8_t  slot2_sw_major;  /**< Slot 2 module software major version               */
    uint8_t  slot2_sw_minor;  /**< Slot 2 module software minor version               */
    uint8_t  slot2_sw_patch;  /**< Slot 2 module software patch version               */
    uint8_t  app_major;       /**< Application firmware major version                 */
    uint8_t  app_minor;       /**< Application firmware minor version                 */
    uint8_t  app_patch;       /**< Application firmware patch version                 */
} EspInterface_StaticInfo_t;

/** ESPIF_MSG_CYCLIC_INFO payload — exactly 21 bytes. Sent every 200 ms. */
typedef struct __attribute__((packed))
{
    uint16_t k15_mv;           /**< K15 (ignition) supply voltage in millivolts       */
    uint16_t k30_mv;           /**< K30 (battery)  supply voltage in millivolts       */
    int16_t  temperature_x10;  /**< ACC sensor temperature × 10 (e.g. 253 = 25.3 °C) */
    uint8_t  can1_bitrate;     /**< CAN1 bit rate: 1=125k 2=250k 3=500k 4=1M 0=unk   */
    uint8_t  can2_bitrate;     /**< CAN2 bit rate: 1=125k 2=250k 3=500k 4=1M 0=unk   */
    uint8_t  can1_busload;     /**< CAN1 bus load in percent (0–100; 0 = not measured)*/
    uint8_t  can2_busload;     /**< CAN2 bus load in percent (0–100; 0 = not measured)*/
    int16_t  accel_x;          /**< Accelerometer X axis in mg (signed)               */
    int16_t  accel_y;          /**< Accelerometer Y axis in mg (signed)               */
    int16_t  accel_z;          /**< Accelerometer Z axis in mg (signed)               */
    uint8_t  cpu_load;         /**< CPU load in percent (0–100)                       */
    uint16_t heap_available;   /**< Free FreeRTOS heap in bytes (capped at 65535)     */
    uint16_t stack_available;  /**< Model task stack high-water mark in bytes          */
} EspInterface_CyclicInfo_t;

/** ESPIF_MSG_GPS_DATA payload — exactly 25 bytes. */
typedef struct __attribute__((packed))
{
    float    latitude;   /**< Decimal degrees, positive = North */
    float    longitude;  /**< Decimal degrees, positive = East  */
    float    altitude;   /**< Metres above sea level            */
    float    speed;      /**< km/h                              */
    uint32_t utc_time;   /**< HHMMSS as integer                 */
    uint32_t utc_date;   /**< DDMMYY as integer                 */
    uint8_t  valid;      /**< 1 = valid GPS fix                 */
} EspInterface_GpsData_t;

/*==============================================================================================
** MQTT subscribe — per-block data struct.
** Allocate one statically per Subscribe block; pass a pointer to GO_communication_esp_mqtt_sub_register.
==============================================================================================*/
typedef struct
{
    const char        *topic;        /**< Pointer to static topic buffer (null-terminated).  */
    uint8_t           *payload;      /**< Pointer to static receive buffer.                  */
    uint16_t           payload_size; /**< Size of the receive buffer in bytes.               */
    volatile uint16_t  length;       /**< Number of valid bytes in the last received message. */
    volatile uint8_t   new_flag;     /**< Set to 1 by the driver when new data has arrived.   */
} EspInterface_MqttSubData_t;

/*==============================================================================================
** Public API — call from application code
==============================================================================================*/

/**************************************************************************************
** \brief     Initialise the EspInterface module. Stores the UART handle and starts
**            the first HAL_UART_Receive_IT call. Must be called after
**            MX_USARTx_UART_Init().
** \param     huart  Pointer to the HAL UART handle to use for ESP communication.
** \return    none
***************************************************************************************/
void GO_communication_esp_init(UART_HandleTypeDef *huart);

/**************************************************************************************
** \brief     Feed one received byte into the RX state machine. Call this from
**            HAL_UART_RxCpltCallback() when huart matches the configured UART.
**            The function re-arms HAL_UART_Receive_IT before returning.
** \param     huart  Pointer to the HAL UART handle that triggered the callback.
** \return    none
***************************************************************************************/
void GO_communication_esp_uart_rx_callback(UART_HandleTypeDef *huart);

/* --- STM32H → ESP frames --- */

/**************************************************************************************
** \brief     Gather module identity and application version data and send an
**            ESPIF_MSG_STATIC_INFO frame to the ESP. Call once after startup.
** \return    none
***************************************************************************************/
void GO_communication_esp_send_static_info(void);

/**************************************************************************************
** \brief     Gather cyclic telemetry (voltages, temperature, CAN bitrates, IMU,
**            CPU/heap/stack load) and send an ESPIF_MSG_CYCLIC_INFO frame to the ESP.
**            Call periodically, typically every 200 ms.
** \return    none
***************************************************************************************/
void GO_communication_esp_send_cyclic_info(void);

/**************************************************************************************
** \brief     Apply a time-sync payload received from the ESP to the STM32 RTC.
**            A weak default implementation is provided; override in application code
**            if additional post-sync actions are needed.
** \param     year    Full calendar year (e.g. 2025), little-endian on the wire.
** \param     month   Month of year (1–12).
** \param     day     Day of month (1–31).
** \param     hour    Hour of day (0–23).
** \param     minute  Minute of hour (0–59).
** \param     second  Second of minute (0–59).
** \return    none
***************************************************************************************/
void GO_communication_esp_on_time_sync(uint16_t year, uint8_t month, uint8_t day,
                                   uint8_t hour, uint8_t minute, uint8_t second);

/**************************************************************************************
** \brief     Send modem APN and optional SIM PIN to the ESP.
** \param     apn      Null-terminated APN string.
** \param     sim_pin  Null-terminated SIM PIN string, or NULL / "" when no PIN required.
** \return    none
***************************************************************************************/
void GO_communication_esp_set_modem_config(const char *apn, const char *sim_pin);

/**************************************************************************************
** \brief     Enable or disable the LTE modem on the ESP.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_lte(bool enable);

/**************************************************************************************
** \brief     Send MQTT broker credentials to the ESP. Changes take effect on the
**            ESP after a reconnect.
** \param     url         Null-terminated broker URL string.
** \param     port        Broker port number.
** \param     user        Null-terminated username string.
** \param     pass        Null-terminated password string.
** \param     client_id   Null-terminated client ID string.
** \param     keep_alive  Keep-alive interval in seconds.
** \return    none
***************************************************************************************/
void GO_communication_esp_set_mqtt_config(const char *url, uint16_t port,
                                      const char *user, const char *pass,
                                      const char *client_id, uint16_t keep_alive);

/**************************************************************************************
** \brief     Enable or disable the MQTT client on the ESP.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_mqtt(bool enable);

/**************************************************************************************
** \brief     Request the ESP to publish an MQTT message. topic must be
**            null-terminated. payload may contain binary data.
** \param     topic   Null-terminated topic string.
** \param     payload Pointer to the payload data.
** \param     len     Length of the payload in bytes.
** \param     qos     QoS level (0, 1 or 2).
** \param     retain  1 = broker retains the message for new subscribers, 0 = no retain.
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_publish(const char *topic, const uint8_t *payload,
                                    uint16_t len, uint8_t qos, uint8_t retain);

/**************************************************************************************
** \brief     Enable or disable the GPS receiver on the ESP/modem.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_gps(bool enable);

/**************************************************************************************
** \brief     Subscribe to an MQTT topic on the ESP.
** \param     topic  Null-terminated topic string (max 64 chars).
** \param     qos    QoS level (0, 1 or 2).
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_subscribe(const char *topic, uint8_t qos);

/**************************************************************************************
** \brief     Register a subscription data block with the driver. The driver will
**            fill sub->payload, sub->length and sub->new_flag whenever a message
**            arrives on sub->topic. Call once at model init before
**            GO_communication_esp_mqtt_subscribe. The pointer must remain valid for
**            the lifetime of the program.
** \param     sub  Pointer to the subscription data struct to register.
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_sub_register(EspInterface_MqttSubData_t *sub);

/**************************************************************************************
** \brief     Unsubscribe from an MQTT topic on the ESP.
** \param     topic  Null-terminated topic string (max 64 chars).
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_unsubscribe(const char *topic);

/*==============================================================================================
** Weak callbacks — override in application code to receive events from the ESP.
==============================================================================================*/

/**************************************************************************************
** \brief     Called when the ESP reports an MQTT broker connection state change.
** \param     status  ESPIF_MQTT_STATUS_DISCONNECTED / _CONNECTING / _CONNECTED.
** \return    none
***************************************************************************************/
void GO_communication_esp_on_mqtt_status(uint8_t status);

/**************************************************************************************
** \brief     Called when the ESP sends a GPS fix. gps is valid only for the
**            duration of the callback.
** \param     gps  Pointer to the GPS data struct.
** \return    none
***************************************************************************************/
void GO_communication_esp_on_gps_data(const EspInterface_GpsData_t *gps);

/**************************************************************************************
** \brief     Called when the ESP reports a modem/LTE state change.
** \param     state  ESPIF_MODEM_STATUS_OFF / _CONNECTING / _CONNECTED.
** \param     ip     Null-terminated IP string (empty string when not connected).
** \return    none
***************************************************************************************/
void GO_communication_esp_on_modem_status(uint8_t state, const char *ip);

#endif /* GOCONTROLL_IOT */

#endif /* GO_COMMUNICATION_ESP_H */
