/************************************************************************************//**
* \file         GO_communication_esp.c
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

#include "GO_communication_esp.h"

#ifdef GOCONTROLL_IOT

#include <string.h>
#include "rtc.h"
#include "GO_board.h"
#include "GO_controller_info.h"
#include "GO_communication_can.h"

/*==============================================================================================
** CRC-16/CCITT — polynomial 0x1021, init 0xFFFF, MSB-first, no reflection.
** Applied over: MSG_ID + LEN_LO + LEN_HI + PAYLOAD bytes.
==============================================================================================*/

static uint16_t Crc16Update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8u;
        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000u) ? ((crc << 1u) ^ 0x1021u) : (crc << 1u);
        }
    }
    return crc;
}

static uint16_t Crc16(const uint8_t *data, size_t len)
{
    return Crc16Update(0xFFFFu, data, len);
}

/*==============================================================================================
** Module-level state
==============================================================================================*/

static UART_HandleTypeDef *s_huart   = NULL;
static volatile uint8_t    s_tx_busy = 0u;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        s_tx_busy = 0u;
    }
}

/* MQTT subscription registry */
#define ESPIF_MQTT_SUB_MAX  8u
static EspInterface_MqttSubData_t *s_mqtt_subs[ESPIF_MQTT_SUB_MAX];
static uint8_t                     s_mqtt_nsubs = 0u;

/* Single-byte receive buffer for interrupt-driven RX */
static uint8_t s_rx_byte;

/* RX state machine */
typedef enum
{
    RX_SOF0 = 0,
    RX_SOF1,
    RX_MSG_ID,
    RX_LEN_LO,
    RX_LEN_HI,
    RX_PAYLOAD,
    RX_CRC_LO,
    RX_CRC_HI,
} RxState_t;

static RxState_t s_rx_state   = RX_SOF0;
static uint8_t   s_rx_msg_id  = 0u;
static uint16_t  s_rx_len     = 0u;
static uint16_t  s_rx_idx     = 0u;
static uint16_t  s_rx_crc_rx  = 0u;
static uint8_t   s_rx_payload[ESPIF_MAX_PAYLOAD_LEN];

/*==============================================================================================
** TX — build and send a framed message (blocking, 100 ms timeout).
** Frame layout: [SOF0][SOF1][MSG_ID][LEN_LO][LEN_HI][PAYLOAD...][CRC_LO][CRC_HI]
** CRC covers:   MSG_ID + LEN_LO + LEN_HI + PAYLOAD
**
** The complete frame is assembled into a single static buffer and sent with one
** HAL_UART_Transmit call.  This eliminates intra-frame gaps between header, payload
** and CRC that could occur when using multiple back-to-back transmit calls.
==============================================================================================*/

static void SendFrame(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    if (s_huart == NULL || len > ESPIF_MAX_PAYLOAD_LEN) { return; }
    if (s_tx_busy) { return; }   /* previous frame still transmitting — drop */

    static uint8_t s_tx_buf[ESPIF_FRAME_OVERHEAD + ESPIF_MAX_PAYLOAD_LEN];
    uint16_t n = 0u;

    s_tx_buf[n++] = ESPIF_SOF0;
    s_tx_buf[n++] = ESPIF_SOF1;
    s_tx_buf[n++] = msg_id;
    s_tx_buf[n++] = (uint8_t)(len & 0xFFu);
    s_tx_buf[n++] = (uint8_t)(len >> 8u);

    if (len > 0u && payload != NULL)
    {
        memcpy(s_tx_buf + n, payload, len);
        n += len;
    }

    /* CRC over bytes 2..4+len  (MSG_ID + LEN_LO + LEN_HI + PAYLOAD) */
    uint16_t crc = Crc16(s_tx_buf + 2u, 3u + len);
    s_tx_buf[n++] = (uint8_t)(crc & 0xFFu);
    s_tx_buf[n++] = (uint8_t)(crc >> 8u);

    s_tx_busy = 1u;
    if (HAL_UART_Transmit_IT(s_huart, s_tx_buf, n) != HAL_OK) {
        s_tx_busy = 0u;
    }
}

static void SendAck(uint8_t msg_id)
{
    SendFrame(ESPIF_MSG_ACK, &msg_id, 1u);
}

/* Forward declaration — defined below in the public API section. */
static void EspInterface_OnMqttReceived(const char *topic,
                                        const uint8_t *payload, uint16_t len);

/*==============================================================================================
** Frame dispatcher — called after a complete, CRC-verified frame is received.
==============================================================================================*/

static void DispatchFrame(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    switch ((EspInterface_MsgId_t)msg_id)
    {
        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_HEARTBEAT:
            SendAck(ESPIF_MSG_HEARTBEAT);
            break;

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_ACK:
            /* Received ACK from ESP — no action needed. */
            break;

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_MQTT_STATUS:
            if (len == 1u)
            {
                GO_communication_esp_on_mqtt_status(payload[0]);
            }
            break;

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_MQTT_RECEIVED:
        {
            if (len < 3u) { break; }
            uint8_t topic_len = payload[0];
            size_t  off       = 1u;
            if (off + topic_len + 2u > len) { break; }

            char topic[65] = {0};
            uint8_t copy_len = (topic_len > 64u) ? 64u : topic_len;
            memcpy(topic, payload + off, copy_len);
            off += topic_len;

            uint16_t plen = (uint16_t)payload[off] | ((uint16_t)payload[off + 1u] << 8u);
            off += 2u;
            if (off + plen > len) { break; }

            EspInterface_OnMqttReceived(topic, payload + off, plen);
            break;
        }

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_TIME_SYNC:
            if (len == 7u)
            {
                uint16_t year   = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8u);
                uint8_t  month  = payload[2];
                uint8_t  day    = payload[3];
                uint8_t  hour   = payload[4];
                uint8_t  minute = payload[5];
                uint8_t  second = payload[6];
                GO_communication_esp_on_time_sync(year, month, day, hour, minute, second);
                SendAck(ESPIF_MSG_TIME_SYNC);
            }
            break;

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_GPS_DATA:
            if (len == sizeof(EspInterface_GpsData_t))
            {
                EspInterface_GpsData_t gps;
                memcpy(&gps, payload, sizeof(gps));
                GO_communication_esp_on_gps_data(&gps);
            }
            break;

        /* ------------------------------------------------------------------ */
        case ESPIF_MSG_MODEM_STATUS:
        {
            if (len < 2u) { break; }
            uint8_t state  = payload[0];
            uint8_t ip_len = payload[1];
            char    ip[16] = {0};
            if (ip_len > 0u && 2u + ip_len <= len)
            {
                uint8_t copy_len = (ip_len > 15u) ? 15u : ip_len;
                memcpy(ip, payload + 2u, copy_len);
            }
            GO_communication_esp_on_modem_status(state, ip);
            break;
        }

        /* ------------------------------------------------------------------ */
        default:
            /* Unknown MSG_ID — no ACK sent. */
            break;
    }
}

/*==============================================================================================
** RX state machine — driven byte-by-byte from GO_communication_esp_uart_rx_callback().
==============================================================================================*/

static void ProcessRxByte(uint8_t byte)
{
    switch (s_rx_state)
    {
        case RX_SOF0:
            if (byte == ESPIF_SOF0) { s_rx_state = RX_SOF1; }
            break;

        case RX_SOF1:
            s_rx_state = (byte == ESPIF_SOF1) ? RX_MSG_ID : RX_SOF0;
            break;

        case RX_MSG_ID:
            s_rx_msg_id = byte;
            s_rx_state  = RX_LEN_LO;
            break;

        case RX_LEN_LO:
            s_rx_len   = (uint16_t)byte;
            s_rx_state = RX_LEN_HI;
            break;

        case RX_LEN_HI:
            s_rx_len |= (uint16_t)byte << 8u;
            s_rx_idx  = 0u;
            if (s_rx_len > ESPIF_MAX_PAYLOAD_LEN)
            {
                /* Payload too large — discard frame */
                s_rx_state = RX_SOF0;
                break;
            }
            s_rx_state = (s_rx_len > 0u) ? RX_PAYLOAD : RX_CRC_LO;
            break;

        case RX_PAYLOAD:
            s_rx_payload[s_rx_idx++] = byte;
            if (s_rx_idx >= s_rx_len) { s_rx_state = RX_CRC_LO; }
            break;

        case RX_CRC_LO:
            s_rx_crc_rx = (uint16_t)byte;
            s_rx_state  = RX_CRC_HI;
            break;

        case RX_CRC_HI:
        {
            s_rx_crc_rx |= (uint16_t)byte << 8u;

            /* Verify CRC over MSG_ID + LEN_LO + LEN_HI + PAYLOAD */
            uint8_t  hdr[3] = { s_rx_msg_id,
                                 (uint8_t)(s_rx_len & 0xFFu),
                                 (uint8_t)(s_rx_len >> 8u) };
            uint16_t exp_crc = Crc16(hdr, sizeof(hdr));
            if (s_rx_len > 0u)
            {
                exp_crc = Crc16Update(exp_crc, s_rx_payload, s_rx_len);
            }

            if (s_rx_crc_rx == exp_crc)
            {
                DispatchFrame(s_rx_msg_id, s_rx_payload, s_rx_len);
            }
            /* CRC mismatch: silently discard frame */
            s_rx_state = RX_SOF0;
            break;
        }

        default:
            s_rx_state = RX_SOF0;
            break;
    }
}

/*==============================================================================================
** Public API implementation
==============================================================================================*/

/**************************************************************************************
** \brief     Initialise the EspInterface module. Stores the UART handle and starts
**            the first HAL_UART_Receive_IT call. Must be called after
**            MX_USARTx_UART_Init().
** \param     huart  Pointer to the HAL UART handle to use for ESP communication.
** \return    none
***************************************************************************************/
void GO_communication_esp_init(UART_HandleTypeDef *huart)
{
    s_huart    = huart;
    s_rx_state = RX_SOF0;
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

/**************************************************************************************
** \brief     Feed one received byte into the RX state machine. Call this from
**            HAL_UART_RxCpltCallback() when huart matches the configured UART.
**            The function re-arms HAL_UART_Receive_IT before returning.
** \param     huart  Pointer to the HAL UART handle that triggered the callback.
** \return    none
***************************************************************************************/
void GO_communication_esp_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) { return; }
    ProcessRxByte(s_rx_byte);
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

/* --- STM32H → ESP send functions --- */

/**************************************************************************************
** \brief     Gather module identity and application version data and send an
**            ESPIF_MSG_STATIC_INFO frame to the ESP. Call once after startup.
** \return    none
***************************************************************************************/
void GO_communication_esp_send_static_info(void)
{

    EspInterface_StaticInfo_t info;
    _moduleInfo mod;
    _modelVersion ver;

    if (GO_controller_info_get_module_info(0, &mod) == 0)
    {
		
        info.slot1          = mod.article_number;
        info.slot1_sw_major = mod.sw_major;
        info.slot1_sw_minor = mod.sw_minor;
        info.slot1_sw_patch = mod.sw_patch;
		
    }
    else
    {
        info.slot1 = 0u; info.slot1_sw_major = 0u;
        info.slot1_sw_minor = 0u; info.slot1_sw_patch = 0u;
    }

    if (GO_controller_info_get_module_info(1, &mod) == 0)
    {
        info.slot2          = mod.article_number;
        info.slot2_sw_major = mod.sw_major;
        info.slot2_sw_minor = mod.sw_minor;
        info.slot2_sw_patch = mod.sw_patch;
    }
    else
    {
        info.slot2 = 0u; info.slot2_sw_major = 0u;
        info.slot2_sw_minor = 0u; info.slot2_sw_patch = 0u;
    }

    GO_controller_info_get_model_version(&ver);
    info.app_major = ver.major;
    info.app_minor = ver.minor;
    info.app_patch = ver.patch;
	


    SendFrame(ESPIF_MSG_STATIC_INFO, (const uint8_t *)&info, sizeof(info));
}

/**************************************************************************************
** \brief     Gather application configuration and send an ESPIF_MSG_APP_CONFIG frame.
**            Call once after startup, immediately after send_static_info.
** \return    none
***************************************************************************************/
void GO_communication_esp_send_app_config(void)
{
    const _appConfig *cfg = GO_controller_info_get_app_config();

    uint8_t url_len = (uint8_t)strnlen(cfg->distribution_url, 255u);

    uint16_t payload_len = (uint16_t)(sizeof(EspInterface_AppConfig_t) + url_len);
    uint8_t  payload[sizeof(EspInterface_AppConfig_t) + 256u];

    EspInterface_AppConfig_t *hdr = (EspInterface_AppConfig_t *)payload;
    memcpy(hdr->app_id, cfg->app_id, 8u);
    hdr->signing_enabled = cfg->signing_enabled;
    memcpy(hdr->public_key, cfg->public_key, 65u);
    hdr->latest_only = cfg->latest_only;
    hdr->url_len = url_len;
    if (url_len > 0u) {
        memcpy(payload + sizeof(EspInterface_AppConfig_t),
               cfg->distribution_url, url_len);
    }

    SendFrame(ESPIF_MSG_APP_CONFIG, payload, payload_len);
}

/**************************************************************************************
** \brief     Gather cyclic telemetry (voltages, temperature, CAN bitrates, IMU,
**            CPU/heap/stack load) and send an ESPIF_MSG_CYCLIC_INFO frame to the ESP.
**            Call periodically, typically every 200 ms.
** \return    none
***************************************************************************************/
void GO_communication_esp_send_cyclic_info(void)
{
    EspInterface_CyclicInfo_t info;
    GOcontrollControllerInfo_t imu;
    uint16_t k15_mv = 0u, k30_mv = 0u;
    uint32_t heap, stack;

    GO_board_controller_power_voltage(2u, &k15_mv);
    GO_board_controller_power_voltage(1u, &k30_mv);
    GO_board_controller_info_get_data(&imu);

    info.k15_mv          = k15_mv;
    info.k30_mv          = k30_mv;
    info.temperature_x10 = (int16_t)(imu.temp * 10.0f);
    info.can1_bitrate    = can_get_esp_bitrate(1u);
    info.can2_bitrate    = can_get_esp_bitrate(2u);
    info.can1_busload    = can_get_busload(1u);
    info.can2_busload    = can_get_busload(2u);
    info.accel_x         = (int16_t)imu.acc_x;
    info.accel_y         = (int16_t)imu.acc_y;
    info.accel_z         = (int16_t)imu.acc_z;
    info.cpu_load        = GO_controller_info_get_cpu_load();

    heap  = GO_controller_info_get_free_heap();
    stack = GO_controller_info_get_model_stack();
    info.heap_available  = (heap  > 0xFFFFu) ? 0xFFFFu : (uint16_t)heap;
    info.stack_available = (stack > 0xFFFFu) ? 0xFFFFu : (uint16_t)stack;

    SendFrame(ESPIF_MSG_CYCLIC_INFO, (const uint8_t *)&info, sizeof(info));
}

/**************************************************************************************
** \brief     Send modem APN and optional SIM PIN to the ESP.
** \param     apn      Null-terminated APN string.
** \param     sim_pin  Null-terminated SIM PIN string, or NULL / "" when no PIN required.
** \return    none
***************************************************************************************/
void GO_communication_esp_set_modem_config(const char *apn, const char *sim_pin)
{
    if (apn == NULL) { return; }

    static uint8_t buf[2u + 64u + 1u + 16u]; /* apn_len + apn(max64) + pin_len + pin(max16) */
    size_t  off = 0u;

    uint8_t apn_len = (uint8_t)strnlen(apn, 64u);
    buf[off++] = apn_len;
    memcpy(buf + off, apn, apn_len);
    off += apn_len;

    uint8_t pin_len = 0u;
    if (sim_pin != NULL && sim_pin[0] != '\0')
    {
        pin_len = (uint8_t)strnlen(sim_pin, 16u);
    }
    buf[off++] = pin_len;
    if (pin_len > 0u)
    {
        memcpy(buf + off, sim_pin, pin_len);
        off += pin_len;
    }

    SendFrame(ESPIF_MSG_MODEM_CONFIG, buf, (uint16_t)off);
}

/**************************************************************************************
** \brief     Enable or disable the LTE modem on the ESP.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_lte(bool enable)
{
    uint8_t val = enable ? 1u : 0u;
    SendFrame(ESPIF_MSG_LTE_ENABLE, &val, 1u);
}

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
                                  const char *client_id, uint16_t keep_alive)
{
    /* Layout: port(u16-LE) + keep_alive(u16-LE) + url_len(u8) + url
     *         + user_len(u8) + user + pass_len(u8) + pass
     *         + client_id_len(u8) + client_id                          */
    static uint8_t buf[4u + 1u + 128u + 1u + 64u + 1u + 64u + 1u + 64u];
    size_t  off = 0u;

    buf[off++] = (uint8_t)(port & 0xFFu);
    buf[off++] = (uint8_t)(port >> 8u);
    buf[off++] = (uint8_t)(keep_alive & 0xFFu);
    buf[off++] = (uint8_t)(keep_alive >> 8u);

#define APPEND_STR(str, maxlen)                                         \
    do {                                                                \
        const char *_s  = ((str) != NULL) ? (str) : "";                \
        uint8_t     _sl = (uint8_t)strnlen(_s, (maxlen));              \
        buf[off++] = _sl;                                               \
        memcpy(buf + off, _s, _sl);                                     \
        off += _sl;                                                     \
    } while (0)

    APPEND_STR(url,       128u);
    APPEND_STR(user,      64u);
    APPEND_STR(pass,      64u);
    APPEND_STR(client_id, 64u);
#undef APPEND_STR

    SendFrame(ESPIF_MSG_MQTT_CONFIG, buf, (uint16_t)off);
}

/**************************************************************************************
** \brief     Enable or disable the MQTT client on the ESP.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_mqtt(bool enable)
{
    uint8_t val = enable ? 1u : 0u;
    SendFrame(ESPIF_MSG_MQTT_ENABLE, &val, 1u);
}

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
                               uint16_t len, uint8_t qos, uint8_t retain)
{
    if (topic == NULL) { return; }

    /* Layout: qos(u8) + retain(u8) + topic_len(u8) + topic + payload_len(u16-LE) + payload */
    uint8_t topic_len = (uint8_t)strnlen(topic, 64u);
    uint16_t total    = 1u + 1u + 1u + topic_len + 2u + len;
    if (total > ESPIF_MAX_PAYLOAD_LEN) { return; }

    static uint8_t buf[ESPIF_MAX_PAYLOAD_LEN];
    size_t  off = 0u;
    buf[off++] = qos;
    buf[off++] = retain;
    buf[off++] = topic_len;
    memcpy(buf + off, topic, topic_len);
    off += topic_len;
    buf[off++] = (uint8_t)(len & 0xFFu);
    buf[off++] = (uint8_t)(len >> 8u);
    if (len > 0u && payload != NULL)
    {
        memcpy(buf + off, payload, len);
    }

    SendFrame(ESPIF_MSG_MQTT_PUBLISH, buf, total);
}

/**************************************************************************************
** \brief     Enable or disable the GPS receiver on the ESP/modem.
** \param     enable  true to enable, false to disable.
** \return    none
***************************************************************************************/
void GO_communication_esp_enable_gps(bool enable)
{
    uint8_t val = enable ? 1u : 0u;
    SendFrame(ESPIF_MSG_GPS_ENABLE, &val, 1u);
}

/**************************************************************************************
** \brief     Subscribe to an MQTT topic on the ESP.
** \param     topic  Null-terminated topic string (max 64 chars).
** \param     qos    QoS level (0, 1 or 2).
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_subscribe(const char *topic, uint8_t qos)
{
    if (topic == NULL) { return; }
    /* Layout: qos(u8) + topic_len(u8) + topic */
    uint8_t topic_len = (uint8_t)strnlen(topic, 64u);
    uint8_t buf[2u + 64u];
    buf[0] = qos;
    buf[1] = topic_len;
    memcpy(buf + 2u, topic, topic_len);
    SendFrame(ESPIF_MSG_MQTT_SUBSCRIBE, buf, (uint16_t)(2u + topic_len));
}

/**************************************************************************************
** \brief     Register a subscription data block with the driver. The driver will
**            fill sub->payload, sub->length and sub->new_flag whenever a message
**            arrives on sub->topic. Call once at model init before
**            GO_communication_esp_mqtt_subscribe. The pointer must remain valid for
**            the lifetime of the program.
** \param     sub  Pointer to the subscription data struct to register.
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_sub_register(EspInterface_MqttSubData_t *sub)
{
    if (sub != NULL && s_mqtt_nsubs < ESPIF_MQTT_SUB_MAX)
    {
        s_mqtt_subs[s_mqtt_nsubs++] = sub;
    }
}

/**************************************************************************************
** \brief     Unsubscribe from an MQTT topic on the ESP.
** \param     topic  Null-terminated topic string (max 64 chars).
** \return    none
***************************************************************************************/
void GO_communication_esp_mqtt_unsubscribe(const char *topic)
{
    if (topic == NULL) { return; }
    /* Layout: topic_len(u8) + topic */
    uint8_t topic_len = (uint8_t)strnlen(topic, 64u);
    uint8_t buf[1u + 64u];
    buf[0] = topic_len;
    memcpy(buf + 1u, topic, topic_len);
    SendFrame(ESPIF_MSG_MQTT_UNSUBSCRIBE, buf, (uint16_t)(1u + topic_len));
}

/*==============================================================================================
** HAL UART callback — routes incoming bytes to the EspInterface RX state machine.
** Called from the UART interrupt (HAL_UART_IRQHandler) after each received byte.
** Re-arming of HAL_UART_Receive_IT is handled inside GO_communication_esp_uart_rx_callback.
==============================================================================================*/

/**************************************************************************************
** \brief     HAL UART receive complete callback. Routes the received byte to the
**            ESP RX state machine. Called automatically by HAL after each byte.
** \param     huart  Pointer to the HAL UART handle that completed reception.
** \return    none
***************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    GO_communication_esp_uart_rx_callback(huart);
}

/*==============================================================================================
** Weak callbacks — override in application code to handle events from the ESP.
==============================================================================================*/

/**************************************************************************************
** \brief     Called when the ESP reports an MQTT broker connection state change.
** \param     status  ESPIF_MQTT_STATUS_DISCONNECTED / _CONNECTING / _CONNECTED.
** \return    none
***************************************************************************************/
__attribute__((weak)) void GO_communication_esp_on_mqtt_status(uint8_t status)
{
    (void)status;
}

/**************************************************************************************
** \brief     Internal static handler dispatched when an MQTT_RECEIVED frame arrives.
**            Matches the topic against registered subscriptions and fills their buffers.
** \param     topic    Null-terminated topic string from the received frame.
** \param     payload  Pointer to the received payload bytes.
** \param     len      Number of payload bytes.
** \return    none
***************************************************************************************/
static void EspInterface_OnMqttReceived(const char *topic,
                                        const uint8_t *payload, uint16_t len)
{
    uint8_t i;
    for (i = 0u; i < s_mqtt_nsubs; i++)
    {
        if (strncmp(s_mqtt_subs[i]->topic, topic, 64u) == 0)
        {
            uint16_t n = (len < s_mqtt_subs[i]->payload_size) ? len
                                                               : s_mqtt_subs[i]->payload_size;
            memcpy(s_mqtt_subs[i]->payload, payload, n);
            s_mqtt_subs[i]->length   = n;
            s_mqtt_subs[i]->new_flag = 1u;
        }
    }
}

/**************************************************************************************
** \brief     Called when the ESP sends a GPS fix. gps is valid only for the
**            duration of the callback.
** \param     gps  Pointer to the GPS data struct.
** \return    none
***************************************************************************************/
__attribute__((weak)) void GO_communication_esp_on_gps_data(const EspInterface_GpsData_t *gps)
{
    (void)gps;
}

/**************************************************************************************
** \brief     Called when the ESP reports a modem/LTE state change.
** \param     state  ESPIF_MODEM_STATUS_OFF / _CONNECTING / _CONNECTED.
** \param     ip     Null-terminated IP string (empty string when not connected).
** \return    none
***************************************************************************************/
__attribute__((weak)) void GO_communication_esp_on_modem_status(uint8_t state, const char *ip)
{
    (void)state;
    (void)ip;
}

/**************************************************************************************
** \brief     Apply a time-sync payload received from the ESP to the STM32 RTC.
**            Override in application code if additional post-sync actions are needed.
** \param     year    Full calendar year (e.g. 2025).
** \param     month   Month of year (1–12).
** \param     day     Day of month (1–31).
** \param     hour    Hour of day (0–23).
** \param     minute  Minute of hour (0–59).
** \param     second  Second of minute (0–59).
** \return    none
***************************************************************************************/
__attribute__((weak)) void GO_communication_esp_on_time_sync(
    uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint8_t second)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours          = hour;
    sTime.Minutes        = minute;
    sTime.Seconds        = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    sDate.WeekDay = RTC_WEEKDAY_MONDAY; /* day-of-week not in TIME_SYNC payload */
    sDate.Month   = month;
    sDate.Date    = day;
    sDate.Year    = (uint8_t)((year > 2000u) ? (year - 2000u) : 0u);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

#endif /* GOCONTROLL_IOT */
