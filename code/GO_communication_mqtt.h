/**************************************************************************************
 * \file   GO_communication_mqtt.h
 * \brief  Platform-independent MQTT client API for GOcontroll hardware.
 *
 *         Provides identical function calls on all supported platforms:
 *           GOCONTROLL_LINUX  — Moduline IV / Moduline Mini
 *                               Backend: libmosquitto (background thread)
 *           GOCONTROLL_IOT    — Moduline IOT (STM32H5)
 *                               Backend: GO_communication_esp (ESP32 co-processor)
 *
 *         Typical usage:
 *
 *           // 1. Register subscription buffers before enabling
 *           static uint8_t rx_buf[128];
 *           static GO_MqttSubData_t sub = {
 *               .topic        = "/cmd/output",
 *               .payload      = rx_buf,
 *               .payload_size = sizeof(rx_buf),
 *           };
 *           GO_communication_mqtt_sub_register(&sub);
 *
 *           // 2. Configure and connect
 *           GO_communication_mqtt_configure("192.168.1.10", 1883,
 *                                           "user", "pass",
 *                                           "moduline-01", 60);
 *           GO_communication_mqtt_subscribe("/cmd/output", 1);
 *           GO_communication_mqtt_enable(true);
 *
 *           // 3. In the application loop — publish and check incoming
 *           GO_communication_mqtt_publish("/sensor/ch1", data, len, 1, 0);
 *           if (sub.new_flag) {
 *               sub.new_flag = 0;
 *               // process sub.payload[0..sub.length-1]
 *           }
 *
 * \note   On Linux the MQTT network loop runs in a background thread started by
 *         GO_communication_mqtt_enable(true). No polling is needed; the driver
 *         writes directly to the registered GO_MqttSubData_t buffers.
 **************************************************************************************/

#ifndef GO_COMMUNICATION_MQTT_H
#define GO_COMMUNICATION_MQTT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================================
** Connection status values — returned via GO_communication_mqtt_on_status().
==============================================================================================*/
#define GO_MQTT_STATUS_DISCONNECTED  0u  /**< Not connected to broker.           */
#define GO_MQTT_STATUS_CONNECTING    1u  /**< Connection attempt in progress.     */
#define GO_MQTT_STATUS_CONNECTED     2u  /**< Successfully connected to broker.   */

/*==============================================================================================
** Subscription data block.
** Allocate one statically per topic; pass a pointer to GO_communication_mqtt_sub_register().
** The driver fills payload, length and new_flag when a matching message arrives.
** new_flag must be cleared by the application after reading.
**
** \note  The struct layout is intentionally identical to EspInterface_MqttSubData_t so
**        that on GOCONTROLL_IOT the pointer can be passed directly to the ESP driver
**        with a simple cast — no copying required.
==============================================================================================*/
typedef struct
{
    const char        *topic;        /**< Null-terminated topic string (pointer must remain valid). */
    uint8_t           *payload;      /**< Pointer to static receive buffer supplied by caller.      */
    uint16_t           payload_size; /**< Size of the receive buffer in bytes.                      */
    volatile uint16_t  length;       /**< Number of valid bytes in the last received message.       */
    volatile uint8_t   new_flag;     /**< Set to 1 by the driver when new data has arrived.         */
} GO_MqttSubData_t;

/*==============================================================================================
** Public API
==============================================================================================*/

/**************************************************************************************
 * \brief  Store broker connection parameters. Must be called before
 *         GO_communication_mqtt_enable(true).
 *
 * \param  url         Broker hostname or IP address (null-terminated).
 * \param  port        Broker port (typically 1883, or 8883 for TLS).
 * \param  user        Username string, or NULL / "" for anonymous access.
 * \param  pass        Password string, or NULL / "" when unused.
 * \param  client_id   Client identifier string (null-terminated). Pass NULL or ""
 *                     to let the library generate one automatically.
 * \param  keep_alive  Keep-alive interval in seconds (typical: 60).
 **************************************************************************************/
void GO_communication_mqtt_configure(const char *url, uint16_t port,
                                     const char *user, const char *pass,
                                     const char *client_id, uint16_t keep_alive);

/**************************************************************************************
 * \brief  Connect to or disconnect from the broker.
 *
 *         enable=true:  Establishes the connection using the parameters supplied to
 *                       GO_communication_mqtt_configure(). On Linux a background
 *                       network thread is started automatically.
 *         enable=false: Gracefully disconnects and frees resources.
 *
 * \param  enable  true to connect, false to disconnect.
 **************************************************************************************/
void GO_communication_mqtt_enable(bool enable);

/**************************************************************************************
 * \brief  Publish a message to a topic.
 *
 * \param  topic    Null-terminated topic string.
 * \param  payload  Pointer to payload data (may contain binary data).
 * \param  len      Length of payload in bytes.
 * \param  qos      QoS level: 0, 1 or 2.
 * \param  retain   1 = broker retains message for new subscribers, 0 = no retain.
 **************************************************************************************/
void GO_communication_mqtt_publish(const char *topic, const uint8_t *payload,
                                   uint16_t len, uint8_t qos, uint8_t retain);

/**************************************************************************************
 * \brief  Subscribe to a topic. A matching GO_MqttSubData_t must be registered first
 *         via GO_communication_mqtt_sub_register() for data to be delivered.
 *
 * \param  topic  Null-terminated topic string (wildcards + and # are supported).
 * \param  qos    QoS level: 0, 1 or 2.
 **************************************************************************************/
void GO_communication_mqtt_subscribe(const char *topic, uint8_t qos);

/**************************************************************************************
 * \brief  Register a subscription data block. Call once before
 *         GO_communication_mqtt_subscribe(). The pointer must remain valid for the
 *         lifetime of the program.
 *
 *         When a message arrives on sub->topic the driver writes up to
 *         sub->payload_size bytes into sub->payload, sets sub->length and
 *         sets sub->new_flag to 1.
 *
 * \param  sub  Pointer to a statically allocated GO_MqttSubData_t.
 **************************************************************************************/
void GO_communication_mqtt_sub_register(GO_MqttSubData_t *sub);

/**************************************************************************************
 * \brief  Unsubscribe from a topic.
 *
 * \param  topic  Null-terminated topic string.
 **************************************************************************************/
void GO_communication_mqtt_unsubscribe(const char *topic);

/*==============================================================================================
** Weak callback — override in application code to react to connection state changes.
==============================================================================================*/

/**************************************************************************************
 * \brief  Called when the broker connection state changes.
 *         A default no-op implementation is provided; override to add behaviour.
 *
 * \param  status  GO_MQTT_STATUS_DISCONNECTED / _CONNECTING / _CONNECTED.
 **************************************************************************************/
void GO_communication_mqtt_on_status(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* GO_COMMUNICATION_MQTT_H */
