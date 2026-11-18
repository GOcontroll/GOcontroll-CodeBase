/**************************************************************************************
 * \file   GO_communication_mqtt.h
 * \brief  Platform-independent MQTT client API for GOcontroll hardware.
 *
 *         Provides identical function calls on all supported platforms:
 *           GOCONTROLL_LINUX  — Moduline L4 / Moduline M1
 *                               Backend: libmosquitto (background thread)
 *           GOCONTROLL_STM    — Moduline S1 (STM32H5)
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
 *           GO_communication_mqtt_poll();   // switch on the receive queue NOW, before
 *                                           // subscribing — see the queue note below
 *
 *           // 2. Configure and connect
 *           GO_communication_mqtt_configure("192.168.1.10", 1883,
 *                                           "user", "pass",
 *                                           "moduline-01", 60);
 *           GO_communication_mqtt_subscribe("/cmd/output", 1);
 *           GO_communication_mqtt_enable(true);
 *
 *           // 3. In the application loop — publish and handle EVERY incoming message
 *           GO_communication_mqtt_publish("/sensor/ch1", data, len, 1, 0);
 *           GO_communication_mqtt_poll();
 *           while (sub.new_flag) {
 *               // process sub.payload[0..sub.length-1]
 *               sub.new_flag = 0;
 *               GO_communication_mqtt_poll();   // next queued message, if any
 *           }
 *
 * \note   Receive queue. Once GO_communication_mqtt_poll() has been called, every
 *         message is queued per subscription (GO_MQTT_RX_QUEUE_DEPTH, default 8) and
 *         poll() — in the caller's thread — moves the next one into the buffer when
 *         new_flag is 0. Nothing is lost while the application is busy, and the
 *         buffer is never written from another thread. When a queue is full the
 *         OLDEST message is dropped and counted (GO_communication_mqtt_dropped()).
 *         Without poll() the old behaviour applies: the driver writes the buffer
 *         directly and the last message wins.
 *         Call poll() once BEFORE the first subscribe (step 1 above). If the first
 *         poll() only comes from the application loop, everything that arrives
 *         between subscribe and the first loop iteration — on Linux the module
 *         detection of a few seconds, plus a retained message on connect — still
 *         takes the old path and is not counted as dropped. Measured on an M1: 5/5
 *         at rest, 1/5 right after a restart, until poll() moved into the init.
 * \note   Linux: the network loop runs in a background thread started by
 *         GO_communication_mqtt_enable(true). S1: messages arrive from the UART
 *         interrupt of the ESP driver.
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
**        that on GOCONTROLL_STM the pointer can be passed directly to the ESP driver
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
 * \note   S1: these values go to the ESP (GO_communication_esp_set_mqtt_config) and prevail
 *         over a customer broker stored in the ESP during provisioning or with the Service
 *         Tool; with an empty url the ESP uses its stored one — see the note there.
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
 *         Linux: the QoS is remembered for that registration and reused when the
 *         client (re)connects, so it also applies when this is called before
 *         GO_communication_mqtt_enable(true) and after a broker reconnect.
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
 *         When a message arrives on sub->topic (wildcards + and # match), up to
 *         sub->payload_size bytes end up in sub->payload, sub->length is set and
 *         sub->new_flag becomes 1 — directly, or via GO_communication_mqtt_poll()
 *         once that is in use.
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

/**************************************************************************************
 * \brief  Move the next queued message of each subscription into its buffer.
 *         For every registered subscription whose new_flag is 0 and whose queue is
 *         not empty: copy the oldest message into payload/length and set new_flag.
 *         Call it once per application cycle, and again after clearing new_flag to
 *         get the next message (see the usage example at the top of this file).
 *         The first call switches the driver to queued delivery — make that first
 *         call before subscribing, so no message takes the old path.
 **************************************************************************************/
void GO_communication_mqtt_poll(void);

/**************************************************************************************
 * \brief  Number of messages dropped for this subscription because its queue was
 *         full (the oldest one goes). 0 for an unknown subscription.
 **************************************************************************************/
uint32_t GO_communication_mqtt_dropped(const GO_MqttSubData_t *sub);

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
