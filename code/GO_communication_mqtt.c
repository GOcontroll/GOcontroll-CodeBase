/**************************************************************************************
 * \file   GO_communication_mqtt.c
 * \brief  Platform-independent MQTT client — implementation.
 *
 *         Conditional compilation selects the correct backend:
 *           GOCONTROLL_LINUX  — libmosquitto, background network thread
 *           GOCONTROLL_STM    — GO_communication_esp (ESP32 co-processor via UART)
 **************************************************************************************/

#include "GO_communication_mqtt.h"
#include <string.h>

#define MQTT_MAX_SUBSCRIPTIONS  8u

/* ============================================================================
 * Receive queue — shared by both backends
 * ============================================================================
 *
 * Why: the driver used to copy every message straight into the one buffer of the
 * subscription and set new_flag, also when the application had not read the previous
 * one yet. A burst therefore kept only its last message (measured on an M1: 5 puts,
 * 1 processed), and the copy ran in the network thread (Linux) or the UART interrupt
 * (S1) while the application could be reading the same buffer.
 *
 * Now, once the application calls GO_communication_mqtt_poll(), every message goes
 * into a per-subscription FIFO and only poll() — in the application's own thread —
 * copies the next one into sub->payload. Writer and reader of the buffer are then
 * the same thread; the lock only guards the FIFO.
 *
 * Compatibility: until the first poll() nothing changes. An application that never
 * calls poll() keeps the old behaviour (the last message wins), so existing firmware
 * (Simulink blockset, hand-written apps) does not change silently.
 *
 * Full FIFO: the OLDEST message is dropped and counted (GO_communication_mqtt_dropped).
 * For a desired-state channel the newest message is the one that matters.
 */

#ifndef GO_MQTT_RX_QUEUE_DEPTH
#define GO_MQTT_RX_QUEUE_DEPTH  8u
#endif

/* Storage for all FIFO slots together, carved out per subscription at registration
 * (depth x payload_size). When it runs out, a subscription gets fewer slots — at
 * least one, or none: then that subscription stays on the old behaviour. */
#ifndef GO_MQTT_RX_POOL_BYTES
#ifdef GOCONTROLL_LINUX
#define GO_MQTT_RX_POOL_BYTES   65536u
#else
#define GO_MQTT_RX_POOL_BYTES   8192u
#endif
#endif

typedef struct {
    GO_MqttSubData_t *sub;
    uint8_t          *slots;   /* depth slots of sub->payload_size bytes */
    uint16_t          len[GO_MQTT_RX_QUEUE_DEPTH];
    uint8_t           depth;
    uint8_t           head;    /* oldest */
    uint8_t           count;
    uint32_t          dropped;
} mqtt_rx_queue_t;

static mqtt_rx_queue_t   s_rxq[MQTT_MAX_SUBSCRIPTIONS];
static uint8_t           s_rxq_count = 0u;
static uint8_t           s_rx_pool[GO_MQTT_RX_POOL_BYTES];
static uint32_t          s_rx_pool_used = 0u;
static volatile uint8_t  s_poll_active = 0u;

static uint32_t mqtt_rx_lock(void);
static void     mqtt_rx_unlock(uint32_t key);

/* MQTT topic filter match: '+' = exactly one level, '#' = this level and all below
 * (only as the last level). The old exact strcmp never matched a wildcard
 * subscription, although the header promised that wildcards work. */
static bool mqtt_topic_matches(const char *filter, const char *topic)
{
    if (filter == NULL || topic == NULL) { return false; }
    while (*filter != '\0') {
        if (filter[0] == '#' && filter[1] == '\0') { return true; }
        if (*filter == '+') {
            while (*topic != '\0' && *topic != '/') { topic++; }
            filter++;
        } else {
            if (*filter != *topic) {
                /* "a/#" also matches "a" itself (MQTT 3.1.1 §4.7.1.2) */
                return (*topic == '\0' && filter[0] == '/' && filter[1] == '#' && filter[2] == '\0');
            }
            filter++;
            topic++;
            continue;
        }
        if (*filter == '\0') { return *topic == '\0'; }
        if (*filter != '/' || *topic != '/') { return false; }
        filter++;
        topic++;
        /* "a/#" also matches "a" itself */
        if (filter[0] == '#' && filter[1] == '\0') { return true; }
    }
    return *topic == '\0';
}

static void mqtt_rx_register(GO_MqttSubData_t *sub)
{
    if (sub == NULL || s_rxq_count >= MQTT_MAX_SUBSCRIPTIONS) { return; }
    mqtt_rx_queue_t *q = &s_rxq[s_rxq_count++];
    memset(q, 0, sizeof(*q));
    q->sub = sub;
    uint32_t want = (uint32_t)sub->payload_size;
    uint32_t room = (want == 0u) ? 0u : (GO_MQTT_RX_POOL_BYTES - s_rx_pool_used) / want;
    q->depth = (uint8_t)((room > GO_MQTT_RX_QUEUE_DEPTH) ? GO_MQTT_RX_QUEUE_DEPTH : room);
    if (q->depth > 0u) {
        q->slots = &s_rx_pool[s_rx_pool_used];
        s_rx_pool_used += (uint32_t)q->depth * want;
    }
}

/* Deliver one received message. Returns true when the queue took it; false means
 * "not queued" and the caller writes the buffer the old way (no poll() yet, or no
 * FIFO for that subscription). Called from the network thread / UART interrupt. */
static bool mqtt_rx_deliver(const char *topic, const uint8_t *payload, uint16_t len)
{
    if (!s_poll_active) { return false; }
    bool taken = false;
    for (uint8_t i = 0u; i < s_rxq_count; i++) {
        mqtt_rx_queue_t *q = &s_rxq[i];
        if (q->depth == 0u || !mqtt_topic_matches(q->sub->topic, topic)) { continue; }
        uint16_t n = (len > q->sub->payload_size) ? q->sub->payload_size : len;
        uint32_t key = mqtt_rx_lock();
        if (q->count == q->depth) {               /* full: drop the oldest */
            q->head = (uint8_t)((q->head + 1u) % q->depth);
            q->count--;
            q->dropped++;
        }
        uint8_t slot = (uint8_t)((q->head + q->count) % q->depth);
        memcpy(q->slots + (uint32_t)slot * q->sub->payload_size, payload, n);
        q->len[slot] = n;
        q->count++;
        mqtt_rx_unlock(key);
        taken = true;
    }
    return taken;
}

void GO_communication_mqtt_poll(void)
{
    s_poll_active = 1u;
    for (uint8_t i = 0u; i < s_rxq_count; i++) {
        mqtt_rx_queue_t *q = &s_rxq[i];
        if (q->depth == 0u || q->sub->new_flag != 0u) { continue; }
        uint32_t key = mqtt_rx_lock();
        if (q->count > 0u) {
            memcpy(q->sub->payload, q->slots + (uint32_t)q->head * q->sub->payload_size, q->len[q->head]);
            q->sub->length   = q->len[q->head];
            q->head          = (uint8_t)((q->head + 1u) % q->depth);
            q->count--;
            q->sub->new_flag = 1u;
        }
        mqtt_rx_unlock(key);
    }
}

uint32_t GO_communication_mqtt_dropped(const GO_MqttSubData_t *sub)
{
    for (uint8_t i = 0u; i < s_rxq_count; i++) {
        if (s_rxq[i].sub == sub) { return s_rxq[i].dropped; }
    }
    return 0u;
}

/* ============================================================================
 * LINUX — libmosquitto
 * ========================================================================= */
#ifdef GOCONTROLL_LINUX

#include <mosquitto.h>
#include <pthread.h>

/* on_message runs in mosquitto's network thread, poll() in the application's. */
static pthread_mutex_t s_rx_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint32_t mqtt_rx_lock(void)          { pthread_mutex_lock(&s_rx_mutex); return 0u; }
static void     mqtt_rx_unlock(uint32_t key) { (void)key; pthread_mutex_unlock(&s_rx_mutex); }

static struct mosquitto *s_mosq      = NULL;
static char     s_url[128]           = {0};
static uint16_t s_port               = 1883u;
static char     s_user[64]           = {0};
static char     s_pass[64]           = {0};
static char     s_client_id[64]      = {0};
static uint16_t s_keep_alive         = 60u;

static GO_MqttSubData_t *s_subs[MQTT_MAX_SUBSCRIPTIONS];
static uint8_t           s_sub_count = 0u;
/* QoS per registered subscription, as last passed to GO_communication_mqtt_subscribe().
 * Kept beside s_subs[] instead of in GO_MqttSubData_t: on the S1 that struct is cast
 * 1-to-1 to the ESP driver's struct, so it cannot grow a field. */
static uint8_t           s_sub_qos[MQTT_MAX_SUBSCRIPTIONS];

static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)obj;
    if (rc == 0) {
        GO_communication_mqtt_on_status(GO_MQTT_STATUS_CONNECTED);
        /* (Re)subscribe everything on every connect: a clean session forgets the
         * subscriptions, and subscribe() calls made before enable() had no
         * connection to go to. Use the QoS each one was requested with. */
        for (uint8_t i = 0u; i < s_sub_count; i++) {
            mosquitto_subscribe(mosq, NULL, s_subs[i]->topic, (int)s_sub_qos[i]);
        }
    } else {
        GO_communication_mqtt_on_status(GO_MQTT_STATUS_DISCONNECTED);
    }
}

static void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)mosq; (void)obj; (void)rc;
    GO_communication_mqtt_on_status(GO_MQTT_STATUS_DISCONNECTED);
}

static void on_message(struct mosquitto *mosq, void *obj,
                       const struct mosquitto_message *msg)
{
    (void)mosq; (void)obj;
    if (msg == NULL || msg->topic == NULL || msg->payloadlen < 0) { return; }
    uint16_t len = (msg->payloadlen > 0xFFFF) ? 0xFFFFu : (uint16_t)msg->payloadlen;
    if (mqtt_rx_deliver(msg->topic, (const uint8_t *)msg->payload, len)) { return; }

    /* No poll() yet: the old behaviour — write the buffer directly, last message wins. */
    for (uint8_t i = 0u; i < s_sub_count; i++) {
        if (mqtt_topic_matches(s_subs[i]->topic, msg->topic)) {
            uint16_t n = (len > s_subs[i]->payload_size) ? s_subs[i]->payload_size : len;
            memcpy(s_subs[i]->payload, msg->payload, n);
            s_subs[i]->length   = n;
            s_subs[i]->new_flag = 1u;
        }
    }
}

void GO_communication_mqtt_configure(const char *url, uint16_t port,
                                     const char *user, const char *pass,
                                     const char *client_id, uint16_t keep_alive)
{
    strncpy(s_url,       url       ? url       : "", sizeof(s_url)       - 1u);
    strncpy(s_user,      user      ? user      : "", sizeof(s_user)      - 1u);
    strncpy(s_pass,      pass      ? pass      : "", sizeof(s_pass)      - 1u);
    strncpy(s_client_id, client_id ? client_id : "", sizeof(s_client_id) - 1u);
    s_port       = port;
    s_keep_alive = keep_alive;
}

void GO_communication_mqtt_enable(bool enable)
{
    if (enable) {
        mosquitto_lib_init();
        s_mosq = mosquitto_new(s_client_id[0] ? s_client_id : NULL, true, NULL);
        if (!s_mosq) { return; }
        if (s_user[0]) { mosquitto_username_pw_set(s_mosq, s_user, s_pass[0] ? s_pass : NULL); }
        mosquitto_connect_callback_set(s_mosq, on_connect);
        mosquitto_disconnect_callback_set(s_mosq, on_disconnect);
        mosquitto_message_callback_set(s_mosq, on_message);
        GO_communication_mqtt_on_status(GO_MQTT_STATUS_CONNECTING);
        mosquitto_connect(s_mosq, s_url, s_port, (int)s_keep_alive);
        mosquitto_loop_start(s_mosq);
    } else {
        if (s_mosq) {
            mosquitto_loop_stop(s_mosq, false);
            mosquitto_disconnect(s_mosq);
            mosquitto_destroy(s_mosq);
            s_mosq = NULL;
            mosquitto_lib_cleanup();
        }
        GO_communication_mqtt_on_status(GO_MQTT_STATUS_DISCONNECTED);
    }
}

void GO_communication_mqtt_publish(const char *topic, const uint8_t *payload,
                                   uint16_t len, uint8_t qos, uint8_t retain)
{
    if (!s_mosq) { return; }
    mosquitto_publish(s_mosq, NULL, topic, (int)len, payload, (int)qos, retain != 0u);
}

void GO_communication_mqtt_subscribe(const char *topic, uint8_t qos)
{
    /* Remember the QoS for the registered subscription on this topic, so the
     * resubscribe in on_connect() uses it too — also when this call comes before
     * enable(), as the documented setup order prescribes. */
    for (uint8_t i = 0u; i < s_sub_count; i++) {
        if (topic != NULL && strcmp(s_subs[i]->topic, topic) == 0) {
            s_sub_qos[i] = (qos > 2u) ? 2u : qos;
        }
    }
    if (!s_mosq) { return; }
    mosquitto_subscribe(s_mosq, NULL, topic, (int)qos);
}

void GO_communication_mqtt_sub_register(GO_MqttSubData_t *sub)
{
    if (s_sub_count < MQTT_MAX_SUBSCRIPTIONS) {
        s_sub_qos[s_sub_count] = 0u;
        s_subs[s_sub_count++]  = sub;
        mqtt_rx_register(sub);
    }
}

void GO_communication_mqtt_unsubscribe(const char *topic)
{
    if (!s_mosq) { return; }
    mosquitto_unsubscribe(s_mosq, NULL, topic);
}

__attribute__((weak)) void GO_communication_mqtt_on_status(uint8_t status) { (void)status; }

#endif /* GOCONTROLL_LINUX */


/* ============================================================================
 * S1 — ESP32 co-processor via GO_communication_esp
 * ========================================================================= */
#ifdef GOCONTROLL_STM

#include "GO_communication_esp.h"
#include "stm32h5xx.h"      /* __get_PRIMASK / __disable_irq (CMSIS core) */

_Static_assert(sizeof(GO_MqttSubData_t) == sizeof(EspInterface_MqttSubData_t),
               "GO_MqttSubData_t and EspInterface_MqttSubData_t must have the same layout");

/* The ESP driver delivers from the UART interrupt (HAL_UART_RxCpltCallback ->
 * DispatchFrame), poll() runs in a task. Masking interrupts on the task side is
 * enough: the task cannot preempt the ISR. PRIMASK is saved and restored, so the
 * lock nests and is also safe when taken inside the ISR itself. */
static uint32_t mqtt_rx_lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}
static void mqtt_rx_unlock(uint32_t key) { __set_PRIMASK(key); }

/* Overrides the weak hook in GO_communication_esp.c: a message for a subscription
 * registered through this API goes into its FIFO once poll() is in use. Returning
 * false leaves the ESP driver's own behaviour untouched, so code that uses
 * GO_communication_esp_* directly (Simulink blockset) does not change. */
bool GO_communication_esp_on_mqtt_message(const char *topic, const uint8_t *payload, uint16_t len)
{
    return mqtt_rx_deliver(topic, payload, len);
}

void GO_communication_mqtt_configure(const char *url, uint16_t port,
                                     const char *user, const char *pass,
                                     const char *client_id, uint16_t keep_alive)
{
    GO_communication_esp_set_mqtt_config(url, port, user, pass, client_id, keep_alive);
}

void GO_communication_mqtt_enable(bool enable)
{
    GO_communication_esp_enable_mqtt(enable);
}

void GO_communication_mqtt_publish(const char *topic, const uint8_t *payload,
                                   uint16_t len, uint8_t qos, uint8_t retain)
{
    GO_communication_esp_mqtt_publish(topic, payload, len, qos, retain);
}

void GO_communication_mqtt_subscribe(const char *topic, uint8_t qos)
{
    GO_communication_esp_mqtt_subscribe(topic, qos);
}

void GO_communication_mqtt_sub_register(GO_MqttSubData_t *sub)
{
    GO_communication_esp_mqtt_sub_register((EspInterface_MqttSubData_t *)sub);
    mqtt_rx_register(sub);
}

void GO_communication_mqtt_unsubscribe(const char *topic)
{
    GO_communication_esp_mqtt_unsubscribe(topic);
}

/* Forward ESP status callback → common callback */
void GO_communication_esp_on_mqtt_status(uint8_t status)
{
    GO_communication_mqtt_on_status(status);
}

__attribute__((weak)) void GO_communication_mqtt_on_status(uint8_t status) { (void)status; }

#endif /* GOCONTROLL_STM */
