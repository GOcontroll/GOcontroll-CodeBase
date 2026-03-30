/**************************************************************************************
 * \file   GO_communication_mqtt.c
 * \brief  Platform-independent MQTT client — implementation.
 *
 *         Conditional compilation selects the correct backend:
 *           GOCONTROLL_LINUX  — libmosquitto, background network thread
 *           GOCONTROLL_IOT    — GO_communication_esp (ESP32 co-processor via UART)
 **************************************************************************************/

#include "GO_communication_mqtt.h"
#include <string.h>

#define MQTT_MAX_SUBSCRIPTIONS  8u

/* ============================================================================
 * LINUX — libmosquitto
 * ========================================================================= */
#ifdef GOCONTROLL_LINUX

#include <mosquitto.h>

static struct mosquitto *s_mosq      = NULL;
static char     s_url[128]           = {0};
static uint16_t s_port               = 1883u;
static char     s_user[64]           = {0};
static char     s_pass[64]           = {0};
static char     s_client_id[64]      = {0};
static uint16_t s_keep_alive         = 60u;

static GO_MqttSubData_t *s_subs[MQTT_MAX_SUBSCRIPTIONS];
static uint8_t           s_sub_count = 0u;

static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)obj;
    if (rc == 0) {
        GO_communication_mqtt_on_status(GO_MQTT_STATUS_CONNECTED);
        for (uint8_t i = 0u; i < s_sub_count; i++) {
            mosquitto_subscribe(mosq, NULL, s_subs[i]->topic, 0);
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
    for (uint8_t i = 0u; i < s_sub_count; i++) {
        if (strcmp(s_subs[i]->topic, msg->topic) == 0) {
            uint16_t len = (uint16_t)msg->payloadlen;
            if (len > s_subs[i]->payload_size) { len = s_subs[i]->payload_size; }
            memcpy(s_subs[i]->payload, msg->payload, len);
            s_subs[i]->length   = len;
            s_subs[i]->new_flag = 1u;
            break;
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
    if (!s_mosq) { return; }
    mosquitto_subscribe(s_mosq, NULL, topic, (int)qos);
}

void GO_communication_mqtt_sub_register(GO_MqttSubData_t *sub)
{
    if (s_sub_count < MQTT_MAX_SUBSCRIPTIONS) { s_subs[s_sub_count++] = sub; }
}

void GO_communication_mqtt_unsubscribe(const char *topic)
{
    if (!s_mosq) { return; }
    mosquitto_unsubscribe(s_mosq, NULL, topic);
}

__attribute__((weak)) void GO_communication_mqtt_on_status(uint8_t status) { (void)status; }

#endif /* GOCONTROLL_LINUX */


/* ============================================================================
 * IOT — ESP32 co-processor via GO_communication_esp
 * ========================================================================= */
#ifdef GOCONTROLL_IOT

#include "GO_communication_esp.h"

_Static_assert(sizeof(GO_MqttSubData_t) == sizeof(EspInterface_MqttSubData_t),
               "GO_MqttSubData_t and EspInterface_MqttSubData_t must have the same layout");

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

#endif /* GOCONTROLL_IOT */
