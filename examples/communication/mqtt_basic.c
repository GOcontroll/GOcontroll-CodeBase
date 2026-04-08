/**************************************************************************************
 * \file   mqtt_basic.c
 * \brief  MQTT publish / subscribe example — platform independent.
 *
 *         Reads channel 1 of a 10-channel input module on slot 1 and publishes
 *         the millivolt value to an MQTT broker every second.
 *         Simultaneously subscribes to a command topic and prints any message
 *         received on that topic.
 *
 *         This example works unchanged on both platforms:
 *           GOCONTROLL_LINUX  (Moduline IV / Moduline Mini) — uses libmosquitto
 *           GOCONTROLL_IOT    (Moduline IOT, STM32H5)       — uses ESP32 co-processor
 *
 *         Topic layout:
 *           Publish:   /gocontroll/sensor/ch1     payload: ASCII millivolt value
 *           Subscribe: /gocontroll/cmd/ch1        payload: any ASCII command
 *
 *         Build (Linux):
 *           make mqtt_publish
 *
 *         Prerequisites:
 *           - An MQTT broker reachable on BROKER_HOST:BROKER_PORT
 *           - For Linux: libmosquitto installed (apt install libmosquitto-dev)
 *           - For IOT:   ESP32 co-processor module present in the controller
 **************************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "GO_board.h"
#include "GO_communication_modules.h"
#include "GO_module_input.h"
#include "GO_communication_mqtt.h"
#include "print.h"

/* ---- Broker configuration ------------------------------------------------- */
#define BROKER_HOST       "192.168.1.19"   /* Change to your broker IP or hostname */
#define BROKER_PORT       1883u
#define BROKER_USER       ""               /* Leave empty for anonymous access      */
#define BROKER_PASS       ""
#define CLIENT_ID         "gocontroll-01"
#define KEEP_ALIVE_SEC    60u

/* ---- Topics --------------------------------------------------------------- */
#define TOPIC_PUBLISH     "/gocontroll/sensor/ch1"
#define TOPIC_SUBSCRIBE   "/gocontroll/cmd/ch1"

/* ---- Timing --------------------------------------------------------------- */
#define PUBLISH_INTERVAL_CYCLES  100   /* 100 × 10 ms = 1 s */

/* ---- Input module --------------------------------------------------------- */
#define INPUT_SLOT        MODULESLOT1

/* =========================================================================== */

static _inputModule inputModule;

/* Subscription data block — payload buffer for incoming commands */
static uint8_t          rx_buf[128];
static GO_MqttSubData_t cmd_sub = {
    .topic        = TOPIC_SUBSCRIBE,
    .payload      = rx_buf,
    .payload_size = sizeof(rx_buf),
};

/* ---- Shutdown callback ---------------------------------------------------- */
static void app_terminate(void)
{
    info("Shutting down — disconnecting MQTT\n");
    GO_communication_mqtt_enable(false);
}

/* ---- MQTT status callback (weak override) --------------------------------- */
void GO_communication_mqtt_on_status(uint8_t status)
{
    switch (status) {
        case GO_MQTT_STATUS_CONNECTING:
            info("MQTT: verbinden met %s:%u ...\n", BROKER_HOST, BROKER_PORT);
            break;
        case GO_MQTT_STATUS_CONNECTED:
            info("MQTT: verbonden\n");
            break;
        case GO_MQTT_STATUS_DISCONNECTED:
            info("MQTT: verbroken\n");
            break;
        default:
            break;
    }
}

/* =========================================================================== */
int main(void)
{
    info("=== MQTT publish/subscribe example ===\n");
    info("Broker: %s:%u\n", BROKER_HOST, BROKER_PORT);
    info("Publish:   %s\n", TOPIC_PUBLISH);
    info("Subscribe: %s\n", TOPIC_SUBSCRIBE);
    info("Press Ctrl+C to stop.\n\n");

    GO_board_get_hardware_version();

    /* --- Input module setup ------------------------------------------------ */
    GO_module_input_set_module_type(&inputModule, INPUTMODULE10CHANNEL);
    GO_module_input_set_module_slot(&inputModule, INPUT_SLOT);
    GO_communication_modules_initialize(INPUT_SLOT);

    for (uint8_t ch = INPUTCHANNEL1; ch <= INPUTCHANNEL10; ch++) {
        GO_module_input_10ch_configure_channel(&inputModule, ch,
                                               INPUTFUNC_MVANALOG,
                                               INPUTPULLUP10CH_10K,
                                               INPUTPULLDOWN10CH_3_3K);
    }
    GO_module_input_configuration(&inputModule);
    info("Input module geconfigureerd (slot 1, mV-analoog)\n\n");

    /* --- MQTT setup -------------------------------------------------------- */

    /* Register the subscription block before enabling — the driver needs
     * to know about it before the first message can arrive. */
    GO_communication_mqtt_sub_register(&cmd_sub);
    GO_communication_mqtt_subscribe(TOPIC_SUBSCRIBE, 1u);

    GO_communication_mqtt_configure(BROKER_HOST, BROKER_PORT,
                                    BROKER_USER[0] ? BROKER_USER : NULL,
                                    BROKER_PASS[0] ? BROKER_PASS : NULL,
                                    CLIENT_ID, KEEP_ALIVE_SEC);
    GO_communication_mqtt_enable(true);

    /* --- Shutdown callback ------------------------------------------------- */
    GO_board_exit_program(app_terminate);

    /* --- Application loop (10 ms) ----------------------------------------- */
    int cycle = 0;

    while (1) {
        /* Read input module */
        GO_module_input_receive_values(&inputModule);
        int32_t mv = (int32_t)inputModule.value[INPUTCHANNEL1];

        /* Publish once per second */
        if (++cycle >= PUBLISH_INTERVAL_CYCLES) {
            cycle = 0;

            char payload[16];
            int  plen = snprintf(payload, sizeof(payload), "%ld", (long)mv);

            GO_communication_mqtt_publish(TOPIC_PUBLISH,
                                          (const uint8_t *)payload,
                                          (uint16_t)plen, 1u, 0u);

            info("TX  %s  →  %ldmV\n", TOPIC_PUBLISH, (long)mv);
        }

        /* Check for incoming command */
        if (cmd_sub.new_flag) {
            cmd_sub.new_flag = 0u;

            /* Null-terminate for safe printing (payload buffer is larger) */
            uint16_t len = cmd_sub.length < sizeof(rx_buf) - 1u
                         ? cmd_sub.length : (uint16_t)(sizeof(rx_buf) - 1u);
            rx_buf[len] = '\0';

            info("RX  %s  →  \"%s\"\n", TOPIC_SUBSCRIBE, (char *)rx_buf);
        }

        usleep(10000); /* 10 ms */
    }

    return 0;
}
