/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <kernel_structs.h>
#include <stdio.h>
#include <string.h>
#include <gps.h>
#include <sensor.h>
#include <console.h>
#include <misc/reboot.h>
#include <logging/log_ctrl.h>
#if defined(CONFIG_BSD_LIBRARY)
#include <net/bsdlib.h>
#include <at_cmd.h>
#include <lte_lc.h>
#include <modem_info.h>
#endif /* CONFIG_BSD_LIBRARY */
#include <net/cloud.h>
#include <net/mqtt.h>
#include <net/socket.h>
#include <net/aws_fota.h>
#include <nrf_cloud.h>

#include <dfu/mcuboot.h>

#include "ui.h"
#include "cloud_codec.h"
#include "env_sensors.h"
#include "gps_controller.h"
#include "orientation_detector.h"

#include "mqtt/mqtt.h"
#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "nvs/local_storage.h"
#include "bme/bme680_helper.h"
#include "modem/modem_helper.h"
#include "icm/icm42605_helper.h"


#if defined(CONFIG_BSD_LIBRARY)
#include "nrf_inbuilt_key.h"
#endif

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
#include "certificates.h"
#endif


#ifdef CONFIG_ACCEL_USE_SIM
#define FLIP_INPUT			CONFIG_FLIP_INPUT
#define CALIBRATION_INPUT		-1
#else
#define FLIP_INPUT			-1
#ifdef CONFIG_ACCEL_CALIBRATE
#define CALIBRATION_INPUT		CONFIG_CALIBRATION_INPUT
#else
#define CALIBRATION_INPUT		-1
#endif /* CONFIG_ACCEL_CALIBRATE */
#endif /* CONFIG_ACCEL_USE_SIM */

#if defined(CONFIG_BSD_LIBRARY) && \
!defined(CONFIG_LTE_LINK_CONTROL)
#error "Missing CONFIG_LTE_LINK_CONTROL"
#endif

#if defined(CONFIG_BSD_LIBRARY) && \
defined(CONFIG_LTE_AUTO_INIT_AND_CONNECT) && \
defined(CONFIG_NRF_CLOUD_PROVISION_CERTIFICATES)
#error "PROVISION_CERTIFICATES \
requires CONFIG_LTE_AUTO_INIT_AND_CONNECT to be disabled!"
#endif


#define SENSOR_PAYLOAD_MAX_LEN 150

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
    printk("[%s:%d] %s: %d <%s>\n", __func__, __LINE__, \
           (func), rc, RC_STR(rc))
#define SUCCESS_OR_BREAK(rc) { if (rc != 0) { return ; } }

struct rsrp_data {
    u16_t value;
    u16_t offset;
};

#if CONFIG_MODEM_INFO
static struct rsrp_data rsrp = {
    .value = 0,
    .offset = MODEM_INFO_RSRP_OFFSET_VAL,
};
#endif /* CONFIG_MODEM_INFO */

extern void unittest();
static struct cloud_backend *cloud_backend;


/* Sensor data */
static struct gps_data gps_data;
static iotex_st_timestamp gps_timestamp;
static struct cloud_channel_data gps_cloud_data;

#if CONFIG_MODEM_INFO
static struct modem_param_info modem_param;
static struct cloud_channel_data signal_strength_cloud_data;
static struct cloud_channel_data device_cloud_data;
#endif /* CONFIG_MODEM_INFO */

/* Structures for work */
static struct k_work send_gps_data_work;
static struct k_delayed_work send_env_data_work;
#if CONFIG_MODEM_INFO
static struct k_work device_status_work;
static struct k_work rsrp_work;
#endif /* CONFIG_MODEM_INFO */

/* File descriptor */
static struct pollfd fds;

/* MQTT Broker details. */
static struct mqtt_client client;

/* Set to true when application should teardown and reboot */
static bool do_reboot;

enum error_type {
    ERROR_CLOUD,
    ERROR_BSD_RECOVERABLE,
    ERROR_BSD_IRRECOVERABLE,
    ERROR_LTE_LC,
    ERROR_SYSTEM_FAULT
};

/* Forward declaration of functions */
static void env_data_send(void);
static void sensors_init(void);
static void work_init(void);
static void sensor_data_send(struct cloud_channel_data *data);

static  void publish_env_sensors_data(void);
static  void publish_gps_data(void);


#if CONFIG_MODEM_INFO
static void device_status_send(struct k_work *work);
#endif

/**@brief nRF Cloud error handler. */
void error_handler(enum error_type err_type, int err_code)
{
    if (err_type == ERROR_CLOUD) {
        if (mqtt_disconnect(&client)) {
            printk("Could not disconnect MQTT client during error handler.\n");
        }

        if (gps_control_is_enabled()) {
            printk("Reboot\n");
            sys_reboot(0);
        }

#if defined(CONFIG_LTE_LINK_CONTROL)
        /* Turn off and shutdown modem */
        printk("LTE link disconnect\n");
        int err = lte_lc_power_off();

        if (err) {
            printk("lte_lc_power_off failed: %d\n", err);
        }

#endif /* CONFIG_LTE_LINK_CONTROL */
#if defined(CONFIG_BSD_LIBRARY)
        printk("Shutdown modem\n");
        bsdlib_shutdown();
#endif
    }

#if !defined(CONFIG_DEBUG) && defined(CONFIG_REBOOT)
    LOG_PANIC();
    sys_reboot(0);
#else

    switch (err_type) {
        case ERROR_CLOUD:
            /* Blinking all LEDs ON/OFF in pairs (1 and 4, 2 and 3)
             * if there is an application error.
             */
            ui_led_set_pattern(UI_LED_ERROR_CLOUD);
            printk("Error of type ERROR_CLOUD: %d\n", err_code);
            break;

        case ERROR_BSD_RECOVERABLE:
            /* Blinking all LEDs ON/OFF in pairs (1 and 3, 2 and 4)
             * if there is a recoverable error.
             */
            ui_led_set_pattern(UI_LED_ERROR_BSD_REC);
            printk("Error of type ERROR_BSD_RECOVERABLE: %d\n", err_code);
            break;

        case ERROR_BSD_IRRECOVERABLE:
            /* Blinking all LEDs ON/OFF if there is an
             * irrecoverable error.
             */
            ui_led_set_pattern(UI_LED_ERROR_BSD_IRREC);
            printk("Error of type ERROR_BSD_IRRECOVERABLE: %d\n", err_code);
            break;

        default:
            /* Blinking all LEDs ON/OFF in pairs (1 and 2, 3 and 4)
             * undefined error.
             */
            ui_led_set_pattern(UI_LED_ERROR_UNKNOWN);
            printk("Unknown error type: %d, code: %d\n",
                   err_type, err_code);
            break;
    }

    while (true) {
        k_cpu_idle();
    }

#endif /* CONFIG_DEBUG */
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const z_arch_esf_t *esf)
{
    ARG_UNUSED(esf);

    LOG_PANIC();
    z_fatal_print("Running main.c error handler");
    error_handler(ERROR_SYSTEM_FAULT, reason);
    CODE_UNREACHABLE;
}

void cloud_error_handler(int err) {
    error_handler(ERROR_CLOUD, err);
}

/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err) {
    error_handler(ERROR_BSD_RECOVERABLE, (int)err);
}

/**@brief Irrecoverable BSD library error. */
void bsd_irrecoverable_error_handler(uint32_t err) {
    error_handler(ERROR_BSD_IRRECOVERABLE, (int)err);
}

static void send_gps_data_work_fn(struct k_work *work) {
    publish_gps_data();
}

static void send_env_data_work_fn(struct k_work *work) {
    printk("[%s:%d]\n", __func__, __LINE__);
    env_data_send();
}

/**@brief Callback for GPS trigger events */
static void gps_trigger_handler(struct device *dev, struct gps_trigger *trigger)
{
    static u32_t fix_count;

    ARG_UNUSED(trigger);
    printk("GPS trigger handler %d\n", fix_count);

    if (ui_button_is_active(UI_SWITCH_2)
            || !atomic_get(&send_data_enable)) {
        return;
    }

    if (++fix_count < CONFIG_GPS_CONTROL_FIX_COUNT) {
        return;
    }

    fix_count = 0;

    ui_led_set_pattern(UI_LED_GPS_FIX);

    gps_sample_fetch(dev);
    gps_channel_get(dev, GPS_CHAN_PVT, &gps_data);
    iotex_modem_get_clock(&gps_timestamp);

    gps_cloud_data.data.buf = gps_data.nmea.buf;
    gps_cloud_data.data.len = gps_data.nmea.len;
    gps_cloud_data.tag += 1;

    if (gps_cloud_data.tag == 0) {
        gps_cloud_data.tag = 0x1;
    }

    gps_control_stop(K_NO_WAIT);
    k_work_submit(&send_gps_data_work);
    k_delayed_work_submit(&send_env_data_work, K_NO_WAIT);
}

#if CONFIG_MODEM_INFO
/**@brief Callback handler for LTE RSRP data. */
static void modem_rsrp_handler(char rsrp_value)
{
    rsrp.value = rsrp_value;

    k_work_submit(&rsrp_work);
}

/**@brief Publish RSRP data to the cloud. */
static void modem_rsrp_data_send(struct k_work *work)
{
    char buf[CONFIG_MODEM_INFO_BUFFER_SIZE] = {0};
    static u32_t timestamp_prev;
    size_t len;

    if (!atomic_get(&send_data_enable)) {
        return;
    }

    if (k_uptime_get_32() - timestamp_prev <
            K_SECONDS(CONFIG_HOLD_TIME_RSRP)) {
        return;
    }

    len = snprintf(buf, CONFIG_MODEM_INFO_BUFFER_SIZE,
                   "%d", rsrp.value - rsrp.offset);

    signal_strength_cloud_data.data.buf = buf;
    signal_strength_cloud_data.data.len = len;
    signal_strength_cloud_data.tag += 1;

    if (signal_strength_cloud_data.tag == 0) {
        signal_strength_cloud_data.tag = 0x1;
    }

    sensor_data_send(&signal_strength_cloud_data);
    timestamp_prev = k_uptime_get_32();
}

/**@brief Poll device info and send data to the cloud. */
static void device_status_send(struct k_work *work)
{
    int len;
    int ret;

    cJSON *root_obj = cJSON_CreateObject();

    if (root_obj == NULL) {
        printk("Unable to allocate JSON object\n");
        return;
    }

    if (!atomic_get(&send_data_enable)) {
        return;
    }

    ret = modem_info_params_get(&modem_param);

    if (ret < 0) {
        printk("Unable to obtain modem parameters: %d\n", ret);
        return;
    }

    len = modem_info_json_object_encode(&modem_param, root_obj);

    if (len < 0) {
        return;
    }

    device_cloud_data.data.buf = (char *)root_obj;
    device_cloud_data.data.len = len;
    device_cloud_data.tag += 1;

    if (device_cloud_data.tag == 0) {
        device_cloud_data.tag = 0x1;
    }

    /* Transmits the data to the cloud. Frees the JSON object. */
    sensor_data_send(&device_cloud_data);
}
#endif /* CONFIG_MODEM_INFO */

/**@brief Get environment data from sensors and send to cloud. */
static void env_data_send(void)
{
    printk("[%s:%d]\n", __func__, __LINE__);

    if (!atomic_get(&send_data_enable)) {
        return;
    }

    if (gps_control_is_active()) {
        k_delayed_work_submit(&send_env_data_work,
                              K_SECONDS(CONFIG_ENVIRONMENT_DATA_BACKOFF_TIME));
        return;
    }

    publish_env_sensors_data();

    k_delayed_work_submit(&send_env_data_work,
                          K_SECONDS(CONFIG_ENVIRONMENT_DATA_SEND_INTERVAL));

    return;

}

/**@brief Send sensor data to nRF Cloud. **/
static void sensor_data_send(struct cloud_channel_data *data)
{
    int err = 0;
    struct cloud_msg msg = {
        .qos = CLOUD_QOS_AT_MOST_ONCE,
        .endpoint.type = CLOUD_EP_TOPIC_MSG
    };

    if (data->type == CLOUD_CHANNEL_DEVICE_INFO) {
        msg.endpoint.type = CLOUD_EP_TOPIC_STATE;
    }

    if (!atomic_get(&send_data_enable) || gps_control_is_active()) {
        return;
    }

    if (data->type != CLOUD_CHANNEL_DEVICE_INFO) {
        err = cloud_encode_data(data, &msg);
    } else {
        err = cloud_encode_digital_twin_data(data, &msg);
    }

    if (err) {
        printk("Unable to encode cloud data: %d\n", err);
    }

    err = cloud_send(cloud_backend, &msg);

    cloud_release_data(&msg);

    if (err) {
        printk("sensor_data_send failed: %d\n", err);
        cloud_error_handler(err);
    }
}

/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
    k_work_init(&send_gps_data_work, send_gps_data_work_fn);
    k_delayed_work_init(&send_env_data_work, send_env_data_work_fn);
#if CONFIG_MODEM_INFO
    k_work_init(&device_status_work, device_status_send);
    k_work_init(&rsrp_work, modem_rsrp_data_send);
#endif /* CONFIG_MODEM_INFO */
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static void modem_configure(void)
{
#if defined(CONFIG_BSD_LIBRARY)

    if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
        /* Do nothing, modem is already turned on
         * and connected.
         */
    } else {
        int err;

        printk("Connecting to LTE network. ");
        printk("This may take several minutes.\n");
        ui_led_set_pattern(UI_LTE_CONNECTING);

        err = lte_lc_init_and_connect();

        if (err) {
            printk("LTE link could not be established.\n");
            error_handler(ERROR_LTE_LC, err);
        }

        printk("Connected to LTE network\n");
        ui_led_set_pattern(UI_LTE_CONNECTED);
    }

#endif
}


#if CONFIG_MODEM_INFO
/**brief Initialize LTE status containers. */
static void modem_data_init(void)
{
    int err;
    err = modem_info_init();

    if (err) {
        printk("Modem info could not be established: %d\n", err);
        return;
    }

    modem_info_params_init(&modem_param);

    signal_strength_cloud_data.type = CLOUD_CHANNEL_LTE_LINK_RSRP;
    signal_strength_cloud_data.tag = 0x1;

    device_cloud_data.type = CLOUD_CHANNEL_DEVICE_INFO;
    device_cloud_data.tag = 0x1;

    k_work_submit(&device_status_work);

    modem_info_rsrp_register(modem_rsrp_handler);
}
#endif /* CONFIG_MODEM_INFO */

/**@brief Initializes the sensors that are used by the application. */
static void sensors_init(void)
{
    /* Iotex Init BME680 */
    iotex_bme680_init();

    /* Iotex Init ICM42605 */
    iotex_icm42605_init();

    gps_control_init(gps_trigger_handler);

    /* Send sensor data after initialization, as it may be a long time until
     * next time if the application is in power optimized mode.
     */
    k_delayed_work_submit(&send_env_data_work, K_SECONDS(10));
    printk("[%s:%d] Sensors initialized\n", __func__, __LINE__);
}

#if defined(CONFIG_USE_UI_MODULE)
/**@brief User interface event handler. */
static void ui_evt_handler(struct ui_evt evt)
{
    printk("ui evt handler %d\n", evt.button);


#if defined(CONFIG_LTE_LINK_CONTROL)

    if ((evt.button == UI_SWITCH_2) &&
            IS_ENABLED(CONFIG_POWER_OPTIMIZATION_ENABLE)) {
        int err;

        if (evt.type == UI_EVT_BUTTON_ACTIVE) {
            err = lte_lc_edrx_req(false);

            if (err) {
                error_handler(ERROR_LTE_LC, err);
            }

            err = lte_lc_psm_req(true);

            if (err) {
                error_handler(ERROR_LTE_LC, err);
            }
        } else {
            err = lte_lc_psm_req(false);

            if (err) {
                error_handler(ERROR_LTE_LC, err);
            }

            err = lte_lc_edrx_req(true);

            if (err) {
                error_handler(ERROR_LTE_LC, err);
            }
        }
    }

#endif /* defined(CONFIG_LTE_LINK_CONTROL) */
}
#endif /* defined(CONFIG_USE_UI_MODULE) */


#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
static int provision_certificates(void)
{
    {
        int err;

        /* Delete certificates */
        nrf_sec_tag_t sec_tag = CONFIG_CLOUD_CERT_SEC_TAG;

        for (nrf_key_mgnt_cred_type_t type = 0; type < 3; type++) {
            err = nrf_inbuilt_key_delete(sec_tag, type);
            printk("nrf_inbuilt_key_delete(%u, %d) => result=%d\n",
                   sec_tag, type, err);
        }

        /* Provision CA Certificate. */
        err = nrf_inbuilt_key_write(CONFIG_CLOUD_CERT_SEC_TAG,
                                    NRF_KEY_MGMT_CRED_TYPE_CA_CHAIN,
                                    NRF_CLOUD_CA_CERTIFICATE,
                                    strlen(NRF_CLOUD_CA_CERTIFICATE));
        printk("nrf_inbuilt_key_write => result=%d\n", err);

        if (err) {
            printk("NFR_CLOUD_CA_CERTIFICATE err: %d", err);
            return err;
        }

        /* Provision Private Certificate. */
        err = nrf_inbuilt_key_write(
                  CONFIG_CLOUD_CERT_SEC_TAG,
                  NRF_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
                  NRF_CLOUD_CLIENT_PRIVATE_KEY,
                  strlen(NRF_CLOUD_CLIENT_PRIVATE_KEY));
        printk("nrf_inbuilt_key_write => result=%d\n", err);

        if (err) {
            printk("NRF_CLOUD_CLIENT_PRIVATE_KEY err: %d", err);
            return err;
        }

        /* Provision Public Certificate. */
        err = nrf_inbuilt_key_write(
                  CONFIG_CLOUD_CERT_SEC_TAG,
                  NRF_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
                  NRF_CLOUD_CLIENT_PUBLIC_CERTIFICATE,
                  strlen(NRF_CLOUD_CLIENT_PUBLIC_CERTIFICATE));
        printk("nrf_inbuilt_key_write => result=%d\n", err);

        if (err) {
            printk("CLOUD_CLIENT_PUBLIC_CERTIFICATE err: %d",
                   err);
            return err;
        }
    }
    return 0;
}
#endif


static char *get_mqtt_payload_devicedata(enum mqtt_qos qos) {
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN];

    snprintf(payload, sizeof(payload),
             "{\"Device\":\"%s\",\"VBAT\":%.2f, \"SNR\":%d, \"timestamp\":%s}",
             iotex_mqtt_get_client_id(), iotex_hal_adc_sample(),
             iotex_model_get_signal_quality(), iotex_modem_get_clock(NULL));
    return payload;
}


static char *get_mqtt_payload_gps(enum mqtt_qos qos) {
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;

    printf("{GPS latitude:%f, longitude:%f, timestamp %s\n", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp.data);
    snprintf(payload, sizeof(payload), "{\"Device\":\"%s\",\"latitude\":%f,\"longitude\":%f, \"timestamp\":%s}",
             "GPS", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp.data);

    return payload;
}

static char *get_mqtt_payload_bme680(enum mqtt_qos qos) {
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN];
    iotex_bme680_get_sensor_data((uint8_t *)&payload, sizeof(payload));
    return payload;
}

static char *get_mqtt_payload_icm42605(enum mqtt_qos qos) {
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;
    iotex_icm42605_get_sensor_data((uint8_t *)&payload, sizeof(payload));
    return payload;
}


static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
    int_fast64_t remaining = timeout;
    int_fast64_t start_time = k_uptime_get();
    int rc;

    while (remaining > 0 && iotex_mqtt_is_connected()) {
        k_busy_wait(remaining);

        rc = mqtt_live(client);

        if (rc != 0) {
            PRINT_RESULT("mqtt_live", rc);
            return rc;
        }

        rc = mqtt_input(client);

        if (rc != 0) {
            PRINT_RESULT("mqtt_input", rc);
            return rc;
        }

        remaining = timeout + start_time - k_uptime_get();
    }

    return 0;
}

void publish_gps_data()
{
    int rc;
    printk("[%s:%d]\n", __func__, __LINE__);

    if (iotex_mqtt_is_connected())
    {
        rc = mqtt_ping(&client);
        PRINT_RESULT("mqtt_ping", rc);
        SUCCESS_OR_BREAK(rc);
        rc = iotex_mqtt_publish_data(&client, 0, get_mqtt_payload_gps(0));
        PRINT_RESULT("mqtt_publish_gps", rc);
    }

}

void publish_env_sensors_data()
{
    int rc;
    printk("[%s:%d]\n", __func__, __LINE__);

    if (iotex_mqtt_is_connected())
    {
        rc = mqtt_ping(&client);
        PRINT_RESULT("mqtt_ping", rc);
        SUCCESS_OR_BREAK(rc);
        // gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
        rc = iotex_mqtt_publish_data(&client, 0, get_mqtt_payload_devicedata(0));
        PRINT_RESULT("mqtt_publish_devicedata", rc);
        rc = iotex_mqtt_publish_data(&client, 0, get_mqtt_payload_bme680(0));
        PRINT_RESULT("mqtt_publish_bme680", rc);
        rc = iotex_mqtt_publish_data(&client, 0, get_mqtt_payload_icm42605(0));
        PRINT_RESULT("mqtt_publish_icm42605", rc);
    }

}


void main(void)
{
    int err;

    /* HAL init, notice gpio must be the first (to set IO_POWER_ON on )*/
    iotex_hal_gpio_init();
    iotex_hal_adc_init();

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
    provision_certificates();
#endif /* CONFIG_USE_PROVISIONED_CERTIFICATES  */

#if defined(CONFIG_USE_UI_MODULE)
    ui_init(ui_evt_handler);
#endif

    work_init();
    modem_configure();
    iotex_modem_get_clock(NULL);
    iotex_local_storage_init();

#ifdef CONFIG_UNITTEST
    unittest();
#endif

    if ((err = iotex_mqtt_client_init(&client, &fds))) {
        printk("ERROR: mqtt_connect %d, rebooting...\n", err);
        k_sleep(500);
        sys_reboot(0);
        return;
    }

    sensors_init();

    while (true) {
        err = poll(&fds, 1, K_SECONDS(CONFIG_MQTT_KEEPALIVE));

        if (err < 0) {
            printk("ERROR: poll %d\n", errno);
            error_handler(ERROR_CLOUD, err);
            break;
        }

        err = mqtt_live(&client);

        if (err != 0) {
            printk("ERROR: mqtt_live %d\n", err);
            error_handler(ERROR_CLOUD, err);
            break;
        }

        printk("mqtt live ????\n");

        if ((fds.revents & POLLIN) == POLLIN) {
            err = mqtt_input(&client);

            if (err != 0) {
                printk("ERROR: mqtt_input %d\n", err);
                error_handler(ERROR_CLOUD, -EIO);
                break;
            }
        }

        if ((fds.revents & POLLERR) == POLLERR) {
            printk("POLLERR\n");
            error_handler(ERROR_CLOUD, -EIO);
            break;
        }

        if ((fds.revents & POLLNVAL) == POLLNVAL) {
            printk("POLLNVAL\n");
            error_handler(ERROR_CLOUD, -EIO);
            break;
        }

        if (do_reboot) {
            /* Teardown */
            mqtt_disconnect(&client);
            sys_reboot(0);
        }
    }

    printk("Disconnecting MQTT client...\n");

    err = mqtt_disconnect(&client);

    if (err) {
        printk("Could not disconnect MQTT client. Error: %d\n", err);
    }

    iotex_hal_gpio_set(LED_RED, LED_ON);
    k_sleep(500);
    iotex_hal_gpio_set(LED_RED, LED_OFF);
    k_sleep(500);
    iotex_hal_gpio_set(LED_RED, LED_OFF);
    k_sleep(500);
    sys_reboot(0);
}
