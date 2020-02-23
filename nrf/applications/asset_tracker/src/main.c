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

#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "bme/bme680_helper.h"
#include "modem/modem_helper.h"
#include "icm/icm42605_helper.h"



#if defined(CONFIG_BSD_LIBRARY)
#include "nrf_inbuilt_key.h"
#endif

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
#include "certificates.h"
#endif

#define CALIBRATION_PRESS_DURATION 	K_SECONDS(5)
#define CLOUD_CONNACK_WAIT_DURATION	K_SECONDS(CONFIG_CLOUD_WAIT_DURATION)


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

#if !defined(CONFIG_CLOUD_CLIENT_ID)
#define IMEI_LEN 15
#define CLIENT_ID_LEN (IMEI_LEN + 4)
#else
#define CLIENT_ID_LEN (sizeof(CONFIG_CLOUD_CLIENT_ID) - 1)
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
static atomic_val_t send_data_enable;


/* Structures for work */
static struct k_work connect_work;
static struct k_work send_gps_data_work;

static struct k_delayed_work send_env_data_work;
static struct k_delayed_work long_press_button_work;
static struct k_delayed_work cloud_reboot_work;
#if CONFIG_MODEM_INFO
static struct k_work device_status_work;
static struct k_work rsrp_work;
#endif /* CONFIG_MODEM_INFO */

static bool connected=0;
static u8_t client_id_buf[CLIENT_ID_LEN+1];

/* Buffers for MQTT client. */
static u8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE];

/* MQTT Broker details. */
static struct sockaddr_storage broker_storage;
static struct mqtt_client client;

/* File descriptor */
static struct pollfd fds;

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
static void app_connect(struct k_work *work);
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

void cloud_error_handler(int err)
{
    error_handler(ERROR_CLOUD, err);
}

/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
    error_handler(ERROR_BSD_RECOVERABLE, (int)err);
}

/**@brief Irrecoverable BSD library error. */
void bsd_irrecoverable_error_handler(uint32_t err)
{
    error_handler(ERROR_BSD_IRRECOVERABLE, (int)err);
}

static void send_gps_data_work_fn(struct k_work *work)
{
    //sensor_data_send(&gps_cloud_data);
    publish_gps_data();
}

static void send_env_data_work_fn(struct k_work *work)
{
    printk("[%s:%d]\n", __func__, __LINE__);
    env_data_send();
}

/**@brief Callback for GPS trigger events */
static void gps_trigger_handler(struct device *dev, struct gps_trigger *trigger)
{
    static u32_t fix_count;

    ARG_UNUSED(trigger);
        printk("GPS trigger handler %d\n",fix_count);
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

/**@brief Reboot the device if CONNACK has not arrived. */
static void cloud_reboot_handler(struct k_work *work)
{
    error_handler(ERROR_CLOUD, -ETIMEDOUT);
}


/**@brief Connect to nRF Cloud, */
static void app_connect(struct k_work *work)
{
    int err;

    ui_led_set_pattern(UI_CLOUD_CONNECTING);
    err = cloud_connect(cloud_backend);

    if (err) {
        printk("cloud_connect failed: %d\n", err);
        cloud_error_handler(err);
    }
}

static void long_press_handler(struct k_work *work)
{
    if (!atomic_get(&send_data_enable)) {
        printk("Link not ready, long press disregarded\n");
        return;
    }

    if (gps_control_is_enabled()) {
        printk("Stopping GPS\n");
        gps_control_disable();
    } else {
        printk("Starting GPS\n");
        gps_control_enable();
        gps_control_start(K_SECONDS(1));
    }
}


/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
    k_work_init(&connect_work, app_connect);
    k_work_init(&send_gps_data_work, send_gps_data_work_fn);
    k_delayed_work_init(&send_env_data_work, send_env_data_work_fn);
 
    k_delayed_work_init(&long_press_button_work, long_press_handler);
    k_delayed_work_init(&cloud_reboot_work, cloud_reboot_handler);
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

    iotex_hal_adc_init();

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
      printk("ui evt handler %d\n",evt.button);


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

/**@brief Function to print strings without null-termination. */
static void data_print(u8_t *prefix, u8_t *data, size_t len)
{
    char buf[len + 1];

    memcpy(buf, data, len);
    buf[len] = 0;
    printk("%s%s\n", prefix, buf);
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client *c,
                   u8_t *write_buf,
                   size_t length)
{
    u8_t *buf = write_buf;
    u8_t *end = buf + length;

    if (length > sizeof(payload_buf)) {
        return -EMSGSIZE;
    }
    while (buf < end) {
        int ret = mqtt_read_publish_payload_blocking(c, buf, end - buf);

        if (ret < 0) {
            return ret;
        } else if (ret == 0) {
            return -EIO;
        }
        buf += ret;
    }
    return 0;
}

/**@brief MQTT client event handler */
void mqtt_evt_handler(struct mqtt_client * const c,
              const struct mqtt_evt *evt)
{
    int err;

    switch (evt->type) {
    case MQTT_EVT_CONNACK:
        if (evt->result != 0) {
            printk("MQTT connect failed %d\n", evt->result);
            break;
        }
        k_delayed_work_cancel(&cloud_reboot_work);
        connected=1;
        atomic_set(&send_data_enable, 1);
        sensors_init();

        printk("[%s:%d]\n", __func__, __LINE__);
        iotex_hal_gpio_set(LED_BLUE, LED_ON);
        iotex_hal_gpio_set(LED_GREEN, LED_OFF);

        printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
        break;

    case MQTT_EVT_DISCONNECT:
        connected=0;
        iotex_hal_gpio_set(LED_BLUE, LED_OFF);
        iotex_hal_gpio_set(LED_GREEN, LED_ON);
        printk("[%s:%d] MQTT client disconnected %d\n", __func__,
               __LINE__, evt->result);
        break;

    case MQTT_EVT_PUBLISH: {
        const struct mqtt_publish_param *p = &evt->param.publish;

        printk("[%s:%d] MQTT PUBLISH result=%d len=%d\n", __func__,
               __LINE__, evt->result, p->message.payload.len);
        err = publish_get_payload(c,
                      payload_buf,
                      p->message.payload.len);
        if (err) {
            printk("mqtt_read_publish_payload: Failed! %d\n", err);
            printk("Disconnecting MQTT client...\n");

            err = mqtt_disconnect(c);
            if (err) {
                printk("Could not disconnect: %d\n", err);
            }
        }

        if (p->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
            const struct mqtt_puback_param ack = {
                .message_id = p->message_id
            };

            /* Send acknowledgment. */
            err = mqtt_publish_qos1_ack(c, &ack);
            if (err) {
                printk("unable to ack\n");
            }
        }
        data_print("Received: ", payload_buf, p->message.payload.len);
        break;
    }

    case MQTT_EVT_PUBACK:
        if (evt->result != 0) {
            printk("MQTT PUBACK error %d\n", evt->result);
            break;
        }

        printk("[%s:%d] PUBACK packet id: %u\n", __func__, __LINE__,
               evt->param.puback.message_id);
        break;

    case MQTT_EVT_SUBACK:
        if (evt->result != 0) {
            printk("MQTT SUBACK error %d\n", evt->result);
            break;
        }

        printk("[%s:%d] SUBACK packet id: %u\n", __func__, __LINE__,
               evt->param.suback.message_id);
        break;

    default:
        printk("[%s:%d] default: %d\n", __func__, __LINE__,
               evt->type);
        break;
    }
}


/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static void broker_init(const char *hostname)
{
    int err;
    struct addrinfo *result;
    struct addrinfo *addr;
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM
    };

    err = getaddrinfo(hostname, NULL, &hints, &result);
    if (err) {
        printk("ERROR: getaddrinfo failed %d\n", err);

        return;
    }

    addr = result;
    err = -ENOENT;

    while (addr != NULL) {
        /* IPv4 Address. */
        if (addr->ai_addrlen == sizeof(struct sockaddr_in)) {
            struct sockaddr_in *broker =
                ((struct sockaddr_in *)&broker_storage);

            broker->sin_addr.s_addr =
                ((struct sockaddr_in *)addr->ai_addr)
                ->sin_addr.s_addr;
            broker->sin_family = AF_INET;
            broker->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

            printk("IPv4 Address 0x%08x\n", broker->sin_addr.s_addr);
            break;
        } else if (addr->ai_addrlen == sizeof(struct sockaddr_in6)) {
            /* IPv6 Address. */
            struct sockaddr_in6 *broker =
                ((struct sockaddr_in6 *)&broker_storage);

            memcpy(broker->sin6_addr.s6_addr,
                ((struct sockaddr_in6 *)addr->ai_addr)
                ->sin6_addr.s6_addr,
                sizeof(struct in6_addr));
            broker->sin6_family = AF_INET6;
            broker->sin6_port = htons(CONFIG_MQTT_BROKER_PORT);

            printk("IPv6 Address");
            break;
        } else {
            printk("error: ai_addrlen = %u should be %u or %u\n",
                (unsigned int)addr->ai_addrlen,
                (unsigned int)sizeof(struct sockaddr_in),
                (unsigned int)sizeof(struct sockaddr_in6));
        }

        addr = addr->ai_next;
        break;
    }

    /* Free the address. */
    freeaddrinfo(result);
}

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

static int client_id_get(char *id_buf)
{
#if !defined(CONFIG_CLOUD_CLIENT_ID)
    enum at_cmd_state at_state;
    char imei_buf[IMEI_LEN + 5];

    int err = at_cmd_write("AT+CGSN", imei_buf, (IMEI_LEN + 5), &at_state);
    if (err) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
    }

    snprintf(id_buf, CLIENT_ID_LEN + 1, "nrf-%s", imei_buf);
#else
    memcpy(id_buf, CONFIG_CLOUD_CLIENT_ID, CLIENT_ID_LEN + 1);
#endif /* !defined(NRF_CLOUD_CLIENT_ID) */
    return 0;
}

/**@brief Initialize the MQTT client structure */
static int client_init(struct mqtt_client *client, char *hostname)
{
    mqtt_client_init(client);
    broker_init(hostname);

    int ret = client_id_get(client_id_buf);
    printk("client_id: %s\n", client_id_buf);

    if (ret != 0) {
        return ret;
    }

    /* MQTT client configuration */
    client->broker = &broker_storage;
    client->evt_cb = mqtt_evt_handler;
    client->client_id.utf8 = client_id_buf;
    client->client_id.size = strlen(client_id_buf);
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    /* MQTT buffers configuration */
    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    /* MQTT transport configuration */
    client->transport.type = MQTT_TRANSPORT_SECURE;

    static sec_tag_t sec_tag_list[] = {CONFIG_CLOUD_CERT_SEC_TAG};
    struct mqtt_sec_config *tls_config = &(client->transport).tls.config;

    tls_config->peer_verify = 2;
    tls_config->cipher_list = NULL;
    tls_config->cipher_count = 0;
    tls_config->sec_tag_count = ARRAY_SIZE(sec_tag_list);
    tls_config->sec_tag_list = sec_tag_list;
    tls_config->hostname = hostname;

    return 0;
}

/**@brief Initialize the file descriptor structure used by poll. */
static int fds_init(struct mqtt_client *c)
{
    fds.fd = c->transport.tls.sock;
    fds.events = POLLIN;
    return 0;
}



static int signal_quality_get(char *id_buf)
{
    enum at_cmd_state at_state;
    char snr_buf[32];

    int err = at_cmd_write("AT+CESQ", snr_buf, 32, &at_state);
    if (err) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
    }

    snprintf(id_buf, 4, "%s", &snr_buf[25]);
    //  printk("SNR:%d\n",atoi(id_buf));
    return 0;
}

static char *get_mqtt_payload_devicedata(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;
    static uint8_t snr[4] ;

    signal_quality_get(snr);
    sprintf(payload, "{\"Device\":\"%s\",\"VBAT\":%.2f, \"SNR\":%d, \"timestamp\":%s}",
            client_id_buf, iotex_hal_adc_sample(), atoi(snr), iotex_modem_get_clock(NULL));
    return payload;
}


static char *get_mqtt_payload_gps(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;

    printf("{GPS  latitude:%f, longitude:%f, timestamp %s\n", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp.data);
    sprintf(payload,"{\"Device\":\"%s\",\"latitude\":%f,\"longitude\":%f, \"timestamp\":%s}",
                        "GPS", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp.data);

    return payload;
}

static char *get_mqtt_payload_bme680(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN];
    iotex_bme680_get_sensor_data((uint8_t*)&payload, sizeof(payload));
    return payload;
}

static char *get_mqtt_payload_icm42605(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;
    iotex_icm42605_get_sensor_data((uint8_t*)&payload, sizeof(payload));
    return payload;
}

static char *get_mqtt_topic(void)
{
    static uint8_t mqtt_topic[IMEI_LEN + 11] ;
    sprintf(mqtt_topic, "topic/%s",client_id_buf);	
    return mqtt_topic;
}

static int publish_data(struct mqtt_client *client, enum mqtt_qos qos, char* data)
{
    struct mqtt_publish_param param;
    param.message.topic.qos = qos;
    param.message.topic.topic.utf8 = (u8_t *)get_mqtt_topic();
    param.message.topic.topic.size =
            strlen(param.message.topic.topic.utf8);       
    param.message.payload.data = data;
    param.message.payload.len = strlen(data);
    param.message_id = sys_rand32_get();
    param.dup_flag = 0U;
    param.retain_flag = 0U;
    return mqtt_publish(client, &param);
}

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
    int_fast64_t remaining = timeout;  
    int_fast64_t start_time = k_uptime_get();
    int rc;

    while (remaining > 0 && connected ) {
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
    if(connected)
    {
        rc = mqtt_ping(&client);
        PRINT_RESULT("mqtt_ping", rc);
        SUCCESS_OR_BREAK(rc);
        rc = publish_data(&client, 0, get_mqtt_payload_gps(0));
        PRINT_RESULT("mqtt_publish_gps", rc);
    }

}

 void publish_env_sensors_data()
 {
    int rc;
    printk("[%s:%d]\n", __func__, __LINE__);
    if(connected)
    { 
        rc = mqtt_ping(&client);
        PRINT_RESULT("mqtt_ping", rc);
        SUCCESS_OR_BREAK(rc);
        // gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
        rc = publish_data(&client, 0, get_mqtt_payload_devicedata(0));
        PRINT_RESULT("mqtt_publish_devicedata", rc);
        rc = publish_data(&client, 0, get_mqtt_payload_bme680(0));
        PRINT_RESULT("mqtt_publish_bme680", rc);
        rc = publish_data(&client, 0, get_mqtt_payload_icm42605(0));
        PRINT_RESULT("mqtt_publish_icm42605", rc);
    }

}


void main(void)
{
    int err;

    iotex_hal_gpio_init();

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
    provision_certificates();
#endif /* CONFIG_USE_PROVISIONED_CERTIFICATES  */

#if defined(CONFIG_USE_UI_MODULE)
    ui_init(ui_evt_handler);
#endif

    work_init();
    modem_configure();
    printk("modem_configure done\n");
    iotex_modem_get_clock(NULL);

    err = client_init(&client, CONFIG_MQTT_BROKER_HOSTNAME);

    err = mqtt_connect(&client);
    if (err != 0) {
        printk("ERROR: mqtt_connect %d, rebooting...\n", err);
        k_sleep(500);
        sys_reboot(0);
        return;
    }
    printk("MQTT_CONNECT done: %s\n", CONFIG_MQTT_BROKER_HOSTNAME);

    err = fds_init(&client);
    if (err != 0) {
        printk("ERROR: fds_init %d\n", err);
        cloud_error_handler(err);
    } else {
        k_delayed_work_submit(&cloud_reboot_work,
                    CLOUD_CONNACK_WAIT_DURATION);
    }

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
