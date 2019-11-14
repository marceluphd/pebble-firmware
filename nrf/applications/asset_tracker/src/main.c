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
#include <gpio.h>
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

#include <i2c.h>
#include <adc.h>
#include <hal/nrf_saadc.h>
#include <dfu/mcuboot.h>

#include "cloud_codec.h"
#include "env_sensors.h"
#include "orientation_detector.h"
#include "ui.h"
#include "gps_controller.h"
#include "bme/bme680.h"
#include "icm/Icm426xxTransport.h"
#include "icm/Icm426xxDefs.h"
#include "icm/Icm426xxDriver_HL.h"

#define I2C_DEV_BME680    "I2C_2"
#define I2C_DEV_ICM42605  "I2C_1"
#define I2C_ADDR_BME680   0x77
#define I2C_ADDR_ICM42605 0x69
#define SERIF_TYPE ICM426XX_UI_I2C
#define IS_LOW_NOISE_MODE 1
#define TMST_CLKIN_32K 0

#if defined(CONFIG_BSD_LIBRARY)
#include "nrf_inbuilt_key.h"
#endif

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
#include "certificates.h"
#endif

#define CALIBRATION_PRESS_DURATION 	K_SECONDS(5)
#define CLOUD_CONNACK_WAIT_DURATION	K_SECONDS(CONFIG_CLOUD_WAIT_DURATION)

#if defined(CONFIG_FLIP_POLL)
#define FLIP_POLL_INTERVAL		K_MSEC(CONFIG_FLIP_POLL_INTERVAL)
#else
#define FLIP_POLL_INTERVAL		0
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

#if !defined(CONFIG_CLOUD_CLIENT_ID)
#define IMEI_LEN 15
#define CLIENT_ID_LEN (IMEI_LEN + 4)
#else
#define CLIENT_ID_LEN (sizeof(CONFIG_CLOUD_CLIENT_ID) - 1)
#endif

#define KEY_POWER_OFF_TIME 3000  //  press power_key 3S to power off system
#define LED_GREEN     0    //p0.00 == LED_GREEN  0=on 1=off
#define LED_BLUE      1    //p0.01 == LED_BLUE   0=on 1=off
#define LED_RED       2    //p0.02 == LED_RED    0=on 1=off
#define IO_POWER_ON  31    //p0.31 == POWER_ON  1=on 0=off

#define IO_NCHRQ     26    //p0.26 == CHRQ      0=charg 1=off
#define POWER_KEY    30    //p0.30 == POWER_KEY  0=down 1=up

#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2

#define CLOUD_LED_ON_STR "{\"led\":\"on\"}"
#define CLOUD_LED_OFF_STR "{\"led\":\"off\"}"
#define CLOUD_LED_MSK UI_LED_1

#define TIMESTAMP_STR_LEN 50
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

 /* Variables to keep track of nRF cloud user association. */
#if defined(CONFIG_USE_UI_MODULE)
static u8_t ua_pattern[6];
#endif
static int buttons_to_capture;
static int buttons_captured;
static atomic_t pattern_recording;
static bool recently_associated;
static bool association_with_pin;

/* Sensor data */
static struct gps_data gps_data;
static u8_t gps_timestamp_str[TIMESTAMP_STR_LEN];
static struct cloud_channel_data flip_cloud_data;
static struct cloud_channel_data gps_cloud_data;
static struct cloud_channel_data button_cloud_data;

#if CONFIG_MODEM_INFO
static struct modem_param_info modem_param;
static struct cloud_channel_data signal_strength_cloud_data;
static struct cloud_channel_data device_cloud_data;
#endif /* CONFIG_MODEM_INFO */
static atomic_val_t send_data_enable;

/* Flag used for flip detection */
static bool flip_mode_enabled = true;

/* Structures for work */
static struct k_work connect_work;
static struct k_work send_gps_data_work;
static struct k_work send_button_data_work;
static struct k_work send_flip_data_work;
static struct k_delayed_work send_env_data_work;
static struct k_delayed_work flip_poll_work;
static struct k_delayed_work long_press_button_work;
static struct k_delayed_work power_off_button_work;
static struct k_delayed_work cloud_reboot_work;
#if CONFIG_MODEM_INFO
static struct k_work device_status_work;
static struct k_work rsrp_work;
#endif /* CONFIG_MODEM_INFO */

static bool connected=0;

static  struct device *gi2c_dev_bme680;
static  struct device *gi2c_dev_icm42605;

struct device *adc_dev;

static struct inv_icm426xx gicm_driver;
struct device *ggpio_dev;
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
struct bme680_dev gas_sensor;

enum error_type {
    ERROR_CLOUD,
    ERROR_BSD_RECOVERABLE,
    ERROR_BSD_IRRECOVERABLE,
    ERROR_LTE_LC,
    ERROR_SYSTEM_FAULT
};

/* Forward declaration of functions */
static void app_connect(struct k_work *work);
static void flip_send(struct k_work *work);
static void env_data_send(void);
static void sensors_init(void);
static void work_init(void);
static void sensor_data_send(struct cloud_channel_data *data);

static  void publish_env_sensors_data(void); 
static  void publish_gps_data(void);

static void Iotex_I2C_Init(void);
static int8_t Iotex_bme680_init(void);
static int8_t Iotex_bme680_config(void);
static int Iotex_icm42605_Configure(uint8_t is_low_noise_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
                       uint8_t is_rtc_mode);

static int app_get_modemclock();
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

static void send_button_data_work_fn(struct k_work *work)
{
    sensor_data_send(&button_cloud_data);
}

static void send_flip_data_work_fn(struct k_work *work)
{
    sensor_data_send(&flip_cloud_data);
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
    snprintf(gps_timestamp_str, TIMESTAMP_STR_LEN, "%s", app_get_modemclock());
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

/**@brief Callback for sensor trigger events */
static void sensor_trigger_handler(struct device *dev,
            struct sensor_trigger *trigger)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(trigger);

    flip_send(NULL);
}

#if defined(CONFIG_USE_UI_MODULE)
/**@brief Send button presses to cloud */
static void button_send(bool pressed)
{
    static char data[] = "1";

    if (!atomic_get(&send_data_enable)) {
        return;
    }

    if (pressed) {
        data[0] = '1';
    } else {
        data[0] = '0';
    }

    button_cloud_data.data.buf = data;
    button_cloud_data.data.len = strlen(data);
    button_cloud_data.tag += 1;

    if (button_cloud_data.tag == 0) {
        button_cloud_data.tag = 0x1;
    }

    k_work_submit(&send_button_data_work);
}
#endif

/**@brief Poll flip orientation and send to cloud if flip mode is enabled. */
static void flip_send(struct k_work *work)
{
    static enum orientation_state last_orientation_state =
        ORIENTATION_NOT_KNOWN;
    static struct orientation_detector_sensor_data sensor_data;

    if (!flip_mode_enabled || !atomic_get(&send_data_enable)) {
        goto exit;
    }

    if (orientation_detector_poll(&sensor_data) == 0) {
        if (sensor_data.orientation == last_orientation_state) {
            goto exit;
        }

        switch (sensor_data.orientation) {
        case ORIENTATION_NORMAL:
            flip_cloud_data.data.buf = "NORMAL";
            flip_cloud_data.data.len = sizeof("NORMAL") - 1;
            break;
        case ORIENTATION_UPSIDE_DOWN:
            flip_cloud_data.data.buf = "UPSIDE_DOWN";
            flip_cloud_data.data.len = sizeof("UPSIDE_DOWN") - 1;
            break;
        default:
            goto exit;
        }

        last_orientation_state = sensor_data.orientation;

        k_work_submit(&send_flip_data_work);
    }

exit:
    if (work) {
        k_delayed_work_submit(&flip_poll_work,
                    FLIP_POLL_INTERVAL);
    }
}

static void cloud_cmd_handler(struct cloud_command *cmd)
{
    /* Command handling goes here. */
    if (cmd->recipient == CLOUD_RCPT_MODEM_INFO) {
#if CONFIG_MODEM_INFO
        if (cmd->type == CLOUD_CMD_READ) {
            device_status_send(NULL);
        }
#endif
    } else if (cmd->recipient == CLOUD_RCPT_UI) {
        if (cmd->type == CLOUD_CMD_LED_RED) {
            ui_led_set_color(127, 0, 0);
        } else if (cmd->type == CLOUD_CMD_LED_GREEN) {
            ui_led_set_color(0, 127, 0);
        } else if (cmd->type == CLOUD_CMD_LED_BLUE) {
            ui_led_set_color(0, 0, 127);
        }
    }
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
    int err;
    env_sensor_data_t env_data;

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

/**@brief Callback for sensor attached event from nRF Cloud. */
void sensors_start(void)
{
    atomic_set(&send_data_enable, 1);
    sensors_init();

    if (IS_ENABLED(CONFIG_FLIP_POLL)) {
        k_delayed_work_submit(&flip_poll_work, K_NO_WAIT);
    }
}

/**@brief nRF Cloud specific callback for cloud association event. */
static void on_user_pairing_req(const struct cloud_event *evt)
{
    if (evt->data.pair_info.type == CLOUD_PAIR_SEQUENCE) {
        if (!atomic_get(&pattern_recording)) {
            ui_led_set_pattern(UI_CLOUD_PAIRING);
            atomic_set(&pattern_recording, 1);
            buttons_captured = 0;
            buttons_to_capture = *evt->data.pair_info.buf;

            printk("Please enter the user association pattern ");
            printk("using the buttons and switches\n");
        }
    } else if (evt->data.pair_info.type == CLOUD_PAIR_PIN) {
        association_with_pin = true;
        ui_led_set_pattern(UI_CLOUD_PAIRING);
        printk("Waiting for cloud association with PIN\n");
    }
}

#if defined(CONFIG_USE_UI_MODULE)
/**@brief Send user association information to nRF Cloud. */
static void cloud_user_associate(void)
{
    int err;
    struct cloud_msg msg = {
        .buf = ua_pattern,
        .len = buttons_to_capture,
        .endpoint = {
            .type = CLOUD_EP_TOPIC_PAIR
        }
    };

    atomic_set(&pattern_recording, 0);

    err = cloud_send(cloud_backend, &msg);
    if (err) {
        printk("Could not send association message, error: %d\n", err);
        cloud_error_handler(err);
    }
}
#endif

/** @brief Handle procedures after successful association with nRF Cloud. */
void on_pairing_done(void)
{
    if (association_with_pin || (buttons_captured > 0)) {
        recently_associated = true;

        printk("Successful user association.\n");
        printk("The device will attempt to reconnect to ");
        printk("nRF Cloud. It may reset in the process.\n");
        printk("Manual reset may be required if connection ");
        printk("to nRF Cloud is not established within ");
        printk("20 - 30 seconds.\n");
    }

    if (!association_with_pin) {
        return;
    }

    int err;

    printk("Disconnecting from nRF cloud...\n");

    err = cloud_disconnect(cloud_backend);
    if (err == 0) {
        printk("Reconnecting to cloud...\n");
        err = cloud_connect(cloud_backend);
        if (err == 0) {
            return;
        }
        printk("Could not reconnect\n");
    } else {
        printk("Disconnection failed\n");
    }

    printk("Fallback to controlled reboot\n");
    printk("Shutting down LTE link...\n");

#if defined(CONFIG_BSD_LIBRARY)
    err = lte_lc_power_off();
    if (err) {
        printk("Could not shut down link\n");
    } else {
        printk("LTE link disconnected\n");
    }
#endif

#ifdef CONFIG_REBOOT
    printk("Rebooting...\n");
    LOG_PANIC();
    sys_reboot(SYS_REBOOT_COLD);
#endif
    printk("**** Manual reboot required ***\n");
}

void cloud_event_handler(const struct cloud_backend *const backend,
             const struct cloud_event *const evt,
             void *user_data)
{
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case CLOUD_EVT_CONNECTED:
        printk("CLOUD_EVT_CONNECTED\n");
        k_delayed_work_cancel(&cloud_reboot_work);
        ui_led_set_pattern(UI_CLOUD_CONNECTED);
        break;
    case CLOUD_EVT_READY:
        printk("CLOUD_EVT_READY\n");
        ui_led_set_pattern(UI_CLOUD_CONNECTED);
        sensors_start();
        break;
    case CLOUD_EVT_DISCONNECTED:
        printk("CLOUD_EVT_DISCONNECTED\n");
        ui_led_set_pattern(UI_LTE_DISCONNECTED);
        break;
    case CLOUD_EVT_ERROR:
        printk("CLOUD_EVT_ERROR\n");
        break;
    case CLOUD_EVT_DATA_SENT:
        printk("CLOUD_EVT_DATA_SENT\n");
        break;
    case CLOUD_EVT_DATA_RECEIVED:
        printk("CLOUD_EVT_DATA_RECEIVED\n");
        cloud_decode_command(evt->data.msg.buf);
        break;
    case CLOUD_EVT_PAIR_REQUEST:
        printk("CLOUD_EVT_PAIR_REQUEST\n");
        on_user_pairing_req(evt);
        break;
    case CLOUD_EVT_PAIR_DONE:
        printk("CLOUD_EVT_PAIR_DONE\n");
        on_pairing_done();
        break;
    default:
        printk("Unknown cloud event type: %d\n", evt->type);
        break;
    }
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

#if defined(CONFIG_USE_UI_MODULE)
/**@brief Function to keep track of user association input when using
 *	  buttons and switches to register the association pattern.
 *	  nRF Cloud specific.
 */
static void pairing_button_register(struct ui_evt *evt)
{
    if (buttons_captured < buttons_to_capture) {
        if (evt->button == UI_BUTTON_1 &&
            evt->type == UI_EVT_BUTTON_ACTIVE) {
            ua_pattern[buttons_captured++] =
                NRF_CLOUD_UA_BUTTON_INPUT_3;
            printk("Button 1\n");
        } else if (evt->button == UI_BUTTON_2 &&
            evt->type == UI_EVT_BUTTON_ACTIVE) {
            ua_pattern[buttons_captured++] =
                NRF_CLOUD_UA_BUTTON_INPUT_4;
            printk("Button 2\n");
        } else if (evt->button == UI_SWITCH_1) {
            ua_pattern[buttons_captured++] =
                NRF_CLOUD_UA_BUTTON_INPUT_1;
            printk("Switch 1\n");
        } else if (evt->button == UI_SWITCH_2) {
            ua_pattern[buttons_captured++] =
                NRF_CLOUD_UA_BUTTON_INPUT_2;
            printk("Switch 2\n");
        }
    }

    if (buttons_captured == buttons_to_capture) {
        cloud_user_associate();
    }
}
#endif

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

static void power_off_handler(struct k_work *work)
{    
         struct device *dev;
    
         dev = device_get_binding("GPIO_0");
            /* Set LED pin as output */
         gpio_pin_write(dev, 31, 0);	//p0.31 == POWER_OFF


}
/**@brief Initializes and submits delayed work. */
static void work_init(void)
{
    k_work_init(&connect_work, app_connect);
    k_work_init(&send_gps_data_work, send_gps_data_work_fn);
    k_work_init(&send_button_data_work, send_button_data_work_fn);
    k_work_init(&send_flip_data_work, send_flip_data_work_fn);
    k_delayed_work_init(&send_env_data_work, send_env_data_work_fn);
    k_delayed_work_init(&flip_poll_work, flip_send);
    k_delayed_work_init(&long_press_button_work, long_press_handler);
    k_delayed_work_init(&power_off_button_work, power_off_handler);
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

/**@brief Initializes the accelerometer device and
 * configures trigger if set.
 */
static void accelerometer_init(void)
{
    if (!IS_ENABLED(CONFIG_FLIP_POLL) &&
         IS_ENABLED(CONFIG_ACCEL_USE_EXTERNAL)) {

        struct device *accel_dev =
        device_get_binding(CONFIG_ACCEL_DEV_NAME);

        if (accel_dev == NULL) {
            printk("Could not get %s device\n",
                CONFIG_ACCEL_DEV_NAME);
            return;
        }

        struct sensor_trigger sensor_trig = {
            .type = SENSOR_TRIG_THRESHOLD,
        };

        printk("Setting trigger\n");
        int err = 0;

        err = sensor_trigger_set(accel_dev, &sensor_trig,
                sensor_trigger_handler);

        if (err) {
            printk("Unable to set trigger\n");
        }
    }
}

/**@brief Initializes flip detection using orientation detector module
 * and configured accelerometer device.
 */
 /*
static void flip_detection_init(void)
{
    int err;
    struct device *accel_dev =
        device_get_binding(CONFIG_ACCEL_DEV_NAME);

    if (accel_dev == NULL) {
        printk("Could not get %s device\n", CONFIG_ACCEL_DEV_NAME);
        return;
    }

    orientation_detector_init(accel_dev);

    if (!IS_ENABLED(CONFIG_ACCEL_CALIBRATE)) {
        return;
    }

    err = orientation_detector_calibrate();
    if (err) {
        printk("Could not calibrate accelerometer device: %d\n", err);
    }
}



static void button_sensor_init(void)
{
    button_cloud_data.type = CLOUD_CHANNEL_BUTTON;
    button_cloud_data.tag = 0x1;
}
*/
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
    /* Iotex Init I2C */
    Iotex_I2C_Init();
    /* Iotex Init BME680 */
    Iotex_bme680_init();
    Iotex_bme680_config();
    /* Iotex Init ICM42605 */
    Iotex_icm42605_Init();
    Iotex_icm42605_Configure((uint8_t)IS_LOW_NOISE_MODE,
                 ICM426XX_ACCEL_CONFIG0_FS_SEL_4g,
                 ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
                 ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ,
                 ICM426XX_GYRO_CONFIG0_ODR_1_KHZ,
                 (uint8_t)TMST_CLKIN_32K);
    adc_init();
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
    if (pattern_recording) {
        pairing_button_register(&evt);
        return;
    }
/*
    if (IS_ENABLED(CONFIG_CLOUD_BUTTON) &&
       (evt.button == CONFIG_CLOUD_BUTTON_INPUT)) {
        button_send(evt.type == UI_EVT_BUTTON_ACTIVE ? 1 : 0);
    }

    if (IS_ENABLED(CONFIG_ACCEL_USE_SIM) && (evt.button == FLIP_INPUT)
       && atomic_get(&send_data_enable)) {
        flip_send(NULL);
    }
*/
    if (IS_ENABLED(CONFIG_GPS_CONTROL_ON_LONG_PRESS) &&
       (evt.button == UI_BUTTON_1)) {
        if (evt.type == UI_EVT_BUTTON_ACTIVE) {
            k_delayed_work_submit(&long_press_button_work,
            K_SECONDS(2));// chang 5S to 2S 
            k_delayed_work_submit(&power_off_button_work,
            K_SECONDS(5));
        } else {
            k_delayed_work_cancel(&long_press_button_work);
            k_delayed_work_cancel(&power_off_button_work);
        }
    }

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
        gpio_pin_write(ggpio_dev, LED_BLUE, 0);	//p0.00 == LED_BLUE OFF
        gpio_pin_write(ggpio_dev, LED_GREEN, 1); //LED_GREEN = ON

        printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
        break;

    case MQTT_EVT_DISCONNECT:
        connected=0;
        gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
        gpio_pin_write(ggpio_dev, LED_GREEN, 0); //LED_GREEN = ON
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
static void Iotex_I2C_Init(void)
{

    uint8_t WhoAmI = 0u;

    printk("Starting i2c Init\n");

    gi2c_dev_bme680 = device_get_binding(I2C_DEV_BME680);
        gi2c_dev_icm42605 = device_get_binding(I2C_DEV_ICM42605);

    if ((!gi2c_dev_bme680)||(!gi2c_dev_icm42605)) {
        printk("I2C: Device driver not found.\n");
        return;
    }

    if (i2c_reg_read_byte(gi2c_dev_bme680, I2C_ADDR_BME680, 0xd0, &WhoAmI) != 0) { // stop wroking at this line
        printk("Error on i2c_read(bme680)\n");
    } else {
        printk("no error\r\n");
    }

    printk("BMD680 ID = 0x%x\r\n", WhoAmI);

    if (i2c_reg_read_byte(gi2c_dev_icm42605, I2C_ADDR_ICM42605, 0x75, &WhoAmI) != 0) { // stop wroking at this line
        printk("Error on i2c_read(icm42605)\n");
    } else {
        printk("no error\r\n");
    }

    printk("ICM42605 ID = 0x%x\r\n", WhoAmI);
}
////////////////////// BME680 START//////////////////////////

static void user_delay_ms(uint32_t period)
{
    k_sleep(period);
}


int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    rslt= i2c_burst_read(gi2c_dev_bme680,I2C_ADDR_BME680,reg_addr,reg_data,((uint32_t)len));
    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    u16_t i;
    for(i=0;i<len;i++)
    {
         i2c_reg_write_byte(gi2c_dev_bme680, I2C_ADDR_BME680, reg_addr+i, *(reg_data+i));
    }
    return rslt;
}

static int8_t Iotex_bme680_init(void)
 {
    gas_sensor.dev_id = I2C_ADDR_BME680;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = user_i2c_read;
    gas_sensor.write = user_i2c_write;
    gas_sensor.delay_ms = user_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor.amb_temp = 25;

    int8_t rslt = BME680_OK;
    rslt = bme680_init(&gas_sensor);
    return rslt;
}

static int8_t Iotex_bme680_config(void)
{
    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    int8_t rslt = BME680_OK;
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
     return rslt;
}

static int8_t Iotex_bme680_Reading_sensor_data( uint8_t* str)
{
    uint16_t meas_period;
    int8_t rslt = BME680_OK;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    struct bme680_field_data data;

   // while(1)
   // {
        user_delay_ms(500); /* Delay till the measurement is ready */

        rslt = bme680_get_sensor_data(&data, &gas_sensor);

        printf("{BME680  T: %.2f degC, P: %.2f hPa, H %.2f %%rH", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            printf(", G: %d ohms", data.gas_resistance);

        printf("}\n");

        sprintf(str,"{\"Device\":\"%s\",\"T(degC)\":%.2f,\"P(hPa)\":%.2f, \"H(%%rH)\":%.2f, \"G(ohms)\":%d, \"timestamp\":%s}" ,"BME680",data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f, data.gas_resistance, app_get_modemclock());

        /* Trigger the next measurement if you would like to read data out continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
        }
   // }
    return rslt;
}
////////////////////// BME680 END//////////////////////////

////////////////////// ICM42605 START//////////////////////////
void inv_icm426xx_sleep_us(uint32_t us)
{
    k_busy_wait(us);
}

uint64_t inv_icm426xx_get_time_us(void)
{
    return (SYS_CLOCK_HW_CYCLES_TO_NS64(k_cycle_get_32())/1000);
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
    uint8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    rslt= i2c_burst_read(gi2c_dev_icm42605, I2C_ADDR_ICM42605,reg,rbuffer,rlen);
    return  rslt;
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
    uint8_t rslt = 0;
    for(uint32_t i=0; i<wlen; i++)
     {
    //rslt = inv_spi_master_write_register(INV_SPI_AP, reg+i, 1, &wbuffer[i]);
        rslt =  i2c_reg_write_byte(gi2c_dev_icm42605, I2C_ADDR_ICM42605, reg+i, wbuffer[i]);
        if(rslt)  return rslt;
    }
    return rslt;

}

int Iotex_icm42605_Init(void)
{
    int rc = 0;
    uint8_t who_am_i;
    static struct inv_icm426xx_serif icm_serif;
    //  printf(" initialize Icm426xx.1\n");
    icm_serif.context   = 0;        /* no need */
    icm_serif.read_reg  = inv_io_hal_read_reg;
    icm_serif.write_reg = inv_io_hal_write_reg;
    icm_serif.max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
    icm_serif.max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
    icm_serif.serif_type = SERIF_TYPE;
    // printf(" initialize Icm426xx.2\n");

    rc = inv_icm426xx_init(&gicm_driver, &icm_serif, NULL);
    // printf(" initialize Icm426xx.3\n");
    rc |= inv_icm426xx_configure_fifo(&gicm_driver, INV_ICM426XX_FIFO_DISABLED);
    //  printf(" initialize Icm426xx.4\n");
    if(rc != INV_ERROR_SUCCESS) {
        printf("!!! ERROR : failed to initialize Icm426xx.\n");
        return rc;
    }

    /* Check WHOAMI */
    rc = inv_icm426xx_get_who_am_i(&gicm_driver, &who_am_i);
    if(rc != INV_ERROR_SUCCESS) {
        printf("!!! ERROR : failed to read Icm426xx whoami value.\n");
        return rc;
    }

    if(who_am_i != ICM_WHOAMI) {
        printf("!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)\n", who_am_i, ICM_WHOAMI);
        return INV_ERROR;
    }
        else    printf("WHOAMI value. Got 0x%02x\n", who_am_i);

    return rc;
}

static int Iotex_icm42605_Configure(uint8_t is_low_noise_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
                       uint8_t is_rtc_mode)
{
    int rc = 0;

    rc |= inv_icm426xx_enable_clkin_rtc(&gicm_driver, is_rtc_mode);

    rc |= inv_icm426xx_set_accel_fsr(&gicm_driver, acc_fsr_g);
    rc |= inv_icm426xx_set_gyro_fsr(&gicm_driver, gyr_fsr_dps);

    rc |= inv_icm426xx_set_accel_frequency(&gicm_driver, acc_freq);
    rc |= inv_icm426xx_set_gyro_frequency(&gicm_driver, gyr_freq);

    if (is_low_noise_mode)
        rc |= inv_icm426xx_enable_accel_low_noise_mode(&gicm_driver);
    else
        rc |= inv_icm426xx_enable_accel_low_power_mode(&gicm_driver);

    rc |= inv_icm426xx_enable_gyro_low_noise_mode(&gicm_driver);

    /* Wait Max of ICM426XX_GYR_STARTUP_TIME_US and ICM426XX_ACC_STARTUP_TIME_US*/
    (ICM426XX_GYR_STARTUP_TIME_US > ICM426XX_ACC_STARTUP_TIME_US) ? inv_icm426xx_sleep_us(ICM426XX_GYR_STARTUP_TIME_US) : inv_icm426xx_sleep_us(ICM426XX_ACC_STARTUP_TIME_US);

    return rc;
}

void inv_icm42605_format_data(const uint8_t endian, const uint8_t *in, uint16_t *out)
{
    if(endian == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN)
        *out = (in[0] << 8) | in[1];
    else
        *out = (in[1] << 8) | in[0];
}

int Iotex_icm42605_Reading_sensor_data(struct inv_icm426xx * s, uint8_t * str)
{
    int status = 0;
    uint8_t int_status;
    uint8_t temperature[2];
    uint8_t accel[ACCEL_DATA_SIZE];
    uint8_t gyro[GYRO_DATA_SIZE];
    uint16_t ftemperature;
    uint16_t faccel[ACCEL_DATA_SIZE/2];
    uint16_t fgyro[GYRO_DATA_SIZE/2];
   
    /* Ensure data ready status bit is set */
    status |= inv_icm426xx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);
    if(status)
        return status;

    if(int_status & BIT_INT_STATUS_DRDY) {

        status = inv_icm426xx_read_reg(s, MPUREG_TEMP_DATA0_UI, TEMP_DATA_SIZE, temperature);
        inv_icm42605_format_data(s->endianess_data, temperature, (uint16_t *)&(ftemperature));

        status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_DATA_X0_UI, ACCEL_DATA_SIZE, accel);
        inv_icm42605_format_data(s->endianess_data, &accel[0], (uint16_t *)&faccel[0]);
        inv_icm42605_format_data(s->endianess_data, &accel[2], (uint16_t *)&faccel[1]);
        inv_icm42605_format_data(s->endianess_data, &accel[4], (uint16_t *)&faccel[2]);

        status |= inv_icm426xx_read_reg(s, MPUREG_GYRO_DATA_X0_UI, GYRO_DATA_SIZE, gyro);
        inv_icm42605_format_data(s->endianess_data, &gyro[0], (uint16_t *)&fgyro[0]);
        inv_icm42605_format_data(s->endianess_data, &gyro[2], (uint16_t *)&fgyro[1]);
        inv_icm42605_format_data(s->endianess_data, &gyro[4], (uint16_t *)&fgyro[2]);
        
        if((faccel[0] != INVALID_VALUE_FIFO) && (fgyro[0] != INVALID_VALUE_FIFO))
            printf("{ICM42605   AX:%d, AY:%d, AZ:%d, TEMP:%.2f,GX:%d, GY:%d, GZ:%d}\n",
                (int16_t)faccel[0], (int16_t)faccel[1], (int16_t)faccel[2],
                (ftemperature/132.48)+25,
                (int16_t)fgyro[0],(int16_t) fgyro[1],(int16_t) fgyro[2]);
              
                
        // sprintf(buf, "{\"String\":\"%s\", \"Value\":%d}", "Hello World!", 12345);
        sprintf(str, "{\"Device\":\"%s\",\"AX\":%d, \"AY\":%d, \"AZ\":%d, \"TEMP\":%.2f,\"GX\":%d,\"GY\":%d, \"GZ\":%d, \"timestamp\":%s}","ICM42605",
            (int16_t)faccel[0], (int16_t)faccel[1], (int16_t)faccel[2],
            (ftemperature/132.48)+25,
            (int16_t)fgyro[0],(int16_t) fgyro[1],(int16_t) fgyro[2],
            app_get_modemclock());
        
    }
    /*else: Data Ready was not set*/
    
    return status;
}
////////////////////// ICM42605 END//////////////////////////

///////////////////////////////ADC START/////////////////////////////////////////

static const struct adc_channel_cfg m_1st_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_1ST_CHANNEL_ID,
    .input_positive = ADC_1ST_CHANNEL_INPUT,
};

float  adc_sample(void)
{
    int ret;
    float adc_voltage = 0;
    s16_t sample_buffer;
    const struct adc_sequence sequence = {
        .channels = BIT(ADC_1ST_CHANNEL_ID),
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
    if (!adc_dev) {
        return -1;
    }

    ret = adc_read(adc_dev, &sequence);
    PRINT_RESULT("ADC read",ret);

    adc_voltage = (float)(((float)sample_buffer / 1023.0f) * 2 * 3600.0f)/1000;
    //printk("ADC raw value: %d\n", sample_buffer);
    //printf("Measured voltage: %f mV\n", adc_voltage);
    return adc_voltage;
}

int adc_init(void)
{
    int err;
    adc_dev = device_get_binding("ADC_0");
    if (!adc_dev) {
        printk("device_get_binding ADC_0 failed\n");
    }
    err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    if (err) {
        printk("Error in adc setup: %d\n", err);
    }
    
    NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;
    adc_sample();
    return err;
}
/////////////////////////ADC END//////////////////////////////////////

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
            client_id_buf, adc_sample(), atoi(snr), app_get_modemclock());
    return payload;
}


static char *get_mqtt_payload_gps(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;

    printf("{GPS  latitude:%f, longitude:%f, timestamp %s\n", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp_str);
    sprintf(payload,"{\"Device\":\"%s\",\"latitude\":%f,\"longitude\":%f, \"timestamp\":%s}",
                        "GPS", gps_data.pvt.latitude, gps_data.pvt.longitude, gps_timestamp_str);

    return payload;
}

static char *get_mqtt_payload_bme680(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;
    Iotex_bme680_Reading_sensor_data((uint8_t*)&payload);
    return payload;
}

static char *get_mqtt_payload_icm42605(enum mqtt_qos qos)
{
    static uint8_t payload[SENSOR_PAYLOAD_MAX_LEN] ;
    Iotex_icm42605_Reading_sensor_data(&gicm_driver,(uint8_t*)&payload);
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

/////////////////////////////GPIO START///////////////////////////////////////
void chrq_callback(struct device *port,
           struct gpio_callback *cb, u32_t pins)
{
    u32_t chrq;
    printk("chrq_Pin %d triggered\n", 26);
    gpio_pin_read(ggpio_dev,IO_NCHRQ,&chrq);
    gpio_pin_write(ggpio_dev, LED_RED, chrq);  //if chrq=0 ,turn LED_RED on
    gpio_pin_write(ggpio_dev, LED_GREEN, (chrq+1)%2); //LED_GREEN =! LED_RED
 }

static struct gpio_callback chrq_gpio_cb;

void Iotex_Gpio_Init(void)
{
    uint32_t chrq;
    ggpio_dev = device_get_binding("GPIO_0");
    /* Set LED pin as output */
    gpio_pin_configure(ggpio_dev, LED_GREEN, GPIO_DIR_OUT); //p0.00 == LED_GREEN
    gpio_pin_configure(ggpio_dev, LED_BLUE, GPIO_DIR_OUT); //p0.01 == LED_BLUE
    gpio_pin_configure(ggpio_dev, LED_RED, GPIO_DIR_OUT); //p0.02 == LED_RED
    gpio_pin_configure(ggpio_dev, IO_POWER_ON, GPIO_DIR_OUT); //p0.31 == POWER_ON
   
    gpio_pin_write(ggpio_dev,LED_GREEN , 0);	//p0.00 == LED_GREEN ON
    gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
    gpio_pin_write(ggpio_dev, LED_RED, 1);	//p0.00 == LED_RED OFF
    gpio_pin_write(ggpio_dev, IO_POWER_ON, 1);	//p0.31 == POWER_ON

    gpio_pin_configure(ggpio_dev, IO_NCHRQ,
                 (GPIO_DIR_IN | GPIO_INT |
                  GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE |
                  GPIO_INT_DEBOUNCE));

    gpio_init_callback(&chrq_gpio_cb, chrq_callback, BIT(IO_NCHRQ));
    gpio_add_callback(ggpio_dev, &chrq_gpio_cb);
    gpio_pin_enable_callback(ggpio_dev, IO_NCHRQ);
    
    gpio_pin_read(ggpio_dev,IO_NCHRQ,&chrq);   //get chrq status
    gpio_pin_write(ggpio_dev, LED_RED, chrq);   // chage LED_RED as chrq    
    gpio_pin_write(ggpio_dev, LED_GREEN, (chrq+1)%2);  //if chrq ,turn off LED_GREEN
}

////////////////////////////////GPIO END/////////////////////////////////////////
bool isLeap(uint32_t year) {
    return (((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0)));
}
static int app_get_modemclock()
{
    enum at_cmd_state at_state;
    static char cclk_r_buf[TIMESTAMP_STR_LEN];
    static char epoch_buf[TIMESTAMP_STR_LEN];

    int err = at_cmd_write("AT+CCLK?", cclk_r_buf, TIMESTAMP_STR_LEN, &at_state);
    if (err) {
        printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
    }
    printk("AT CMD Modem time is:%s\n",cclk_r_buf);

    //cclk_r_buf: +CCLK: "19/11/12,19:52:09-32"

    uint32_t YY,MM,DD,hh,mm,ss;
    double epoch;
    int daysPerMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // num of years since 1900, the formula works only for 2xxx
    YY = (cclk_r_buf[8]-'0') * 10 + (cclk_r_buf[9]-'0') + 100;
    MM = (cclk_r_buf[11]-'0') * 10 + (cclk_r_buf[12]-'0');
    DD = (cclk_r_buf[14]-'0') * 10 + (cclk_r_buf[15]-'0');
    hh = (cclk_r_buf[17]-'0') * 10 + (cclk_r_buf[18]-'0');
    mm = (cclk_r_buf[20]-'0') * 10 + (cclk_r_buf[21]-'0');
    ss = (cclk_r_buf[23]-'0') * 10 + (cclk_r_buf[24]-'0');

    if (isLeap(YY+1900)) daysPerMonth[2] = 29;
    // accumulate
    for (int i = 1; i <=12; i++) daysPerMonth[i] += daysPerMonth[i-1];

    epoch  = ss + mm * 60 + hh * 3600 + (daysPerMonth[ MM - 1] + DD -1) * 86400 +
    (YY-70)*31536000L + ((YY-69)/4)*86400L -
    ((YY-1)/100)*86400L + ((YY+299)/400)*86400L;

    snprintf(epoch_buf, TIMESTAMP_STR_LEN, "%.0f", epoch);
    printk("UTC epoch %s\n", epoch_buf);

    return epoch_buf;
}

void main(void)
{
    int err;
    struct device *dev;

    Iotex_Gpio_Init();

#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
    provision_certificates();
#endif /* CONFIG_USE_PROVISIONED_CERTIFICATES  */

#if defined(CONFIG_USE_UI_MODULE)
    ui_init(ui_evt_handler);
#endif

    work_init();
    modem_configure();
    printk("modem_configure done\n");
    app_get_modemclock();
    err = client_init(&client, CONFIG_MQTT_BROKER_HOSTNAME);

    err = mqtt_connect(&client);
    if (err != 0) {
        printk("ERROR: mqtt_connect %d, rebooting...\n", err);
        k_sleep(500);
        sys_reboot(0);
        return;
    }
    printk("MQTT_CONNECT done\n");

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
        printk("mqtt live\n");

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

    gpio_pin_write(ggpio_dev, LED_RED, 0);	//p0.00 == LED_BLUE OFF
    k_sleep(500);
    gpio_pin_write(ggpio_dev, LED_RED, 1);	//p0.00 == LED_BLUE OFF
    k_sleep(500);
    gpio_pin_write(ggpio_dev, LED_RED, 0);	//p0.00 == LED_BLUE OFF
    k_sleep(500);
    sys_reboot(0);
}
