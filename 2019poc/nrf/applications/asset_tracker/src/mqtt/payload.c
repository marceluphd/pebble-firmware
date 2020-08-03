#include <stdio.h>
#include <net/cloud.h>
#include "cJSON.h"
#include "cJSON_os.h"
#include "mqtt.h"
#include "hal/hal_adc.h"
#include "nvs/local_storage.h"
#include "modem/modem_helper.h"
#include "bme/bme680_helper.h"
#include "icm/icm42605_helper.h"


#define SENSOR_PAYLOAD_MAX_LEN 150

static int json_add_obj(cJSON *parent, const char *str, cJSON *item) {
    cJSON_AddItemToObject(parent, str, item);
    return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item) {

    cJSON *json_str;
    json_str = cJSON_CreateString(item);

    if (json_str == NULL) {
        return -ENOMEM;
    }

    return json_add_obj(parent, str, json_str);
}


int iotex_mqtt_get_devinfo_payload(struct cloud_msg *output) {

    int ret = 0;
    char *payload;
    cJSON *root_obj = cJSON_CreateObject();
    cJSON *vbat_obj = cJSON_CreateNumber(iotex_hal_adc_sample());
    cJSON *snr_obj = cJSON_CreateNumber(iotex_model_get_signal_quality());

    if (!root_obj || !vbat_obj || !snr_obj) {
        goto out;
    }

    /* Format device info */
    ret += json_add_str(root_obj, "Device", iotex_mqtt_get_client_id());
    ret += json_add_str(root_obj, "timestamp", iotex_modem_get_clock(NULL));
    ret += json_add_obj(root_obj, "VBAT", vbat_obj);
    ret += json_add_obj(root_obj, "SNR", snr_obj);

    if (ret != 0) {
        goto out;
    }

    payload = cJSON_PrintUnformatted(root_obj);
    cJSON_Delete(root_obj);

    output->buf = payload;
    output->len = strlen(payload);
    return 0;

out:
    cJSON_Delete(root_obj);
    cJSON_Delete(vbat_obj);
    cJSON_Delete(snr_obj);
    return -ENOMEM;
}

int iotex_mqtt_get_env_sensor_payload(struct cloud_msg *output) {

    iotex_storage_bme680 data;

    if (iotex_bme680_get_sensor_data(&data)) {
        return -1;
    }

    int ret = 0;
    char *payload;
    const char *device = "BME680";
    cJSON *root_obj = cJSON_CreateObject();
    cJSON *humidity = cJSON_CreateNumber(data.humidity);
    cJSON *pressure = cJSON_CreateNumber(data.pressure);
    cJSON *temperature = cJSON_CreateNumber(data.temperature);
    cJSON *gas_resistance = cJSON_CreateNumber(data.gas_resistance);

    if (!root_obj || !humidity || !pressure || !temperature || !gas_resistance) {
        goto out;
    }

    ret += json_add_str(root_obj, "Device", device);
    ret += json_add_obj(root_obj, "humidity", humidity);
    ret += json_add_obj(root_obj, "pressure", pressure);
    ret += json_add_obj(root_obj, "temperature", temperature);
    ret += json_add_obj(root_obj, "gas_resistance", gas_resistance);
    ret += json_add_str(root_obj, "timestamp", iotex_modem_get_clock(NULL));

    if (ret != 0) {
        goto out;
    }

    payload = cJSON_PrintUnformatted(root_obj);
    cJSON_Delete(root_obj);

    output->buf = payload;
    output->len = strlen(payload);
    return 0;

out:
    cJSON_Delete(root_obj);
    cJSON_Delete(humidity);
    cJSON_Delete(pressure);
    cJSON_Delete(temperature);
    cJSON_Delete(gas_resistance);
    return -ENOMEM;
}

int iotex_mqtt_get_action_sensor_payload(struct cloud_msg *output) {

    int i;
    int ret = 0;
    char *payload;
    const char *device = "ICM42605";

    iotex_storage_icm42605 data;
    int gyroscope[ARRAY_SIZE(data.gyroscope)];
    int accelerometer[ARRAY_SIZE(data.accelerometer)];

    if (iotex_icm42605_get_sensor_data(&data)) {
        return -1;
    }

    for (i = 0; i < ARRAY_SIZE(data.gyroscope); i++) {
        gyroscope[i] = data.gyroscope[i];
    }

    for (i = 0; i < ARRAY_SIZE(data.accelerometer); i++) {
        accelerometer[i] = data.accelerometer[i];
    }

    cJSON *root_obj = cJSON_CreateObject();
    cJSON *temperature_obj = cJSON_CreateNumber(data.temperature);
    cJSON *gyroscope_obj = cJSON_CreateIntArray(gyroscope, ARRAY_SIZE(gyroscope));
    cJSON *accelerometer_obj = cJSON_CreateIntArray(accelerometer, ARRAY_SIZE(accelerometer));

    if (!root_obj || !temperature_obj || !gyroscope_obj || !accelerometer_obj) {
        goto out;
    }

    ret += json_add_str(root_obj, "Device", device);
    ret += json_add_obj(root_obj, "gyroscope", gyroscope_obj);
    ret += json_add_obj(root_obj, "temperature", temperature_obj);
    ret += json_add_obj(root_obj, "accelerometer", accelerometer_obj);
    ret += json_add_str(root_obj, "timestamp", iotex_modem_get_clock(NULL));

    if (ret != 0) {
        goto out;
    }

    payload = cJSON_PrintUnformatted(root_obj);
    cJSON_Delete(root_obj);

    output->buf = payload;
    output->len = strlen(payload);
    return 0;

out:
    cJSON_Delete(root_obj);
    cJSON_Delete(gyroscope_obj);
    cJSON_Delete(temperature_obj);
    cJSON_Delete(accelerometer_obj);
    return -ENOMEM;

    return 0;
}