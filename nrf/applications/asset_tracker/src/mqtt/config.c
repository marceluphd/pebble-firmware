#include "mqtt.h"
#include "config.h"
#include "cJSON.h"
#include "cJSON_os.h"

/* Global configure */
static iotex_mqtt_config __config = {
    .upload_period = {
        .gps = 0,
        .devinfo = 30,
        .env_sensor = 10,
        .action_sensor = 60,
    }
};

static char config_buffer[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE];

int iotex_mqtt_parse_config(const uint8_t *payload, uint32_t len) {

    memcpy(config_buffer, payload, len);
    config_buffer[len] = 0;

    printk("[%s]: Received mqtt json config: [%u]%s\n", __func__, len, config_buffer);
    return 0;
}