#ifndef  _IOTEX_MQTT_CONFIG_H_
#define  _IOTEX_MQTT_CONFIG_H_

#include <stdint.h>

typedef struct {
    struct {
        uint16_t gps;
        uint16_t devinfo;
        uint16_t env_sensor;
        uint16_t action_sensor;
    } upload_period;

} iotex_mqtt_config;

int iotex_mqtt_parse_config(const uint8_t *payload, uint32_t len);

#endif /* _IOTEX_MQTT_CONFIG_H_ */