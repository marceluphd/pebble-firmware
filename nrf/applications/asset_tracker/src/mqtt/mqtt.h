#ifndef  _IOTEX_MQTT_H_
#define  _IOTEX_MQTT_H_

#include <net/mqtt.h>

extern atomic_val_t send_data_enable;

/* Get MQTT topic */
const char *iotex_mqtt_get_topic(void);

/* Get MQTT client id */
const uint8_t *iotex_mqtt_get_client_id(void);

bool iotex_mqtt_is_connected(void);
int iotex_mqtt_client_init(struct mqtt_client *client, struct pollfd *fds);
int iotex_mqtt_publish_data(struct mqtt_client *client, enum mqtt_qos qos, char *data);


#endif