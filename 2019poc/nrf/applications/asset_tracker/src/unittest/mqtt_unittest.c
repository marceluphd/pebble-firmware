#include <net/cloud.h>
#include "unittest.h"
#include "../mqtt/mqtt.h"
#include "../cloud_codec/cloud_codec.h"

static void test_device_info_payload() {

    struct cloud_msg device_info;
    UNITTEST_ASSERT_EQ(iotex_mqtt_get_devinfo_payload(&device_info), 0);

    printk("%s:[%d] %s\n", __func__, device_info.len, device_info.buf);
    free(device_info.buf);
    UNITTEST_AUTO_PASS();
}

static void test_env_sensor_payload() {

    struct cloud_msg env_sonsor;
    UNITTEST_ASSERT_EQ(iotex_mqtt_get_env_sensor_payload(&env_sonsor), 0);

    printk("%s:[%d] %s\n", __func__, env_sonsor.len, env_sonsor.buf);
    free(env_sonsor.buf);
    UNITTEST_AUTO_PASS();
}

static void test_action_sensor_payload() {

    struct cloud_msg action_sensor;
    UNITTEST_ASSERT_EQ(iotex_mqtt_get_action_sensor_payload(&action_sensor), 0);

    printk("%s:[%d] %s\n", __func__, action_sensor.len, action_sensor.buf);
    free(action_sensor.buf);
    UNITTEST_AUTO_PASS();
}


void mqtt_unittest() {

    test_device_info_payload();
    test_env_sensor_payload();
    test_action_sensor_payload();

    UNITTEST_AUTO_PASS();
}