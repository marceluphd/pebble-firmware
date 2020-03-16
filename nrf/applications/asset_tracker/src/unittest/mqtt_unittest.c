#include <net/cloud.h>
#include "unittest.h"
#include "../mqtt/mqtt.h"
#include "../cloud_codec/cloud_codec.h"

static void test_device_info_payload() {

    struct cloud_msg device_info;
    UNITTEST_ASSERT_EQ(iotex_mqtt_payload_device_data(&device_info), 0);

    printk("%s:[%d] %s\n", __func__, device_info.len, device_info.buf);
    free(device_info.buf);
    UNITTEST_AUTO_PASS();
}

void mqtt_unittest() {

    test_device_info_payload();

    UNITTEST_AUTO_PASS();
}