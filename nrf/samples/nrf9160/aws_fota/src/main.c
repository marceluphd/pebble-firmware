/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <uart.h>
#include <string.h>

#include <at_cmd.h>
#include <lte_lc.h>
#include <net/mqtt.h>
#include <net/socket.h>
#include <net/aws_fota.h>

#include <dfu/mcuboot.h>
#include <misc/reboot.h>
#include "bme/bme680.h"
#include "icm/Icm426xxTransport.h"
#include "icm/Icm426xxDefs.h"
#include "icm/Icm426xxDriver_HL.h"

#include <kernel.h>
//#include <soc/nrfx_coredep.h>
#include <i2c.h>
#include <gpio.h>
#include <adc.h>
#include <hal/nrf_saadc.h>

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

//static int8_t Iotex_bme680_Reading_sensor_data(static uint8_t *);
//static int8_t Iotex_bme680_config(void);
//static int8_t Iotex_bme680_init(void);
//int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
//int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
//static void user_delay_ms(uint32_t period);
//static void Iotex_i2c_Init(void);

static bool connected=0;

static  struct device *gi2c_dev_bme680;
static  struct device *gi2c_dev_icm42605;

struct device *adc_dev;

static struct inv_icm426xx gicm_driver;
struct device *ggpio_dev;
static u32_t gstart_time;
static u8_t client_id_buf[CLIENT_ID_LEN+1];

/* Buffers for MQTT client. */
static u8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE];

/* MQTT Broker details. */
static struct sockaddr_storage broker_storage;

/* File descriptor */
static struct pollfd fds;

static int   gps_fd;
/* Set to true when application should teardown and reboot */
static bool do_reboot;
struct bme680_dev gas_sensor;

#if defined(CONFIG_BSD_LIBRARY)
/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
	printk("bsdlib recoverable error: %u\n", err);
}

/**@brief Irrecoverable BSD library error. */
void bsd_irrecoverable_error_handler(uint32_t err)
{
	printk("bsdlib irrecoverable error: %u\n", err);

	__ASSERT_NO_MSG(false);
}

#endif /* defined(CONFIG_BSD_LIBRARY) */

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

	err = aws_fota_mqtt_evt_handler(c, evt);
	if (err > 0) {
		/* Event handled by FOTA library so we can skip it */
		return;
	} else if (err < 0) {
		printk("aws_fota_mqtt_evt_handler: Failed! %d\n", err);
		printk("Disconnecting MQTT client...\n");
		err = mqtt_disconnect(c);
		if (err) {
			printk("Could not disconnect: %d\n", err);
		}
	}

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}
        connected=1;
        gpio_pin_write(ggpio_dev, LED_BLUE, 0);	//p0.00 == LED_BLUE OFF
        gpio_pin_write(ggpio_dev, LED_GREEN, 1); //LED_GREEN = ON

		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
		if (err) {
			printk("Unable to initialize AWS jobs upon "
			       "connection\n");
			err = mqtt_disconnect(c);
			if (err) {
				printk("Could not disconnect: %d\n", err);
			}
		}
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
					CLOUD_CA_CERTIFICATE,
					strlen(CLOUD_CA_CERTIFICATE));
		printk("nrf_inbuilt_key_write => result=%d\n", err);
		if (err) {
			printk("CLOUD_CA_CERTIFICATE err: %d", err);
			return err;
		}

		/* Provision Private Certificate. */
		err = nrf_inbuilt_key_write(
			CONFIG_CLOUD_CERT_SEC_TAG,
			NRF_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
			CLOUD_CLIENT_PRIVATE_KEY,
			strlen(CLOUD_CLIENT_PRIVATE_KEY));
		printk("nrf_inbuilt_key_write => result=%d\n", err);
		if (err) {
			printk("NRF_CLOUD_CLIENT_PRIVATE_KEY err: %d", err);
			return err;
		}

		/* Provision Public Certificate. */
		err = nrf_inbuilt_key_write(
			CONFIG_CLOUD_CERT_SEC_TAG,
			NRF_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
				 CLOUD_CLIENT_PUBLIC_CERTIFICATE,
				 strlen(CLOUD_CLIENT_PUBLIC_CERTIFICATE));
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

/**@brief Configures modem to provide LTE link.
 *
 * Blocks until link is successfully established.
 */
static void modem_configure(void)
{
#if defined(CONFIG_LTE_LINK_CONTROL)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	} else {
		int err;

		printk("LTE Link Connecting ...\n");
		err = lte_lc_init_and_connect();
		__ASSERT(err == 0, "LTE link could not be established.");
		printk("LTE Link Connected!\n");
	}
#endif
}

static void aws_fota_cb_handler(enum aws_fota_evt_id evt)
{
	switch (evt) {
	case AWS_FOTA_EVT_DONE:
		printk("AWS_FOTA_EVT_DONE, rebooting to apply update.\n");
		do_reboot = true;
		break;

	case AWS_FOTA_EVT_ERROR:
		printk("AWS_FOTA_EVT_ERROR\n");
		break;
	}
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
    //printk("SNR:%d\n",atoi(id_buf));

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


        sprintf(str,"{\"Device\":\"%s\",\"T(degC)\":%.2f,\"P(hPa)\":%.2f, \"H(%%rH)\":%.2f, \"G(ohms)\":%d}" ,"BME680",data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f, data.gas_resistance);

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

int Iotex_icm42605_Configure(uint8_t is_low_noise_mode,
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
        sprintf(str, "{\"Device\":\"%s\",\"AX\":%d, \"AY\":%d, \"AZ\":%d, \"TEMP\":%.2f,\"GX\":%d,\"GY\":%d, \"GZ\":%d}","ICM42605",
			(int16_t)faccel[0], (int16_t)faccel[1], (int16_t)faccel[2],
			(ftemperature/132.48)+25,
			(int16_t)fgyro[0],(int16_t) fgyro[1],(int16_t) fgyro[2]);
		
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


#define BUFFER_SIZE 1

static s16_t m_sample_buffer[BUFFER_SIZE];

float  adc_sample(void)
{
	int ret;
        float adc_voltage = 0;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};
	if (!adc_dev) {
		return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	printk("ADC read err: %d\n", ret);

	for (int i = 0; i < BUFFER_SIZE; i++) {
		adc_voltage = (float)(((float)m_sample_buffer[i] / 1023.0f) *2*
				      3600.0f)/1000;
	//	printk("ADC raw value: %d\n", m_sample_buffer[i]);
	//	printf("Measured voltage: %f mV\n", adc_voltage);
	}
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
	return err;
}
/////////////////////////ADC END//////////////////////////////////////

static char *get_mqtt_payload_devicedata(enum mqtt_qos qos)
{
	static uint8_t payload[100] ;
	static uint8_t snr[4] ;

    signal_quality_get(snr);
    sprintf(payload, "{\"Device\":\"%s\",\"VBAT\":%.2f, \"SNR\":%d}",client_id_buf,adc_sample(),atoi(snr));

    //  Iotex_bme680_Reading_sensor_data((uint8_t*)&payload);
	//payload[strlen(payload) - 1] = '0' + qos;

	return payload;
}

static char *get_mqtt_payload_bme680(enum mqtt_qos qos)
{
	static uint8_t payload[100] ;
    Iotex_bme680_Reading_sensor_data((uint8_t*)&payload);
	//payload[strlen(payload) - 1] = '0' + qos;
	return payload;
}

static char *get_mqtt_payload_icm42605(enum mqtt_qos qos)
{
	static uint8_t payload[100] ;
    Iotex_icm42605_Reading_sensor_data(&gicm_driver,(uint8_t*)&payload);
	//payload[strlen(payload) - 1] = '0' + qos;
	return payload;
}

static char *get_mqtt_topic(void)
{
    static uint8_t mqtt_topic[IMEI_LEN + 10] ;
    sprintf(mqtt_topic, "topic/%s",client_id_buf);
	return mqtt_topic;
}

static int publish_devicedata(struct mqtt_client *client, enum mqtt_qos qos)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (u8_t *)get_mqtt_topic();
	param.message.topic.topic.size =
		strlen(param.message.topic.topic.utf8);
    printk("topic: %s \n",param.message.topic.topic.utf8);
	param.message.payload.data = get_mqtt_payload_devicedata(qos);
	param.message.payload.len =
		strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

static int publish_bme680(struct mqtt_client *client, enum mqtt_qos qos)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (u8_t *)get_mqtt_topic();
	param.message.topic.topic.size =
		strlen(param.message.topic.topic.utf8);
	param.message.payload.data = get_mqtt_payload_bme680(qos);
	param.message.payload.len =
		strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

static int publish_icm42605(struct mqtt_client *client, enum mqtt_qos qos)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (u8_t *)get_mqtt_topic();
	param.message.topic.topic.size =
			strlen(param.message.topic.topic.utf8);
	param.message.payload.data = get_mqtt_payload_icm42605(qos);
	param.message.payload.len =
			strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}



#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
	printk("[%s:%d] %s: %d <%s>\n", __func__, __LINE__, \
	       (func), rc, RC_STR(rc))
#define SUCCESS_OR_BREAK(rc) { if (rc != 0) { break; } }

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
	s64_t remaining = timeout;
	s64_t start_time = k_uptime_get();
	int rc;

	while (remaining > 0&&connected ) {
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


/////////////////////////////GPIO START///////////////////////////////////////

void pwr_Key_callback(struct device *port,
		   struct gpio_callback *cb, u32_t pins)
{

    u32_t pwr_key;
    u32_t end_time;
    int32_t key_time;

    gpio_pin_read(ggpio_dev,30,&pwr_key);
    if(pwr_key==0)
    {
		printk("power_key %d key down\n", 30);
        //printf("time is : %u \n",k_uptime_get_32());
        gstart_time=k_uptime_get_32();
        printf("gstart_time is : %u \n",gstart_time);
    }
	else
    {
        printk("power_key %d key up\n", 30);
        //printf("time is : %u \n",k_uptime_get_32());
        end_time=k_uptime_get_32();
        printf("end_time is : %u \n",end_time);
        if (end_time > gstart_time)
        {
            key_time=end_time-gstart_time;
        }
        else
        {
             key_time=end_time+((int32_t)gstart_time);
        }
        printf("key_time is : %u \n",key_time);
         
        if (key_time > KEY_POWER_OFF_TIME)  gpio_pin_write(ggpio_dev, IO_POWER_ON, 0);	  //POWET_OFF
    }
}

void chrq_callback(struct device *port,
		   struct gpio_callback *cb, u32_t pins)
{
    u32_t chrq;
	printk("chrq_Pin %d triggered\n", 26);
    gpio_pin_read(ggpio_dev,IO_NCHRQ,&chrq);
    gpio_pin_write(ggpio_dev, LED_RED, chrq);  //if chrq=0 ,turn LED_RED on
    gpio_pin_write(ggpio_dev, LED_GREEN, (chrq+1)%2); //LED_GREEN =! LED_RED
 }

static struct gpio_callback pwr_key_gpio_cb;
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


    gpio_pin_configure(ggpio_dev, POWER_KEY,
				 (GPIO_DIR_IN | GPIO_INT |
				  GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE |
				  GPIO_INT_DEBOUNCE));

    gpio_init_callback(&pwr_key_gpio_cb, pwr_Key_callback, BIT(POWER_KEY));
    gpio_add_callback(ggpio_dev, &pwr_key_gpio_cb);
    gpio_pin_enable_callback(ggpio_dev, POWER_KEY);


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


///////////////////////////////GPS START///////////////////////////////////////////


#define AT_MAGPIO      "AT\%XMAGPIO=1,0,0,1,1,1574,1577"


static char           nmea_strings[10][NRF_GNSS_NMEA_MAX_LEN];
static u32_t          nmea_string_cnt;

static bool           got_first_fix;
static bool           update_terminal;
static u64_t          fix_timestamp;
nrf_gnss_data_frame_t last_fix;
static const char     update_indicator[] = {'\\', '|', '/', '-'};


static int gps_magpio_init(void)
{

	enum at_cmd_state at_state;
	char msg_buf[40];

	int err = at_cmd_write(AT_MAGPIO, msg_buf,40, &at_state);
	if (err) {
		printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
	}

	//snprintf(id_buf, CLIENT_ID_LEN + 1, "nrf-%s", imei_buf);

	return 0;
}

static int init_gps(void)
{
	u16_t fix_retry     = 0;
	u16_t fix_interval  = 1;
	u16_t nmea_mask     = NRF_CONFIG_NMEA_GSV_MASK |
			      NRF_CONFIG_NMEA_GSA_MASK |
			      NRF_CONFIG_NMEA_GLL_MASK |
			      NRF_CONFIG_NMEA_GGA_MASK |
			      NRF_CONFIG_NMEA_RMC_MASK;
	int   retval;
    gps_magpio_init();
	
	gps_fd = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_DGRAM, NRF_PROTO_GNSS);

	if (gps_fd >= 0) {
		printk("Socket created\n");
	} else {
		printk("Could not init socket (err: %d)\n", gps_fd);
		return -1;
	}

	retval = nrf_setsockopt(gps_fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_RETRY,
				&fix_retry,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set fix retry value\n");
		return -1;
	}

	retval = nrf_setsockopt(gps_fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_FIX_INTERVAL,
				&fix_interval,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set fix interval value\n");
		return -1;
	}

	retval = nrf_setsockopt(gps_fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_NMEA_MASK,
				&nmea_mask,
				sizeof(uint16_t));

	if (retval != 0) {
		printk("Failed to set nmea mask\n");
		return -1;
	}

	retval = nrf_setsockopt(gps_fd,
				NRF_SOL_GNSS,
				NRF_SO_GNSS_START,
				NULL,
				0);

	if (retval != 0) {
		printk("Failed to start GPS\n");
		return -1;
	}

	return 0;
}

static void print_satellite_stats(nrf_gnss_data_frame_t *pvt_data)
{
	u8_t  tracked          = 0;
	u8_t  in_fix           = 0;
	u8_t  unhealthy        = 0;

	for (int i = 0; i < NRF_GNSS_MAX_SATELLITES; ++i) {

		if ((pvt_data->pvt.sv[i].sv > 0) &&
		    (pvt_data->pvt.sv[i].sv < 33)) {

			tracked++;

			if (pvt_data->pvt.sv[i].flags &
					NRF_GNSS_PVT_FLAG_FIX_VALID_BIT) {
				in_fix++;
			}

			if (pvt_data->pvt.sv[i].flags &
					NRF_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printk("Tracking: %d Using: %d Unhealthy: %d", tracked,
						       in_fix,
						       unhealthy);

	printk("\nSeconds since last fix %lld\n",
			(k_uptime_get() - fix_timestamp) / 1000);
}

static void print_pvt_data(nrf_gnss_data_frame_t *pvt_data)
{
	printf("Longitude:  %f\n", pvt_data->pvt.longitude);
	printf("Latitude:   %f\n", pvt_data->pvt.latitude);
	printf("Altitude:   %f\n", pvt_data->pvt.altitude);
	printf("Speed:      %f\n", pvt_data->pvt.speed);
	printf("Heading:    %f\n", pvt_data->pvt.heading);
	printk("Date:       %02u-%02u-%02u\n", pvt_data->pvt.datetime.day,
					       pvt_data->pvt.datetime.month,
					       pvt_data->pvt.datetime.year);
	printk("Time (UTC): %02u:%02u:%02u\n", pvt_data->pvt.datetime.hour,
					       pvt_data->pvt.datetime.minute,
					      pvt_data->pvt.datetime.seconds);
}

static void print_nmea_data(void)
{
	printk("NMEA strings:\n");

	for (int i = 0; i < nmea_string_cnt; ++i) {
		printk("%s\n", nmea_strings[i]);
	}
}

int process_gps_data(nrf_gnss_data_frame_t *gps_data)
{
	int retval;

	retval = nrf_recv(gps_fd, gps_data, sizeof(nrf_gnss_data_frame_t), NRF_MSG_DONTWAIT);

	if (retval > 0) {

		switch (gps_data->data_id) {
		case NRF_GNSS_PVT_DATA_ID:

			if ((gps_data->pvt.flags &
				NRF_GNSS_PVT_FLAG_FIX_VALID_BIT)
				== NRF_GNSS_PVT_FLAG_FIX_VALID_BIT) {

				if (!got_first_fix) {
					got_first_fix = true;
				}

				fix_timestamp = k_uptime_get();
				memcpy(&last_fix, gps_data, sizeof(nrf_gnss_data_frame_t));

				nmea_string_cnt = 0;
				update_terminal = true;
			}
			break;

		case NRF_GNSS_NMEA_DATA_ID:
			if (nmea_string_cnt < 10) {
				memcpy(nmea_strings[nmea_string_cnt++],
				       gps_data->nmea,
				       retval);
			}
			break;

		default:
			break;
		}
	}

	return retval;
}

/////////////////////////////GPS END///////////////////////////////////////////

void main(void)
{
	int err;
    int rc;
       
	/* The mqtt client struct */
	struct mqtt_client client;
        
    ////////Turn on power by drive POWER_ON=1   qiuhm /////////////
    Iotex_Gpio_Init();
   
    /* Iotex Init I2C */
    Iotex_I2C_Init();
    /* Iotex Init BME680 */
    Iotex_bme680_init();
    Iotex_bme680_config();
    /* Iotex Init ICM42605 */
    Iotex_icm42605_Init();
    Iotex_icm42605_Configure((uint8_t )IS_LOW_NOISE_MODE,
	                                  ICM426XX_ACCEL_CONFIG0_FS_SEL_4g,
	                                  ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
	                                  ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ,
	                                  ICM426XX_GYRO_CONFIG0_ODR_1_KHZ,
	                        (uint8_t )TMST_CLKIN_32K);
     
	printk("The MQTT AWS Publisher Sample\n");
        adc_init();
	printk("adc_int done\n");
#if !defined(CONFIG_USE_PROVISIONED_CERTIFICATES)
	provision_certificates();
#endif /* CONFIG_USE_PROVISIONED_CERTIFICATES  */
	modem_configure();
	printk("modem_configure\n");

	err=client_init(&client, CONFIG_MQTT_BROKER_HOSTNAME);

/*
	err = aws_fota_init(&client, CONFIG_APP_VERSION, aws_fota_cb_handler);
	if (err != 0) {
		printk("ERROR: aws_fota_init %d\n", err);
		return;
	}

*/
	err = mqtt_connect(&client);
	if (err != 0) {
		printk("ERROR: mqtt_connect %d\n", err);
		return;
	}

	err = fds_init(&client);
	if (err != 0) {
		printk("ERROR: fds_init %d\n", err);
		return;
	}

	/* All initializations were successful mark image as working so that we
	 * will not revert upon reboot. */
	boot_write_img_confirmed();

	while (1) {
		err = poll(&fds, 1, K_SECONDS(CONFIG_MQTT_KEEPALIVE));
		if (err < 0) {
			printk("ERROR: poll %d\n", errno);
			break;
		}

		err = mqtt_live(&client);
		if (err != 0) {
			printk("ERROR: mqtt_live %d\n", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN) {
			err = mqtt_input(&client);
			if (err != 0) {
				printk("ERROR: mqtt_input %d\n", err);
				break;
			}
		}

		if ((fds.revents & POLLERR) == POLLERR) {
			printk("POLLERR\n");
			break;
		}

		if ((fds.revents & POLLNVAL) == POLLNVAL) {
			printk("POLLNVAL\n");
			break;
		}

		if (do_reboot) {
			/* Teardown */
			mqtt_disconnect(&client);
			sys_reboot(0);
		}

               while(1)
                {
                     if(connected)
                     {
                      user_delay_ms(10000); 
                      rc = mqtt_ping(&client);
                     PRINT_RESULT("mqtt_ping", rc);
                      SUCCESS_OR_BREAK(rc);
                     gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
                      rc = process_mqtt_and_sleep(&client, 100);
                      SUCCESS_OR_BREAK(rc);
                       rc = publish_devicedata(&client, 0);
                      PRINT_RESULT("mqtt_publish_devicedata", rc);


                     rc = process_mqtt_and_sleep(&client, 100);
                     SUCCESS_OR_BREAK(rc);
                     rc = publish_bme680(&client, 0);
                      PRINT_RESULT("mqtt_publish_bme680", rc);
                   //   gpio_pin_write(ggpio_dev, LED_BLUE, 0);	//p0.00 == LED_BLUE ON
                      SUCCESS_OR_BREAK(rc);

                      rc = process_mqtt_and_sleep(&client, 100);
                      SUCCESS_OR_BREAK(rc);
                   //  gpio_pin_write(ggpio_dev, LED_BLUE, 1);	//p0.00 == LED_BLUE OFF
                  
                      rc = publish_icm42605(&client, 0);
                      PRINT_RESULT("mqtt_publish_icm42605", rc);
                      gpio_pin_write(ggpio_dev, LED_BLUE, 0);	//p0.00 == LED_BLUE ON
                  
                      SUCCESS_OR_BREAK(rc);
                    }
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
