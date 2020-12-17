/*
 * 
 * IOTEX Power Manager
 * 
 */

#include <assert.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <nrf9160.h>
#include <stdio.h>
#include <string.h>
#include <drivers/adc.h>
#include <nrfx.h>
#include <nrfx_saadc.h>



#define SAADC_DEFAULT_CHANNEL_CONFIG    \
{                                                   \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
    .gain       = NRF_SAADC_GAIN1_6,                \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
    .acq_time   = NRF_SAADC_ACQTIME_10US,           \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
    .burst      = NRF_SAADC_BURST_DISABLED          \
}


#define SAADC_DEFAULT_CHANNEL(PIN_P) \
{                                      \
    .channel_config =   ((nrf_saadc_channel_config_t)SAADC_DEFAULT_CHANNEL_CONFIG),  \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
    .pin_n      = NRF_SAADC_INPUT_DISABLED,          \
    .channel_index = 0                                  \
}

nrfx_saadc_channel_t  saadc_channel_config_1 = SAADC_DEFAULT_CHANNEL(NRF_SAADC_INPUT_AIN0);


#define BUFFER_SIZE 3
static nrf_saadc_value_t m_sample_buffer[BUFFER_SIZE];


extern void PowerOfIndicator(void);

void adc_module_init(void)
{
    nrfx_err_t err_code;
    nrfx_saadc_init(7);
    if(err_code != NRFX_SUCCESS){
        printk("nrfx_saadc_init error : %d\n",err_code);
    }
    nrfx_saadc_channels_config(&saadc_channel_config_1,1);
    nrfx_saadc_simple_mode_set(1,NRF_SAADC_RESOLUTION_10BIT,NRF_SAADC_OVERSAMPLE_DISABLED,NULL);
    nrfx_saadc_buffer_set(m_sample_buffer,1);
}
int power_check(void)
{
    float adc_voltage = 0;
    nrfx_saadc_mode_trigger();   
    adc_voltage = m_sample_buffer[0]/1023.0 * 2 * 3600.0;
    if(adc_voltage < 3100.0)
    {
        PowerOfIndicator();
    }
    return 0;
}

