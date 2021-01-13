/*
 * Copyright (c) Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */


/**
 * @file sdc.h
 *
 * @defgroup sdc SoftDevice Controller API
 *
 * The main APIs needed to configure, enable, and use the SoftDevice Controller.
 * @{
 */


#ifndef SDC_H__
#define SDC_H__


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "nrf.h"
#include "nrf_errno.h"


/** @brief Default resource configuration tag. */
#define SDC_DEFAULT_RESOURCE_CFG_TAG  0

/** @brief Default maximum number of concurrent slave links. */
#define SDC_DEFAULT_SLAVE_COUNT   1

/** @brief Default maximum number of concurrent master links. */
#define SDC_DEFAULT_MASTER_COUNT  1

/** @brief Default maximum Link Layer TX packet size. */
#define SDC_DEFAULT_TX_PACKET_SIZE 27

/** @brief Default maximum Link Layer RX packet size. */
#define SDC_DEFAULT_RX_PACKET_SIZE 27

/** @brief Default maximum Link Layer TX packet count per connection.
 *
 * With the default count, the application is able to refill the buffers during a connection event.
 */
#define SDC_DEFAULT_TX_PACKET_COUNT 3

/** @brief Default maximum Link Layer RX packet count per connection.
 *
 * With the default count, the application is able to empty the buffers during a connection event.
 */
#define SDC_DEFAULT_RX_PACKET_COUNT 3

/** @brief Default connection event length. */
#define SDC_DEFAULT_EVENT_LENGTH_US 7500UL

/** @brief Size of build revision array in bytes. */
#define SDC_BUILD_REVISION_SIZE 20

/**
 * @defgroup sdc_mem_defines Memory requirement defines
 *
 * The SoftDevice Controller memory requirement defines may be used to determine the dynamic memory usage
 * at compile time. The defines specify an upper limit, therefore the actual memory required
 * may be less.
 *
 * @note The values of the memory requirement defines may change between minor releases.
 * @{
 */

/** @brief Maximum number of bytes required per master link for the default buffer configuration. */
#define SDC_MEM_DEFAULT_MASTER_LINK_SIZE 788

/** @brief Maximum number of bytes required per slave link for the default buffer configuration. */
#define SDC_MEM_DEFAULT_SLAVE_LINK_SIZE 940

/** @brief Memory overhead per LL packet buffer. */
#define SDC_MEM_BUFFER_OVERHEAD_SIZE 10

/** @brief Maximum additional number of bytes required per link.
 *
 * This macro will return the additional memory required per link
 * if non-default buffer sizes are used.
 *
 * @param[in] tx_size Link Layer TX packet size.
 * @param[in] rx_size Link Layer RX packet size.
 * @param[in] tx_count Link Layer TX packet count.
 * @param[in] rx_count Link Layer RX packet count.
 */
#define SDC_MEM_ADDITIONAL_LINK_SIZE(tx_size, rx_size, tx_count, rx_count) \
    ((tx_count) * (tx_size - SDC_DEFAULT_TX_PACKET_SIZE) + \
     (rx_count) * (rx_size - SDC_DEFAULT_RX_PACKET_SIZE) + \
     (tx_count - SDC_DEFAULT_TX_PACKET_COUNT) * \
        (SDC_MEM_BUFFER_OVERHEAD_SIZE + SDC_DEFAULT_TX_PACKET_SIZE) + \
     (rx_count - SDC_DEFAULT_RX_PACKET_COUNT) * \
        (SDC_MEM_BUFFER_OVERHEAD_SIZE + SDC_DEFAULT_RX_PACKET_SIZE))

/** @brief Maximum memory required per master link.
 *
 * @param[in] tx_size Link Layer TX packet size.
 * @param[in] rx_size Link Layer RX packet size.
 * @param[in] tx_count Link Layer TX packet count.
 * @param[in] rx_count Link Layer RX packet count.
 */
#define SDC_MEM_PER_MASTER_LINK(tx_size, rx_size, tx_count, rx_count) \
    (SDC_MEM_DEFAULT_MASTER_LINK_SIZE + \
     SDC_MEM_ADDITIONAL_LINK_SIZE(tx_size, rx_size, tx_count, rx_count))

/** @brief Maximum memory required per slave link.
 *
 * @param[in] tx_size Link Layer TX packet size.
 * @param[in] rx_size Link Layer RX packet size.
 * @param[in] tx_count Link Layer TX packet count.
 * @param[in] rx_count Link Layer RX packet count.
 */
#define SDC_MEM_PER_SLAVE_LINK(tx_size, rx_size, tx_count, rx_count) \
    (SDC_MEM_DEFAULT_SLAVE_LINK_SIZE + \
     SDC_MEM_ADDITIONAL_LINK_SIZE(tx_size, rx_size, tx_count, rx_count))

/** Maximum shared memory required for master links. */
#define SDC_MEM_MASTER_LINKS_SHARED 24

/** Maximum shared memory required for slave links. */
#define SDC_MEM_SLAVE_LINKS_SHARED  24

/** @} end of sdc_mem_defines */

/** @brief Function prototype for the fault handler.
 *
 * @note The SoftDevice Controller will disable all interrupts prior to calling the
 *       fault handler. The SoftDevice Controller will reset the chip if the
 *       application returns from this function.
 *
 * @param[in] file  The filename where the assertion occurred.
 * @param[in] line  The line number where the assertion occurred.
 */
typedef void (*sdc_fault_handler_t)(const char * file, const uint32_t line);


/** @brief Function prototype for the SoftDevice Controller callback.
 *
 *  @sa @ref sdc_enable().
 */
typedef void (*sdc_callback_t)(void);


enum sdc_cfg_type
{
    SDC_CFG_TYPE_NONE         = 0,  /**< No configuration update. */
    SDC_CFG_TYPE_MASTER_COUNT = 1,  /**< Number of concurrent master roles.
                                         @sa sdc_cfg_t::master_count. */
    SDC_CFG_TYPE_SLAVE_COUNT  = 2,  /**< Number of concurrent slave roles.
                                         @sa sdc_cfg_t::slave_count. */
    SDC_CFG_TYPE_BUFFER_CFG   = 3,  /**< Buffer configuration per connection.
                                         @sa sdc_cfg_t::buffer_cfg. */
    SDC_CFG_TYPE_EVENT_LENGTH = 4,  /**< Maximum event length.
                                         @sa sdc_cfg_t::event_length. */
};

/** @brief Role count. */
typedef struct
{
    uint8_t count;   /**< Max number of concurrent roles. */
} sdc_cfg_role_count_t;


/** @brief Buffer configuration. */
typedef struct
{
    uint8_t tx_packet_size;   /**< Link Layer TX packet size. Valid range: 27-251. */
    uint8_t rx_packet_size;   /**< Link Layer RX packet size. Valid range: 27-251. */
    uint8_t tx_packet_count;  /**< Link Layer TX packet count per link. */
    uint8_t rx_packet_count;  /**< Link Layer RX packet count per link. */
} sdc_cfg_buffer_cfg_t;


/** @brief Connection event length configuration. */
typedef struct
{
    uint32_t event_length_us; /**< Maximum connection event length */
} sdc_cfg_event_length_t;


/** @brief SoftDevice Controller configuration.  */
typedef union
{
    sdc_cfg_role_count_t   master_count;  /**< Max number of concurrent master connections.
                                               Default: @ref SDC_DEFAULT_MASTER_COUNT. */
    sdc_cfg_role_count_t   slave_count;   /**< Max number of concurrent slave connections.
                                               Default: @ref SDC_DEFAULT_SLAVE_COUNT. */
    sdc_cfg_buffer_cfg_t   buffer_cfg;    /**< Max number of concurrent slave connections.
                                               Default: @ref SDC_DEFAULT_SLAVE_COUNT. */
    sdc_cfg_event_length_t event_length;  /**< Max connection event length.
                                               Default: @ref SDC_DEFAULT_EVENT_LENGTH_US. */
} sdc_cfg_t;


/** @brief Initialize the SoftDevice Controller
 *
 * After this function is called, the application may use SoC APIs.
 *
 * @param[in]  fault_handler  The fault handler will be executed when there is an
 *                            internal error in the SoftDevice Controller.
 *
 * @retval 0            Success
 * @retval -NRF_EINVAL  Invalid argument provided
 * @retval -NRF_EPERM   Unable to initialize because
 *                        - MPSL is not initialized
 *                        - MPSL needs to be configured with a LFCLK accuracy
 *                          of 500 ppm or better.
 */
int32_t sdc_init(sdc_fault_handler_t fault_handler);


/** @brief Change or add a SoftDevice Controller configuration
 *
 * To change the default configuration, update @ref
 * SDC_DEFAULT_RESOURCE_CFG_TAG. To create or update a new
 * configuration, provide another resource_cfg_tag.
 *
 * @note The application can set config_type to @ref
 *       SDC_CFG_TYPE_NONE to obtain the required memory size for the
 *       current configuration in bytes.
 *
 * @note Resource configuration can only be performed prior to calling @ref
 *       sdc_enable(). However, the current configuration may be
 *       changed after enabling the SoftDevice Controller.
 *
 * @param[in]  config_tag       Configuration tag.
 * @param[in]  config_type      Configuration type. @sa sdc_cfg_type.
 * @param[in]  p_resource_cfg   Configuration to be changed.
 *
 * @returns Required memory size for the current configuration in bytes.
 * @retval -NRF_EOPNOTSUPP    Unsupported configuration
 * @retval -NRF_EINVAL        Invalid argument provided
 */
int32_t sdc_cfg_set(uint8_t config_tag,
                    uint8_t config_type,
                    sdc_cfg_t const * p_resource_cfg);


/** @brief Enable the SoftDevice Controller
 *
 * After this function is called, the application may utilize HCI APIs.
 *
 * @param[in] callback  The callback will be executed when HCI data or and HCI
 *                      event is available. The callback will be executed in
 *                      the same context as mpsl_low_priority_process.
 *                      @sa @ref sdc_hci_evt_get() and @ref sdc_hci_data_get().
 * @param[in]  p_mem    Provide memory for the current resource configuration. If
 *                      custom resource configurations are used, use the value
 *                      returned from @ref sdc_cfg_set().
 *
 * @retval 0            Success
 * @retval -NRF_EINVAL  Invalid argument provided
 */
int32_t sdc_enable(sdc_callback_t callback,
                   uint8_t * p_mem);


/** @brief Disable the SoftDevice Controller
 *
 * This call is synchronous. After the SoftDevice Controller is disabled, Bluetooth LE
 * functionality is no longer available.
 *
 * @retval 0 Success
 */
int32_t sdc_disable(void);


/** @brief Obtain build revision string
 *
 * The application must provide a buffer that is at least @ref SDC_BUILD_REVISION_SIZE
 * bytes long. The SoftDevice Controller will copy the build revision string to the provided buffer.
 *
 *  @param[in,out] p_build_revision  Build revision.
 *
 * @retval 0            Success
 * @retval -NRF_EINVAL  Invalid argument provided
 */
int32_t sdc_build_revision_get(uint8_t * p_build_revision);


/** @brief SoftDevice Controller RNG interrupt handler
 *
 * @note This function should be called when a RNG interrupt occurs. The
 *       interrupt priority level should be lower than priority level 0, that
 *       is, a higher numerical priority value.
 */
void sdc_RNG_IRQHandler(void);

/** @brief Support Advertising State
 *
 * After this API is called, the controller will support the HCI commands
 * and events related to the Advertising State.
 * Only non-connectable advertising is supported. To support connectable
 * advertising, call @ref sdc_support_slave().
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  Advertising state is not supported.
 */
int32_t sdc_support_adv(void);

/** @brief Support Slave role
 *
 * After this API is called, the controller will support the HCI commands
 * and events related to the slave role.
 * This state requires the advertising state to be supported.
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  Slave role is not supported.
 */
int32_t sdc_support_slave(void);

/** @brief Support Scanning state
 *
 * After this API is called, the controller will support the HCI commands
 * and events related to the scanning state.
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  Scanning state is not supported.
 */
int32_t sdc_support_scan(void);

/** @brief Support Master role
 *
 * After this API is called, the controller will support the HCI commands
 * and events related to the master role.
 * This state requires the scanning state to be supported.
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  Master role is not supported.
 */
int32_t sdc_support_master(void);

/** @brief Support Data Length Extensions
 *
 * After this API is called, the controller will support data length extension.
 * That is:
 *  - All DLE HCI APIs are supported. The controller replies with LL_LENGTH_RSP
 *  - when a LL_LENGTH_REQ is received. DLE is marked supported in the LL
 *  - Feature Exchange procedure.
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  Data Length Extension is not supported.
 */
int32_t sdc_support_dle(void);

/** @brief Support LE 2M PHY
 *
 * After this API is called, the controller will support LE 2M PHY. That is:
 *  - All HCI APIs for obtaining or changing PHYs are supported.
 *  - The controller can use 2M PHY in both the connected and non-connected state.
 *  - LE 2M PHY is marked supported in the LL Feature Exchange procedure.
 *
 * @retval 0           Success
 * @retval -NRF_EPERM  This API must be called before @ref sdc_enable().
 */
int32_t sdc_support_le_2m_phy(void);

/** @brief Support LE Coded PHY
 *
 * After this API is called, the controller will support LE Coded PHY. That is:
 *  - All HCI APIs for obtaining or changing PHYs are supported.
 *  - The controller can use LE Coded PHY in both the connected and non-connected state.
 *  - LE Coded PHY is marked supported in the LL Feature Exchange procedure.
 *
 * @retval 0                Success
 * @retval -NRF_EPERM       This API must be called before @ref sdc_enable().
 * @retval -NRF_EOPNOTSUPP  LE Coded PHY is not supported.
 */
int32_t sdc_support_le_coded_phy(void);

#ifdef __cplusplus
}
#endif

/** @} end of sdc */

#endif /* SDC_H__ */
