#ifndef NRFX_SPIMX_H__
#define NRFX_SPIMX_H__

#include <nrfx.h>
#include <hal/nrf_spim.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_spimx SPIM driver
 * @{
 * @ingroup nrf_spim
 * @brief   Serial Peripheral Master with EasyDMA driver with extra features.
 */

/** @brief Data structure of the SPIMX driver instance. */
typedef struct
{
  NRF_SPIM_Type* p_reg;        ///< Pointer to a structure with SPIM registers.
  uint8_t        drv_inst_idx; ///< Index of the driver instance.
}
nrfx_spimx_t;

/** @brief SPIMX driver instance indices. */
typedef enum
{
#if NRFX_CHECK(NRFX_SPIMX0_ENABLED)
  NRFX_SPIMX0_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_SPIMX1_ENABLED)
  NRFX_SPIMX1_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_SPIMX2_ENABLED)
  NRFX_SPIMX2_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_SPIMX3_ENABLED)
  NRFX_SPIMX3_INST_IDX,
#endif
  NRFX_SPIMX_ENABLED_COUNT
}
nrfx_spimx_inst_idx_t;

/** @brief Macro for creating an instance of the SPIMX driver. */
#define NRFX_SPIMX_INSTANCE(id)                             \
{                                                           \
  .p_reg = NRFX_CONCAT_2(NRF_SPIM, id),                     \
  .drv_inst_idx = NRFX_CONCAT_3(NRFX_SPIMX, id, _INST_IDX), \
}

/**
 * @brief This value can be provided instead of a pin number for MOSI/MISO
 *        to use spi 3wire(half-duplex) mode with default input/output state
 *        depending on which pin not used:
 *        if MOSI not used - INPUT
 *        if MISO not used - OUTPUT
 */
#define NRFX_SPIMX_PIN_NOT_USED  0xFF

/** @brief Configuration structure of the SPIMX driver instance. */
typedef struct
{
  uint8_t sck_pin;        ///< SCK pin number.
  uint8_t mosi_pin;       ///< MOSI pin number.
  uint8_t miso_pin;       ///< MISO pin number.
  uint8_t ss_pin;         ///< Slave Select pin number (optional).
                          /**< Set to @ref NRFX_SPIM_PIN_NOT_USED
                           *   if this signal is not needed. */
  bool    ss_active_high; ///< Polarity of the Slave Select pin during transmission.
  uint8_t irq_priority;   ///< Interrupt priority.
  uint8_t orc;            ///< Overrun character.
                          /**< This character is used when all bytes from the TX buffer are sent,
                               but the transfer continues due to RX. */

  nrf_spim_frequency_t frequency; ///< SPIMX frequency.
  nrf_spim_mode_t      mode;      ///< SPIMX mode.
  nrf_spim_bit_order_t bit_order; ///< SPIMX bit order.
}
nrfx_spimx_config_t;

/** @brief The default configuration of the SPIMX master instance. */
#define NRFX_SPIMX_DEFAULT_CONFIG                           \
{                                                           \
  .sck_pin        = NRFX_SPIMX_PIN_NOT_USED,                \
  .mosi_pin       = NRFX_SPIMX_PIN_NOT_USED,                \
  .miso_pin       = NRFX_SPIMX_PIN_NOT_USED,                \
  .ss_pin         = NRFX_SPIMX_PIN_NOT_USED,                \
  .ss_active_high = false,                                  \
  .irq_priority   = NRFX_SPIMX_DEFAULT_CONFIG_IRQ_PRIORITY, \
  .orc            = 0xFF,                                   \
  .frequency      = NRF_SPIM_FREQ_4M,                       \
  .mode           = NRF_SPIM_MODE_0,                        \
  .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,           \
}

/** @brief Single transfer descriptor structure. */
typedef struct
{
  const uint8_t* p_tx_buffer; ///< Pointer to TX buffer.
  size_t         tx_length;   ///< TX buffer length.
  uint8_t*       p_rx_buffer; ///< Pointer to RX buffer.
  size_t         rx_length;   ///< RX buffer length.
}
nrfx_spimx_xfer_desc_t;

/**
 * @brief Macro for setting up single transfer descriptor.
 *
 * This macro is for internal use only.
 */
#define NRFX_SPIMX_SINGLE_XFER(p_tx, tx_len, p_rx, rx_len) \
{                                                          \
  .p_tx_buffer = (uint8_t const *)(p_tx),                  \
  .tx_length = (tx_len),                                   \
  .p_rx_buffer = (p_rx),                                   \
  .rx_length = (rx_len),                                   \
}

/** @brief Macro for setting the duplex TX RX transfer. */
#define NRFX_SPIMX_XFER_TRX(p_tx_buf, tx_length, p_rx_buf, rx_length) NRFX_SPIMX_SINGLE_XFER(p_tx_buf, tx_length, p_rx_buf, rx_length)

/** @brief Macro for setting the TX transfer. */
#define NRFX_SPIMX_XFER_TX(p_buf, length) NRFX_SPIMX_SINGLE_XFER(p_buf, length, NULL, 0)

/** @brief Macro for setting the RX transfer. */
#define NRFX_SPIMX_XFER_RX(p_buf, length) NRFX_SPIMX_SINGLE_XFER(NULL, 0, p_buf, length)

/**
 * @brief SPIMX master driver event types, passed to the handler routine provided
 *        during initialization.
 */
typedef enum
{
  NRFX_SPIMX_EVENT_DONE, ///< Transfer done.
}
nrfx_spimx_evt_type_t;

/** @brief SPIMX event description with transmission details. */
typedef struct
{
  nrfx_spimx_evt_type_t  type;      ///< Event type.
  nrfx_spimx_xfer_desc_t xfer_desc; ///< Transfer details.
}
nrfx_spimx_evt_t;

/** @brief SPIMX driver event handler type. */
typedef void (*nrfx_spimx_evt_handler_t)(const nrfx_spimx_evt_t* p_event, void* p_context);

void nrfx_spimx_init(const nrfx_spimx_t* const  p_instance,
                     const nrfx_spimx_config_t* p_conf,
                     nrfx_spimx_evt_handler_t   handler,
                     void*                      p_context);

void nrfx_spimx_uninit(const nrfx_spimx_t* const p_instance);

void nrfx_spimx_xfer(const nrfx_spimx_t* const p_instance, const nrfx_spimx_xfer_desc_t* p_xfer_desc);

void nrfx_spimx_abort(const nrfx_spimx_t* p_instance);

void nrfx_spim_0_irq_handler();
void nrfx_spim_1_irq_handler();
void nrfx_spim_2_irq_handler();
void nrfx_spim_3_irq_handler();

#ifdef __cplusplus
}
#endif

#endif // NRFX_SPIMX_H__
