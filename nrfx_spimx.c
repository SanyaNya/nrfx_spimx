#include "nrfx_spimx.h"

#if NRFX_CHECK(NRFX_SPIMX_ENABLED)

#if NRFX_CHECK(NRFX_SPIM_ENABLED) || defined(NRFX_SPIM_H__)
#error "nrfx_spim driver must be disabled to use nrfx_spimx"
#endif

#if !(NRFX_CHECK(NRFX_SPIMX0_ENABLED) || NRFX_CHECK(NRFX_SPIMX1_ENABLED) || \
      NRFX_CHECK(NRFX_SPIMX2_ENABLED) || NRFX_CHECK(NRFX_SPIMX3_ENABLED))
#error "No enabled SPIMX instances. Check <nrfx_config.h>."
#endif

#include <nrfx_spimx.h>
#include <hal/nrf_gpio.h>

#define NRFX_LOG_MODULE SPIMX
#include <nrfx_log.h>

#if NRFX_CHECK(NRFX_SPIMX0_ENABLED)
#define SPIMX_LENGTH_VALIDATE0(drv_inst_idx, rx_len, tx_len) \
  ((drv_inst_idx) == NRFX_SPIMX0_INST_IDX && NRFX_EASYDMA_LENGTH_VALIDATE(SPIM0, rx_len, tx_len))
#else
  #define SPIMX_LENGTH_VALIDATE0(drv_inst_idx, rx_len, tx_len) 0
#endif

#if NRFX_CHECK(NRFX_SPIMX1_ENABLED)
#define SPIMX_LENGTH_VALIDATE1(drv_inst_idx, rx_len, tx_len) \
  ((drv_inst_idx) == NRFX_SPIMX1_INST_IDX && NRFX_EASYDMA_LENGTH_VALIDATE(SPIM1, rx_len, tx_len))
#else
  #define SPIMX_LENGTH_VALIDATE1(drv_inst_idx, rx_len, tx_len) 0
#endif

#if NRFX_CHECK(NRFX_SPIMX2_ENABLED)
#define SPIMX_LENGTH_VALIDATE2(drv_inst_idx, rx_len, tx_len) \
  ((drv_inst_idx) == NRFX_SPIMX2_INST_IDX && NRFX_EASYDMA_LENGTH_VALIDATE(SPIM2, rx_len, tx_len))
#else
  #define SPIMX_LENGTH_VALIDATE2(drv_inst_idx, rx_len, tx_len) 0
#endif

#if NRFX_CHECK(NRFX_SPIMX3_ENABLED)
#define SPIMX_LENGTH_VALIDATE3(drv_inst_idx, rx_len, tx_len) \
  ((drv_inst_idx) == NRFX_SPIMX3_INST_IDX && NRFX_EASYDMA_LENGTH_VALIDATE(SPIM3, rx_len, tx_len))
#else
  #define SPIMX_LENGTH_VALIDATE3(drv_inst_idx, rx_len, tx_len) 0
#endif

#define SPIMX_LENGTH_VALIDATE(drv_inst_idx, rx_len, tx_len)  \
  (SPIMX_LENGTH_VALIDATE0(drv_inst_idx, rx_len, tx_len) ||   \
   SPIMX_LENGTH_VALIDATE1(drv_inst_idx, rx_len, tx_len) ||   \
   SPIMX_LENGTH_VALIDATE2(drv_inst_idx, rx_len, tx_len) ||   \
   SPIMX_LENGTH_VALIDATE3(drv_inst_idx, rx_len, tx_len))

#define ASSERT_SPIM_INSTANCE(p_instance)                            \
do                                                                  \
{                                                                   \
  NRFX_ASSERT(p_instance);                                          \
  NRFX_ASSERT(p_instance->p_reg);                                   \
  NRFX_ASSERT(                                                      \
    p_instance->p_reg == NRF_SPIM0 ||                               \
    p_instance->p_reg == NRF_SPIM1 ||                               \
    p_instance->p_reg == NRF_SPIM2 ||                               \
    p_instance->p_reg == NRF_SPIM3);                                \
  NRFX_ASSERT(p_instance->drv_inst_idx < NRFX_SPIMX_ENABLED_COUNT); \
}while(0)

#define ASSERT_SPIM_CONFIG(p_conf)                                                                            \
do                                                                                                            \
{                                                                                                             \
  NRFX_ASSERT(p_conf);                                                                                        \
  NRFX_ASSERT(p_conf->sck_pin  != NRFX_SPIMX_PIN_NOT_USED);                                                   \
  NRFX_ASSERT(p_conf->ss_pin   != NRFX_SPIMX_PIN_NOT_USED);                                                   \
  NRFX_ASSERT(!(p_conf->mosi_pin == NRFX_SPIMX_PIN_NOT_USED && p_conf->miso_pin == NRFX_SPIMX_PIN_NOT_USED)); \
  NRFX_ASSERT(                                                                                                \
    p_conf->sck_pin != p_conf->mosi_pin  &&                                                                   \
    p_conf->sck_pin != p_conf->miso_pin  &&                                                                   \
    p_conf->sck_pin != p_conf->ss_pin    &&                                                                   \
    p_conf->mosi_pin != p_conf->miso_pin &&                                                                   \
    p_conf->mosi_pin != p_conf->ss_pin   &&                                                                   \
    p_conf->miso_pin != p_conf->ss_pin);                                                                      \
  NRFX_ASSERT(p_conf->irq_priority <= 7);                                                                     \
  NRFX_ASSERT(p_conf->mode <= NRF_SPIM_MODE_3);                                                               \
  NRFX_ASSERT(                                                                                                \
    p_conf->bit_order == NRF_SPIM_BIT_ORDER_MSB_FIRST ||                                                      \
    p_conf->bit_order == NRF_SPIM_BIT_ORDER_LSB_FIRST);                                                       \
}while(0)

#define ASSERT_XFER_DESC(p_xfer_desc)                                                                           \
do                                                                                                              \
{                                                                                                               \
  NRFX_ASSERT(p_xfer_desc);                                                                                     \
  NRFX_ASSERT(p_xfer_desc->p_tx_buffer != NULL || p_xfer_desc->tx_length == 0);                                 \
  NRFX_ASSERT(p_xfer_desc->p_rx_buffer != NULL || p_xfer_desc->rx_length == 0);                                 \
  NRFX_ASSERT(p_xfer_desc->p_tx_buffer == NULL || nrfx_is_in_ram(p_xfer_desc->p_tx_buffer));                    \
  NRFX_ASSERT(p_xfer_desc->p_rx_buffer == NULL || nrfx_is_in_ram(p_xfer_desc->p_rx_buffer));                    \
  NRFX_ASSERT(SPIMX_LENGTH_VALIDATE(p_instance->drv_inst_idx, p_xfer_desc->rx_length, p_xfer_desc->tx_length)); \
}while(0)

typedef enum
{
  SPI_TRANSFER_TYPE_NONE,
  SPI_TRANSFER_TYPE_WRITE,
  SPI_TRANSFER_TYPE_READ,
  SPI_TRANSFER_TYPE_WRITE_AND_READ
}
spi_transfer_type_t;

typedef enum 
{
  MOMI_STATE_OUTPUT,
  MOMI_STATE_INPUT
}
momi_state_t;

// Control block - driver instance local data.
typedef struct
{
  nrfx_spimx_evt_handler_t     handler;
  void*                        p_context;
  nrfx_spimx_evt_t             evt;
  nrfx_drv_state_t             state;
  bool                         is_3wire;
  volatile spi_transfer_type_t transfer_in_progress;

  volatile momi_state_t momi_state;
  uint8_t momi_pin;
  uint8_t ss_pin;
  bool    ss_active_high;
  uint8_t orc;
}
spimx_control_block_t;

__STATIC_INLINE void spim_set_sck(NRF_SPIM_Type* p_spim, uint32_t sck_pin)
{
  p_spim->PSEL.SCK = sck_pin; 
}
__STATIC_INLINE void spim_set_mosi(NRF_SPIM_Type* p_spim, uint32_t mosi_pin)
{
  p_spim->PSEL.MOSI = mosi_pin; 
}
__STATIC_INLINE void spim_set_miso(NRF_SPIM_Type* p_spim, uint32_t miso_pin)
{
  p_spim->PSEL.MISO = miso_pin; 
}

static spimx_control_block_t g_spimx_cb[NRFX_SPIMX_ENABLED_COUNT] = {};

__STATIC_INLINE void set_momi_cfg_out(uint8_t momi_pin)
{
  nrf_gpio_cfg(momi_pin,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRFX_SPIMX_PIN_DRIVE_CFG,
    NRF_GPIO_PIN_NOSENSE);
}

__STATIC_INLINE void set_momi_cfg_in(uint8_t momi_pin)
{
  nrf_gpio_cfg(momi_pin,
    NRF_GPIO_PIN_DIR_INPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRFX_SPIMX_MISO_PULL_CFG,
    NRFX_SPIMX_PIN_DRIVE_CFG,
    NRF_GPIO_PIN_NOSENSE);
}

__STATIC_INLINE void change_momi_state(NRF_SPIM_Type* p_spim,
                                       spimx_control_block_t* p_cb,
                                       momi_state_t momi_state)
{
  if(p_cb->momi_state != momi_state)
  {
    nrf_spim_disable(p_spim);

    if(momi_state == MOMI_STATE_OUTPUT)
    {
      set_momi_cfg_out(p_cb->momi_pin);
      spim_set_mosi(p_spim, p_cb->momi_pin);
      spim_set_miso(p_spim, NRF_SPIM_PIN_NOT_CONNECTED);
    }
    else
    {
      set_momi_cfg_in(p_cb->momi_pin);
      spim_set_mosi(p_spim, NRF_SPIM_PIN_NOT_CONNECTED);
      spim_set_miso(p_spim, p_cb->momi_pin);
    }

    p_cb->momi_state = momi_state;

    nrf_spim_enable(p_spim);
  }
}

__STATIC_INLINE void set_pin_non_active(uint8_t pin, bool active_high)
{
  if(active_high) nrf_gpio_pin_clear(pin);
  else nrf_gpio_pin_set(pin);
}

__STATIC_INLINE void set_pin_active(uint8_t pin, bool active_high)
{
  if(active_high) nrf_gpio_pin_set(pin);
  else nrf_gpio_pin_clear(pin);
}

__STATIC_INLINE void set_ss_active(const spimx_control_block_t* p_cb)
{
  set_pin_active(p_cb->ss_pin, p_cb->ss_active_high);
}

__STATIC_INLINE void set_ss_non_active(const spimx_control_block_t* p_cb)
{
  set_pin_non_active(p_cb->ss_pin, p_cb->ss_active_high);
}

__STATIC_INLINE spi_transfer_type_t transfer_type(const nrfx_spimx_xfer_desc_t* xfer_desc)
{
  if(xfer_desc->p_tx_buffer && xfer_desc->p_rx_buffer) return SPI_TRANSFER_TYPE_WRITE_AND_READ;
  if(xfer_desc->p_tx_buffer) return SPI_TRANSFER_TYPE_WRITE;
  if(xfer_desc->p_rx_buffer) return SPI_TRANSFER_TYPE_READ;
  
  return SPI_TRANSFER_TYPE_NONE;
}

__STATIC_INLINE uint32_t spim_pin(uint8_t pin_number)
{
  return pin_number != NRFX_SPIMX_PIN_NOT_USED ? pin_number : NRF_SPIM_PIN_NOT_CONNECTED;
}

void nrfx_spimx_init(const nrfx_spimx_t* const  p_instance,
                     const nrfx_spimx_config_t* p_conf,
                     nrfx_spimx_evt_handler_t   handler,
                     void*                      p_context)
{
  ASSERT_SPIM_INSTANCE(p_instance);
  ASSERT_SPIM_CONFIG(p_conf);
  
  spimx_control_block_t* p_cb = &g_spimx_cb[p_instance->drv_inst_idx];
  NRF_SPIM_Type* p_spim = p_instance->p_reg;

  NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_UNINITIALIZED);

  // Configure SCK pin
  // According to the reference manual guidelines this pin and its input
  // buffer must always be connected for the SPI to work.
  set_pin_non_active(p_conf->sck_pin, /*active_high*/ p_conf->mode == NRF_SPIM_MODE_0 || p_conf->mode == NRF_SPIM_MODE_1);
  nrf_gpio_cfg(p_conf->sck_pin,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_CONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRFX_SPIMX_PIN_DRIVE_CFG,
    NRF_GPIO_PIN_NOSENSE);
  
  // Configure MOSI pin
  if(p_conf->mosi_pin != NRFX_SPIMX_PIN_NOT_USED)
  {
    set_pin_non_active(p_conf->mosi_pin, /*active_high*/ true);
    nrf_gpio_cfg(p_conf->mosi_pin,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRFX_SPIMX_PIN_DRIVE_CFG,
      NRF_GPIO_PIN_NOSENSE);
  }

  // Configure MISO pin
  if(p_conf->miso_pin != NRFX_SPIMX_PIN_NOT_USED)
  {
    nrf_gpio_cfg(p_conf->miso_pin,
      NRF_GPIO_PIN_DIR_INPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRFX_SPIMX_MISO_PULL_CFG,
      NRFX_SPIMX_PIN_DRIVE_CFG,
      NRF_GPIO_PIN_NOSENSE);
  }
  
  // Configure SS pin
  set_pin_non_active(p_conf->ss_pin, p_conf->ss_active_high);
  nrf_gpio_cfg(p_conf->ss_pin,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRFX_SPIMX_PIN_DRIVE_CFG,
    NRF_GPIO_PIN_NOSENSE);

  // Set SPIM registers and enable
  nrf_spim_pins_set(p_spim, p_conf->sck_pin, spim_pin(p_conf->mosi_pin), spim_pin(p_conf->miso_pin));
  nrf_spim_frequency_set(p_spim, p_conf->frequency);
  nrf_spim_configure(p_spim, p_conf->mode, p_conf->bit_order);
  nrf_spim_orc_set(p_spim, p_conf->orc);

  nrf_spim_enable(p_spim);

  // Enable interrupts handler
  if(handler)
  {
    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_spim), p_conf->irq_priority);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_spim));
    nrf_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);
  }

  // Fill control block
  p_cb->handler = handler;
  p_cb->p_context = p_context;
  p_cb->state = NRFX_DRV_STATE_INITIALIZED;
  p_cb->is_3wire = p_conf->mosi_pin == NRFX_SPIMX_PIN_NOT_USED || p_conf->miso_pin == NRFX_SPIMX_PIN_NOT_USED;
  p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_NONE;
  p_cb->momi_state = p_conf->mosi_pin != NRFX_SPIMX_PIN_NOT_USED ? MOMI_STATE_OUTPUT : MOMI_STATE_INPUT;
  p_cb->momi_pin = p_conf->mosi_pin != NRFX_SPIMX_PIN_NOT_USED ? p_conf->mosi_pin : p_conf->miso_pin;
  p_cb->ss_pin = p_conf->ss_pin;
  p_cb->ss_active_high = p_conf->ss_active_high;
  p_cb->orc = p_conf->orc;
}

void nrfx_spimx_uninit(const nrfx_spimx_t* const p_instance)
{
  ASSERT_SPIM_INSTANCE(p_instance);

  spimx_control_block_t* p_cb = &g_spimx_cb[p_instance->drv_inst_idx];
  NRF_SPIM_Type* p_spim = p_instance->p_reg;

  // Abort running xfer
  if(p_cb->transfer_in_progress != SPI_TRANSFER_TYPE_NONE)
      nrfx_spimx_abort(p_instance);

  // Disable interrupts handler
  if(p_cb->handler)
  {
    NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_spim));
    nrf_spim_int_disable(p_spim, NRF_SPIM_ALL_INTS_MASK);
  }
  
  nrf_spim_disable(p_spim);
  
  p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
}

__STATIC_INLINE void spim_xfer(NRF_SPIM_Type* p_spim, const nrfx_spimx_xfer_desc_t* p_xfer_desc)
{
  // Set buffers
  nrf_spim_tx_buffer_set(p_spim, p_xfer_desc->p_tx_buffer, p_xfer_desc->tx_length);
  nrf_spim_rx_buffer_set(p_spim, p_xfer_desc->p_rx_buffer, p_xfer_desc->rx_length);

  nrf_spim_tx_list_disable(p_spim);
  nrf_spim_rx_list_disable(p_spim);

  // Start transfer
  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);
}

void nrfx_spimx_xfer(const nrfx_spimx_t* const p_instance, const nrfx_spimx_xfer_desc_t* p_xfer_desc)
{
  ASSERT_SPIM_INSTANCE(p_instance);
  ASSERT_XFER_DESC(p_xfer_desc);
  
  NRF_SPIM_Type* p_spim = p_instance->p_reg;
  spimx_control_block_t* p_cb = &g_spimx_cb[p_instance->drv_inst_idx];
  NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);
  NRFX_ASSERT(p_cb->transfer_in_progress == SPI_TRANSFER_TYPE_NONE);
  
  spi_transfer_type_t type = transfer_type(p_xfer_desc);

  p_cb->transfer_in_progress = type;
  p_cb->evt.xfer_desc = *p_xfer_desc;

  nrfx_spimx_xfer_desc_t internal_xfer_desc;

  // Configure line input/output state for 3-wire depending on transfer type
  if(p_cb->is_3wire)
  {
    if(type == SPI_TRANSFER_TYPE_WRITE || type == SPI_TRANSFER_TYPE_WRITE_AND_READ)
    {
      change_momi_state(p_spim, p_cb, MOMI_STATE_OUTPUT);
      internal_xfer_desc = (nrfx_spimx_xfer_desc_t)NRFX_SPIMX_XFER_TX(p_xfer_desc->p_tx_buffer, p_xfer_desc->tx_length);
    }
    else
    {
      change_momi_state(p_spim, p_cb, MOMI_STATE_INPUT);
      internal_xfer_desc = (nrfx_spimx_xfer_desc_t)NRFX_SPIMX_XFER_RX(p_xfer_desc->p_rx_buffer, p_xfer_desc->rx_length);
    }
  }
  else
    internal_xfer_desc = *p_xfer_desc;
  
  set_ss_active(p_cb);

  spim_xfer(p_spim, &internal_xfer_desc);

  if(!p_cb->handler)
  {
    // Wait for transfer end
    while(!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END));

    // Perform read xfer after write for 3-wire
    if(p_cb->is_3wire && type == SPI_TRANSFER_TYPE_WRITE_AND_READ)
    {
      p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_READ;
      change_momi_state(p_spim, p_cb, MOMI_STATE_INPUT);
      internal_xfer_desc = (nrfx_spimx_xfer_desc_t)NRFX_SPIMX_XFER_RX(p_xfer_desc->p_rx_buffer, p_xfer_desc->rx_length);
      spim_xfer(p_spim, &internal_xfer_desc);

      while(!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END));
    }

    // Finish xfer
    set_ss_non_active(p_cb);
    p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_NONE;
  }
}

void nrfx_spimx_abort(const nrfx_spimx_t* p_instance)
{
  ASSERT_SPIM_INSTANCE(p_instance);

  spimx_control_block_t* p_cb = &g_spimx_cb[p_instance->drv_inst_idx];
  NRF_SPIM_Type* p_spim = p_instance->p_reg;
  
  NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);
  NRFX_ASSERT(p_cb->transfer_in_progress != SPI_TRANSFER_TYPE_NONE);

  // Stop operation
  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_STOP);
  while(!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_STOPPED));

  p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_NONE;
}

static void irq_handler(NRF_SPIM_Type* p_spim, spimx_control_block_t* p_cb)
{
  NRFX_ASSERT(p_cb->handler);

  if(nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END))
  {
    nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);

    // Perform read xfer after write for 3-wire
    if(p_cb->is_3wire && p_cb->transfer_in_progress == SPI_TRANSFER_TYPE_WRITE_AND_READ)
    {
      p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_READ;
      change_momi_state(p_spim, p_cb, MOMI_STATE_INPUT);
      nrfx_spimx_xfer_desc_t read_xfer_desc =
        NRFX_SPIMX_XFER_RX(p_cb->evt.xfer_desc.p_rx_buffer, p_cb->evt.xfer_desc.rx_length);
      spim_xfer(p_spim, &read_xfer_desc);
    }
    // Finish xfer
    else
    {
      set_ss_non_active(p_cb);

      p_cb->transfer_in_progress = SPI_TRANSFER_TYPE_NONE;
      p_cb->evt.type = NRFX_SPIMX_EVENT_DONE;
      p_cb->handler(&p_cb->evt, p_cb->p_context);
    }
  }
}

// nrfx_spim_N_irq_handler called by sdk on event if prs not enabled for this N

#if NRFX_CHECK(NRFX_SPIMX0_ENABLED)
void nrfx_spim_0_irq_handler()
{
  irq_handler(NRF_SPIM0, &g_spimx_cb[NRFX_SPIMX0_INST_IDX]);
}
#endif
#if NRFX_CHECK(NRFX_SPIMX1_ENABLED)
void nrfx_spim_1_irq_handler()
{
  irq_handler(NRF_SPIM1, &g_spimx_cb[NRFX_SPIMX1_INST_IDX]);
}
#endif
#if NRFX_CHECK(NRFX_SPIMX2_ENABLED)
void nrfx_spim_2_irq_handler()
{
  irq_handler(NRF_SPIM2, &g_spimx_cb[NRFX_SPIMX2_INST_IDX]);
}
#endif
#if NRFX_CHECK(NRFX_SPIMX3_ENABLED)
void nrfx_spim_3_irq_handler()
{
  irq_handler(NRF_SPIM3, &g_spimx_cb[NRFX_SPIMX3_INST_IDX]);
}
#endif

#endif // NRFX_SPIMX_ENABLED
