# STM32 Timer Driver Methods - First Version

## Core Initialization & Configuration

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_init()` | `timer_handle_t*`, `timer_config_t*` | Initialize timer with basic configuration (ARPE, DIR, CMS, OPM, URS, UDIS) | TIMx_CR1 |
| `timer_deinit()` | `timer_handle_t*` | Reset timer to default state and disable clock | TIMx_CR1, RCC |
| `timer_start()` | `timer_handle_t*` | Enable timer counter (CEN bit) | TIMx_CR1 |
| `timer_stop()` | `timer_handle_t*` | Disable timer counter (CEN bit) | TIMx_CR1 |

## Event Generation

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_generate_update_event()` | `timer_handle_t*` | Generate update event (UG bit) | TIMx_EGR |
| `timer_generate_cc_event()` | `timer_handle_t*`, `uint8_t channel` | Generate capture/compare event (CCxG bit) | TIMx_EGR |

## Output Compare Configuration

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_configure_output_compare()` | `timer_handle_t*`, `uint8_t channel`, `oc_mode_t mode`, `bool preload_enable` | Configure OC mode (CCxS=00, OCxM, OCxPE) | TIMx_CCMRx |
| `timer_enable_channel()` | `timer_handle_t*`, `uint8_t channel` | Enable output compare channel (CCxE) | TIMx_CCER |
| `timer_disable_channel()` | `timer_handle_t*`, `uint8_t channel` | Disable output compare channel (CCxE) | TIMx_CCER |
| `timer_set_channel_polarity()` | `timer_handle_t*`, `uint8_t channel`, `bool active_high` | Set output polarity (CCxP bit) | TIMx_CCER |

## Value Setters & Getters

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_set_prescaler()` | `timer_handle_t*`, `uint32_t prescaler` | Set timer prescaler value | TIMx_PSC |
| `timer_set_auto_reload()` | `timer_handle_t*`, `uint32_t value` | Set auto-reload value | TIMx_ARR |
| `timer_set_counter()` | `timer_handle_t*`, `uint32_t value` | Set current counter value | TIMx_CNT |
| `timer_get_counter()` | `timer_handle_t*` | Get current counter value | TIMx_CNT |
| `timer_set_compare_value()` | `timer_handle_t*`, `uint8_t channel`, `uint32_t value` | Set compare value for channel | TIMx_CCRx |
| `timer_get_compare_value()` | `timer_handle_t*`, `uint8_t channel` | Get compare value for channel | TIMx_CCRx |

## Advanced Timer Features (TIM1, TIM8)

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_set_repetition_counter()` | `timer_handle_t*`, `uint8_t value` | Set repetition counter (advanced timers only) | TIMx_RCR |
| `timer_enable_master_output()` | `timer_handle_t*` | Enable master output (MOE bit) | TIMx_BDTR |
| `timer_disable_master_output()` | `timer_handle_t*` | Disable master output (MOE bit) | TIMx_BDTR |
| `timer_set_dead_time()` | `timer_handle_t*`, `uint8_t dead_time` | Set dead-time value (DTG bits) | TIMx_BDTR |
| `timer_enable_complementary_output()` | `timer_handle_t*`, `uint8_t channel` | Enable complementary output (CCxNE) | TIMx_CCER |

## Interrupt Management

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_enable_interrupt()` | `timer_handle_t*`, `uint32_t interrupt_mask` | Enable specific interrupts | TIMx_DIER |
| `timer_disable_interrupt()` | `timer_handle_t*`, `uint32_t interrupt_mask` | Disable specific interrupts | TIMx_DIER |
| `timer_get_interrupt_status()` | `timer_handle_t*` | Get interrupt status flags | TIMx_SR |
| `timer_clear_interrupt()` | `timer_handle_t*`, `uint32_t interrupt_mask` | Clear interrupt flags | TIMx_SR |

## Utility Functions

| Method | Parameters | Description | Registers Used |
|--------|------------|-------------|----------------|
| `timer_is_running()` | `timer_handle_t*` | Check if timer is enabled | TIMx_CR1 |
| `timer_calculate_frequency()` | `uint32_t clock_freq`, `uint32_t prescaler`, `uint32_t arr` | Calculate output frequency | N/A |
| `timer_calculate_pwm_duty()` | `uint32_t ccr`, `uint32_t arr` | Calculate PWM duty cycle percentage | N/A |

## Data Structures & Enums

### Required Structures
```c
typedef struct {
    TIM_TypeDef *instance;
    uint32_t clock_freq;
    bool is_advanced;
} timer_handle_t;

typedef struct {
    bool auto_reload_preload;     // ARPE
    uint8_t count_mode;          // DIR + CMS (0-3)
    bool one_pulse_mode;         // OPM
    bool update_request_source;  // URS
    bool update_disable;         // UDIS
} timer_config_t;
```

### Required Enums
```c
typedef enum {
    OC_MODE_FROZEN = 0,
    OC_MODE_ACTIVE_ON_MATCH = 1,
    OC_MODE_INACTIVE_ON_MATCH = 2,
    OC_MODE_TOGGLE = 3,
    OC_MODE_FORCE_INACTIVE = 4,
    OC_MODE_FORCE_ACTIVE = 5,
    OC_MODE_PWM1 = 6,
    OC_MODE_PWM2 = 7
} oc_mode_t;

typedef enum {
    TIMER_OK = 0,
    TIMER_ERROR = 1,
    TIMER_INVALID_PARAM = 2
} timer_status_t;
```

**Total Methods: 25** (including utility functions)