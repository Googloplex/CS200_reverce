// Bit-banged SWD on plain GPIOs (direct register access for speed)

#include "swd.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define PIN_SWCLK   CONFIG_DAP_SWCLK_GPIO
#define PIN_SWDIO   CONFIG_DAP_SWDIO_GPIO
#define PIN_NRESET  CONFIG_DAP_NRESET_GPIO
#define PIN_LED     CONFIG_DAP_LED_GPIO

#define MASK_SWCLK  (1UL << PIN_SWCLK)
#define MASK_SWDIO  (1UL << PIN_SWDIO)

// Approximate CPU cycles spent on register accesses per half clock period
#define BIT_OVERHEAD_CYCLES 24

swd_config_t swd_cfg = {
    .turnaround = 1,
    .data_phase = false,
    .idle_cycles = 0,
};

static uint32_t s_half_period;  // CPU cycles

static inline void delay_half(void)
{
    if (s_half_period) {
        uint32_t start = esp_cpu_get_cycle_count();
        while ((uint32_t)(esp_cpu_get_cycle_count() - start) < s_half_period) {
        }
    }
}

static inline void clk_lo(void)  { REG_WRITE(GPIO_OUT_W1TC_REG, MASK_SWCLK); }
static inline void clk_hi(void)  { REG_WRITE(GPIO_OUT_W1TS_REG, MASK_SWCLK); }
static inline void dio_out(uint32_t bit)
{
    REG_WRITE(bit ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, MASK_SWDIO);
}
static inline uint32_t dio_in(void) { return (REG_READ(GPIO_IN_REG) >> PIN_SWDIO) & 1; }
static inline void dio_drive(void)   { REG_WRITE(GPIO_ENABLE_W1TS_REG, MASK_SWDIO); }
static inline void dio_release(void) { REG_WRITE(GPIO_ENABLE_W1TC_REG, MASK_SWDIO); }

static inline void write_bit(uint32_t bit)
{
    dio_out(bit);
    clk_lo();
    delay_half();
    clk_hi();
    delay_half();
}

static inline uint32_t read_bit(void)
{
    clk_lo();
    delay_half();
    uint32_t bit = dio_in();
    clk_hi();
    delay_half();
    return bit;
}

static inline void clock_cycle(void)
{
    clk_lo();
    delay_half();
    clk_hi();
    delay_half();
}

void swd_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = MASK_SWCLK | MASK_SWDIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_SWCLK, 1);
    gpio_set_level(PIN_SWDIO, 1);

#if PIN_NRESET >= 0
    gpio_set_level(PIN_NRESET, 1);
    gpio_config(&(gpio_config_t) {
        .pin_bit_mask = 1ULL << PIN_NRESET,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    });
#endif

#if PIN_LED >= 0
    gpio_config(&(gpio_config_t) {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    });
    gpio_set_level(PIN_LED, 0);
#endif

    swd_set_clock(CONFIG_DAP_DEFAULT_CLOCK_HZ);
}

void swd_connect(void)
{
    // Input stays enabled so SWDIO can be read back while we toggle output enable
    gpio_set_direction(PIN_SWCLK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_SWDIO, GPIO_MODE_INPUT_OUTPUT);
    clk_hi();
    dio_out(1);
    dio_drive();
}

void swd_disconnect(void)
{
    gpio_set_direction(PIN_SWCLK, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SWDIO, GPIO_MODE_INPUT);
}

void swd_set_clock(uint32_t hz)
{
    if (hz == 0) {
        hz = CONFIG_DAP_DEFAULT_CLOCK_HZ;
    }
    uint32_t half = (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000UL) / (2 * hz);
    s_half_period = half > BIT_OVERHEAD_CYCLES ? half - BIT_OVERHEAD_CYCLES : 0;
}

void swd_swdio_drive(bool drive)
{
    drive ? dio_drive() : dio_release();
}

void swd_seq_out(const uint8_t *data, uint32_t bits)
{
    for (uint32_t i = 0; i < bits; i++) {
        write_bit((data[i / 8] >> (i % 8)) & 1);
    }
}

void swd_seq_in(uint8_t *data, uint32_t bits)
{
    for (uint32_t i = 0; i < (bits + 7) / 8; i++) {
        data[i] = 0;
    }
    for (uint32_t i = 0; i < bits; i++) {
        data[i / 8] |= read_bit() << (i % 8);
    }
}

static void turnaround(void)
{
    for (uint32_t i = 0; i < swd_cfg.turnaround; i++) {
        clock_cycle();
    }
}

uint8_t swd_transfer(uint8_t request, uint32_t *data)
{
    uint32_t parity = __builtin_parity(request & 0x0F);
    // Start, APnDP, RnW, A2, A3, Parity, Stop, Park
    uint32_t header = 0x81 | ((request & 0x0F) << 1) | (parity << 5);
    for (uint32_t i = 0; i < 8; i++) {
        write_bit((header >> i) & 1);
    }

    dio_release();
    turnaround();

    uint32_t ack = 0;
    for (uint32_t i = 0; i < 3; i++) {
        ack |= read_bit() << i;
    }

    if (ack == SWD_ACK_OK) {
        if (request & 0x02) {
            uint32_t val = 0;
            for (uint32_t i = 0; i < 32; i++) {
                val |= read_bit() << i;
            }
            uint32_t par = read_bit();
            turnaround();
            dio_drive();
            if (par != (uint32_t)__builtin_parity(val)) {
                ack = SWD_PARITY_ERROR;
            }
            if (data) {
                *data = val;
            }
        } else {
            turnaround();
            dio_drive();
            uint32_t val = data ? *data : 0;
            for (uint32_t i = 0; i < 32; i++) {
                write_bit((val >> i) & 1);
            }
            write_bit(__builtin_parity(val));
        }
        dio_out(0);
        for (uint32_t i = 0; i < swd_cfg.idle_cycles; i++) {
            clock_cycle();
        }
        dio_out(1);
        return ack;
    }

    if (ack == SWD_ACK_WAIT || ack == SWD_ACK_FAULT) {
        if (swd_cfg.data_phase && (request & 0x02)) {
            for (uint32_t i = 0; i < 33; i++) {
                clock_cycle();
            }
        }
        turnaround();
        dio_drive();
        if (swd_cfg.data_phase && !(request & 0x02)) {
            dio_out(0);
            for (uint32_t i = 0; i < 33; i++) {
                clock_cycle();
            }
        }
        dio_out(1);
        return ack;
    }

    // Protocol error or no response: back off for a full data phase
    for (uint32_t i = 0; i < swd_cfg.turnaround + 33; i++) {
        clock_cycle();
    }
    dio_drive();
    dio_out(1);
    return ack;
}

void swd_pins_write(uint8_t value, uint8_t select)
{
    if (select & SWJ_PIN_SWCLK) {
        value & SWJ_PIN_SWCLK ? clk_hi() : clk_lo();
    }
    if (select & SWJ_PIN_SWDIO) {
        dio_out(value & SWJ_PIN_SWDIO);
    }
#if PIN_NRESET >= 0
    if (select & SWJ_PIN_NRESET) {
        gpio_set_level(PIN_NRESET, (value & SWJ_PIN_NRESET) ? 1 : 0);
    }
#endif
}

uint8_t swd_pins_read(void)
{
    uint32_t in = REG_READ(GPIO_IN_REG);
    uint8_t pins = 0;
    if (in & MASK_SWCLK) {
        pins |= SWJ_PIN_SWCLK;
    }
    if (in & MASK_SWDIO) {
        pins |= SWJ_PIN_SWDIO;
    }
#if PIN_NRESET >= 0
    if (gpio_get_level(PIN_NRESET)) {
        pins |= SWJ_PIN_NRESET;
    }
#else
    pins |= SWJ_PIN_NRESET;
#endif
    return pins;
}

void swd_led(bool on)
{
#if PIN_LED >= 0
    gpio_set_level(PIN_LED, on);
#else
    (void)on;
#endif
}
