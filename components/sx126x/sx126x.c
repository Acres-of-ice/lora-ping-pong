#include "sx126x.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "sx126x";

// ---- Opcodes ----
#define CMD_SET_SLEEP              0x84
#define CMD_SET_STANDBY            0x80
#define CMD_SET_TX                 0x83
#define CMD_SET_RX                 0x82
#define CMD_SET_REGULATOR_MODE     0x96
#define CMD_CALIBRATE              0x89
#define CMD_CALIBRATE_IMAGE        0x98
#define CMD_SET_PA_CONFIG          0x95
#define CMD_WRITE_REGISTER         0x0D
#define CMD_READ_REGISTER          0x1D
#define CMD_WRITE_BUFFER           0x0E
#define CMD_READ_BUFFER            0x1E
#define CMD_SET_DIO_IRQ_PARAMS     0x08
#define CMD_GET_IRQ_STATUS         0x12
#define CMD_CLR_IRQ_STATUS         0x02
#define CMD_SET_DIO2_AS_RF_SWITCH  0x9D
#define CMD_SET_DIO3_AS_TCXO_CTRL  0x97
#define CMD_CLR_DEVICE_ERRORS      0x07
#define CMD_SET_RF_FREQUENCY       0x86
#define CMD_SET_PACKET_TYPE        0x8A
#define CMD_SET_TX_PARAMS          0x8E
#define CMD_SET_MODULATION_PARAMS  0x8B
#define CMD_SET_PACKET_PARAMS      0x8C
#define CMD_SET_BUFFER_BASE_ADDR   0x8F
#define CMD_GET_STATUS             0xC0
#define CMD_GET_RSSI_INST          0x15
#define CMD_GET_RX_BUFFER_STATUS   0x13
#define CMD_GET_PACKET_STATUS      0x14
#define CMD_GET_DEVICE_ERRORS      0x17
#define CMD_GET_STATS              0x10
#define CMD_RESET_STATS            0x00

// ---- Registers ----
#define REG_SYNCWORD_MSB   0x0740
#define REG_SYNCWORD_LSB   0x0741
#define REG_OCP            0x08E7
#define REG_TX_CLAMP       0x08D8
#define REG_TX_MODULATION  0x0889

// ---- IRQ bits ----
#define IRQ_TX_DONE     0x0001
#define IRQ_RX_DONE     0x0002
#define IRQ_HEADER_ERR  0x0020
#define IRQ_CRC_ERR     0x0040
#define IRQ_TIMEOUT     0x0200
#define IRQ_ALL_USED    (IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_TIMEOUT)

#define STDBY_RC        0x00
#define PACKET_TYPE_LORA 0x01
#define REGULATOR_LDO   0x00
#define REGULATOR_DCDC  0x01
#if CONFIG_SX126X_DCDC
#define REGULATOR_MODE  REGULATOR_DCDC
#define REGULATOR_NAME  "DC-DC"
#else
#define REGULATOR_MODE  REGULATOR_LDO
#define REGULATOR_NAME  "LDO"
#endif
#define RAMP_200U       0x04
#define LORA_HEADER_EXPLICIT 0x00

// GetDeviceErrors bits. XOSC_START_ERR is the one that matters most here: with an
// unpowered TCXO the chip happily accepts every command over SPI (that runs off
// the RC oscillator) but can never enter TX or RX, because those need the XOSC.
#define ERR_RC64K_CALIB  0x0001
#define ERR_RC13M_CALIB  0x0002
#define ERR_PLL_CALIB    0x0004
#define ERR_ADC_CALIB    0x0008
#define ERR_IMG_CALIB    0x0010
#define ERR_XOSC_START   0x0020
#define ERR_PLL_LOCK     0x0040
#define ERR_PA_RAMP      0x0100
// Only bits 0..8 are defined; a real part reads the rest back as zero.
#define ERR_RESERVED     0xFE00

// SetRx/SetTx timeouts count in 15.625 us steps, so 1 ms = 64 steps.
#define RTC_STEPS_PER_MS  64
#define RX_CONTINUOUS     0xFFFFFF
// Largest finite timeout the 24-bit field can hold (~262 s). 0xFFFFFF itself
// means "continuous" for RX, so finite timeouts stop one short of it.
#define TIMEOUT_MAX_STEPS 0xFFFFFE

// Convert a millisecond timeout to SetTx/SetRx steps, saturating rather than
// wrapping. A full-size packet at SF12/BW7.8kHz is ~145 s, and the callers'
// 2x-airtime budget for it overflowed 24 bits - the truncated value was a few
// seconds, so the chip aborted the packet with a timeout mid-air.
static uint32_t timeout_steps(uint32_t timeout_ms)
{
    const uint64_t steps = (uint64_t)timeout_ms * RTC_STEPS_PER_MS;
    return steps > TIMEOUT_MAX_STEPS ? TIMEOUT_MAX_STEPS : (uint32_t)steps;
}

// The chip accepts SPI up to 16 MHz; 8 MHz is comfortable over jumper wiring.
#define SPI_CLOCK_HZ  8000000

#if CONFIG_SX126X_SPI3_HOST
#define HOST_ID SPI3_HOST
#else
#define HOST_ID SPI2_HOST
#endif

static spi_device_handle_t s_spi;
static SemaphoreHandle_t   s_irq_sem;   // given by the DIO1 ISR
static SemaphoreHandle_t   s_lock;      // serialises SPI access
static sx126x_cfg_t        s_cfg;
static bool                s_ready;

// Consecutive TX/RX/apply failures before the driver stops waiting for a wedged
// chip to clear on its own and hard-resets it instead. Mirrors powerload.c's
// brownout streak: a fault that hasn't cleared after a few tries needs the radio
// power-cycled, not another retry against the same stuck state.
#define RECOVER_AFTER 3
static uint8_t s_fail_streak;

// Runs one exchange while the lock is held; on failure, unlocks and returns the
// error to the caller instead of aborting the process. A wedged radio (a wiring
// or supply fault, per wait_busy()/log_bus_fault() above) is a fact about the
// hardware the caller needs to hear, not a firmware bug worth crashing over.
#define BAIL(x) do { const esp_err_t _err = (x); if (_err != ESP_OK) { unlock(); return _err; } } while (0)

// Same as BAIL(), but also counts the failure toward RECOVER_AFTER. Used by the
// runtime entry points (apply/tx/rx_start); not by chip_bringup() itself, so a
// failed recovery attempt cannot recursively trigger another one.
#define BAIL_NOTE(x) do { const esp_err_t _err = (x); if (_err != ESP_OK) { unlock(); note_result(false); return _err; } } while (0)

static void IRAM_ATTR dio1_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_irq_sem, &hp);
    if (hp == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void lock(void)   { xSemaphoreTake(s_lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_lock); }

// The chip holds BUSY high while it processes the previous command. Sending
// anything during that window is silently dropped, so every transaction waits
// here first. A permanent high almost always means miswiring, not firmware.
static esp_err_t wait_busy(uint32_t timeout_ms)
{
    const int64_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms) + 1;
    while (gpio_get_level(CONFIG_SX126X_BUSY_GPIO) == 1) {
        if ((int64_t)xTaskGetTickCount() > deadline) {
            ESP_LOGE(TAG, "BUSY stuck high (GPIO%d) - check wiring/power", CONFIG_SX126X_BUSY_GPIO);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

// One NSS-low exchange: opcode, then params out / status in. tx and rx may be the
// same length; either may be NULL.
static esp_err_t xfer(uint8_t opcode, const uint8_t *tx, uint8_t *rx, size_t len)
{
    esp_err_t err = wait_busy(100);
    if (err != ESP_OK) {
        return err;
    }

    // Static and word-aligned: a 257-byte FIFO write exceeds the SPI hardware
    // FIFO, so it goes via DMA, which needs 4-byte-aligned DMA-capable memory -
    // a plain stack array guarantees neither. Every caller already holds the
    // driver lock, so sharing one pair of buffers is safe and keeps 520 bytes
    // off the radio task's stack.
    static WORD_ALIGNED_ATTR uint8_t out[260];
    static WORD_ALIGNED_ATTR uint8_t in[260];
    if (len + 1 > sizeof(out)) {
        return ESP_ERR_INVALID_SIZE;
    }
    out[0] = opcode;
    if (tx != NULL) {
        memcpy(&out[1], tx, len);
    } else {
        memset(&out[1], 0, len);
    }

    spi_transaction_t t = {
        .length    = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = in,
    };
    err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK && rx != NULL) {
        memcpy(rx, &in[1], len);
    }
    return err;
}

static esp_err_t cmd(uint8_t opcode, const uint8_t *params, size_t len)
{
    return xfer(opcode, params, NULL, len);
}

// Register access is opcode + 16-bit address + data, with one status byte of
// padding on reads.
static esp_err_t reg_write(uint16_t addr, uint8_t val)
{
    const uint8_t p[3] = { (uint8_t)(addr >> 8), (uint8_t)addr, val };
    return cmd(CMD_WRITE_REGISTER, p, sizeof(p));
}

static esp_err_t reg_read(uint16_t addr, uint8_t *val)
{
    uint8_t p[4] = { (uint8_t)(addr >> 8), (uint8_t)addr, 0x00, 0x00 };
    uint8_t r[4] = { 0 };
    esp_err_t err = xfer(CMD_READ_REGISTER, p, r, sizeof(p));
    if (err == ESP_OK) {
        *val = r[3];  // [0..1] address echo, [2] status byte, [3] data
    }
    return err;
}

static esp_err_t buf_write(uint8_t offset, const uint8_t *data, uint8_t len)
{
    uint8_t p[256];
    p[0] = offset;
    memcpy(&p[1], data, len);
    return cmd(CMD_WRITE_BUFFER, p, len + 1);
}

static esp_err_t buf_read(uint8_t offset, uint8_t *data, uint8_t len)
{
    uint8_t p[258] = { 0 };
    uint8_t r[258] = { 0 };
    p[0] = offset;
    esp_err_t err = xfer(CMD_READ_BUFFER, p, r, len + 2);
    if (err == ESP_OK) {
        memcpy(data, &r[2], len);  // [0] offset echo, [1] status byte
    }
    return err;
}

static esp_err_t set_standby(void)
{
    const uint8_t p = STDBY_RC;
    return cmd(CMD_SET_STANDBY, &p, 1);
}

static esp_err_t clear_irq(uint16_t mask)
{
    const uint8_t p[2] = { (uint8_t)(mask >> 8), (uint8_t)mask };
    return cmd(CMD_CLR_IRQ_STATUS, p, sizeof(p));
}

static esp_err_t get_irq(uint16_t *out)
{
    uint8_t r[3] = { 0 };
    esp_err_t err = xfer(CMD_GET_IRQ_STATUS, NULL, r, sizeof(r));
    if (err == ESP_OK) {
        *out = ((uint16_t)r[1] << 8) | r[2];  // r[0] is the status byte
    }
    return err;
}

// Must be called with the lock held.
static uint16_t read_device_errors(void)
{
    uint8_t r[3] = { 0 };
    if (xfer(CMD_GET_DEVICE_ERRORS, NULL, r, sizeof(r)) != ESP_OK) {
        return 0;
    }
    return ((uint16_t)r[1] << 8) | r[2];  // r[0] is the status byte
}

static const char *chip_mode_str(uint8_t status)
{
    switch ((status >> 4) & 0x07) {
        case 0x02: return "STDBY_RC";
        case 0x03: return "STDBY_XOSC";
        case 0x04: return "FS";
        case 0x05: return "RX";
        case 0x06: return "TX";
        default:   return "?";
    }
}

// True when a status/error pair cannot have come from an SX1262, so nothing in
// it is worth acting on. Chip modes 0, 1 and 7 are undefined and the top seven
// error bits are reserved.
//
// Worth checking before believing any of it. A MISO that nothing is driving
// reads as alternating 0xAA/0x55, and 0xAAAA happens to set the XOSC_START_ERR
// bit - so a plain wiring or brownout fault reports itself as a TCXO problem
// and sends you off measuring a voltage that was never wrong.
static bool reply_is_garbage(uint8_t status, uint16_t errors)
{
    const uint8_t mode = (status >> 4) & 0x07;
    return (errors & ERR_RESERVED) != 0 || mode < 0x02 || mode == 0x07;
}

static void log_bus_fault(void)
{
    ESP_LOGE(TAG, "  This reply cannot come from an SX1262, so the flags above mean");
    ESP_LOGE(TAG, "  nothing - the SPI read itself is bad. Look at the bus and the");
    ESP_LOGE(TAG, "  supply (MISO=GPIO%d, NSS=GPIO%d, shared ground, 3V3 sag under load),",
             CONFIG_SX126X_MISO_GPIO, CONFIG_SX126X_NSS_GPIO);
    ESP_LOGE(TAG, "  not at the radio configuration.");
}

// Log why the radio is unhappy. Called after a TX/RX timeout, where the useful
// information is the chip mode it is actually sitting in and the sticky error
// flags - a silent timeout on its own tells you almost nothing.
static void log_diagnostics(const char *what)
{
    uint8_t st[1] = { 0 };
    xfer(CMD_GET_STATUS, NULL, st, sizeof(st));
    const uint16_t err = read_device_errors();

    ESP_LOGE(TAG, "%s: chip mode %s, device errors 0x%04x", what, chip_mode_str(st[0]), err);

    if (reply_is_garbage(st[0], err)) {
        log_bus_fault();
        return;
    }
    if (err & ERR_XOSC_START) {
        ESP_LOGE(TAG, "  XOSC_START_ERR - the crystal/TCXO never started.");
#if CONFIG_SX126X_TCXO
        // Don't tell anyone to enable an option that is already on - that is how
        // a real fault gets read as a stale message and ignored.
        ESP_LOGE(TAG, "  DIO3 TCXO control is already on (voltage selector %d, %d us",
                 CONFIG_SX126X_TCXO_VOLTAGE, CONFIG_SX126X_TCXO_DELAY_US);
        ESP_LOGE(TAG, "  startup delay). Check the selector matches the part's supply,");
        ESP_LOGE(TAG, "  raise the delay, or confirm the module really has a TCXO.");
#else
        ESP_LOGE(TAG, "  If this module has a TCXO it needs DIO3 powering it:");
        ESP_LOGE(TAG, "  enable CONFIG_SX126X_TCXO and check the voltage matches the part.");
#endif
    }
    if (err & ERR_PLL_LOCK)  ESP_LOGE(TAG, "  PLL_LOCK_ERR - frequency out of range for this part?");
    if (err & ERR_PA_RAMP)   ESP_LOGE(TAG, "  PA_RAMP_ERR - check the supply can carry the PA current.");
    if (err & ERR_IMG_CALIB) ESP_LOGE(TAG, "  IMG_CALIB_ERR - image calibration band mismatch.");
}

// The external antenna switch on the Wio-SX1262 needs powering separately from
// DIO2's internal TX/RX steering. Some module revisions instead want this line
// driven high only during transmit, hence the Kconfig option. Modules like the
// Ra-01SH need nothing from the host here: DIO2 alone steers their switch.
static void rf_switch(bool transmitting)
{
#if !CONFIG_SX126X_RFSW_ENABLE
    (void)transmitting;
#elif CONFIG_SX126X_RFSW_TX_HIGH
    gpio_set_level(CONFIG_SX126X_RFSW_GPIO, transmitting ? 1 : 0);
#else
    (void)transmitting;
    gpio_set_level(CONFIG_SX126X_RFSW_GPIO, 1);
#endif
}

static void hw_reset(void)
{
    gpio_set_level(CONFIG_SX126X_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(CONFIG_SX126X_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// ---------------------------------------------------------------- maths ----

uint32_t sx126x_bw_hz(uint8_t bw)
{
    switch (bw) {
        case SX126X_BW_7810:   return 7810;
        case SX126X_BW_10420:  return 10420;
        case SX126X_BW_15630:  return 15630;
        case SX126X_BW_20830:  return 20830;
        case SX126X_BW_31250:  return 31250;
        case SX126X_BW_41670:  return 41670;
        case SX126X_BW_62500:  return 62500;
        case SX126X_BW_125000: return 125000;
        case SX126X_BW_250000: return 250000;
        case SX126X_BW_500000: return 500000;
        default:               return 125000;
    }
}

bool sx126x_ldro_required(uint8_t sf, uint8_t bw)
{
    // Symbol time above 16 ms needs the low-data-rate optimisation.
    return ((1u << sf) * 1000.0f / (float)sx126x_bw_hz(bw)) > 16.0f;
}

float sx126x_snr_floor_db(uint8_t sf)
{
    // Demodulator floor, -2.5 dB at SF5 falling 2.5 dB per step to -20 at SF12.
    if (sf < 5)  sf = 5;
    if (sf > 12) sf = 12;
    return -2.5f * (float)(sf - 4);
}

float sx126x_airtime_ms(const sx126x_cfg_t *cfg, uint8_t payload_len)
{
    const float bw = (float)sx126x_bw_hz(cfg->bw);
    const int   sf = cfg->sf;
    const float tsym_ms = (float)(1u << sf) * 1000.0f / bw;

    const int de  = sx126x_ldro_required(cfg->sf, cfg->bw) ? 1 : 0;
    const int crc = cfg->crc_on ? 1 : 0;
    const int ih  = 0;  // explicit header throughout

    // SF5/SF6 use a longer preamble and take no low-data-rate term.
    const float n_preamble = (float)cfg->preamble + (sf < 7 ? 6.25f : 4.25f);
    const int   den = (sf < 7) ? (4 * sf) : (4 * (sf - 2 * de));

    const int num = 8 * (int)payload_len - 4 * sf + 28 + 16 * crc - 20 * ih;
    int blocks = (num + den - 1) / den;  // ceil, num can be negative for tiny payloads
    if (blocks < 0) {
        blocks = 0;
    }
    // cfg->cr is the denominator 5..8, which is exactly the (CR + 4) multiplier.
    const float n_payload = 8.0f + (float)(blocks * cfg->cr);

    return (n_preamble + n_payload) * tsym_ms;
}

float sx126x_bitrate_bps(const sx126x_cfg_t *cfg, uint8_t payload_len)
{
    const float toa = sx126x_airtime_ms(cfg, payload_len);
    if (toa <= 0.0f) {
        return 0.0f;
    }
    return (float)payload_len * 8.0f * 1000.0f / toa;
}

float sx126x_sensitivity_dbm(const sx126x_cfg_t *cfg)
{
    // Thermal noise floor + receiver noise figure + demodulator SNR floor.
    const float nf = 6.0f;  // SX1262 typical
    return -174.0f + 10.0f * log10f((float)sx126x_bw_hz(cfg->bw)) + nf +
           sx126x_snr_floor_db(cfg->sf);
}

// ----------------------------------------------------------------- init ----

// Hardware-reset the chip and run it all the way back up to a configured-and-
// verified state. Runs both at boot (from sx126x_init(), after the one-time
// GPIO/SPI/ISR setup below) and at runtime, from note_result(), when the chip
// has gone unresponsive - a plain hw_reset() pulse costs nothing to retry, so a
// power/wiring fault that hasn't actually cleared is reported back up rather
// than crashing the process trying.
static esp_err_t chip_bringup(void)
{
    s_ready = false;  // stay unready for the duration; a caller mid-recovery
                       // must see INVALID_STATE, not a half-configured chip.

    rf_switch(false);
    hw_reset();

    lock();
    BAIL(wait_busy(100));  // BUSY never dropped: wiring, not firmware

    BAIL(set_standby());

    // DC-DC only where the module fits the inductor for it; the Ra-01SH does not.
    const uint8_t reg_mode = REGULATOR_MODE;
    BAIL(cmd(CMD_SET_REGULATOR_MODE, &reg_mode, 1));

    // DIO2 drives the module's internal TX/RX steering (on the Ra-01SH it is
    // wired to the antenna switch inside the module, hence unconnected outside).
    const uint8_t dio2_rf = 0x01;
    BAIL(cmd(CMD_SET_DIO2_AS_RF_SWITCH, &dio2_rf, 1));

#if CONFIG_SX126X_TCXO
    // Modules like the Wio-SX1262 clock from a TCXO that DIO3 powers. Without this the XOSC
    // never starts, and the failure is deceptive: every SPI command still works
    // (STDBY_RC runs off the RC oscillator) but TX and RX silently never happen.
    // The delay is how long the chip waits for the TCXO to settle, in 15.625 us
    // steps.
    const uint32_t tcxo_delay = (uint32_t)CONFIG_SX126X_TCXO_DELAY_US / 16;
    const uint8_t tcxo[4] = {
        CONFIG_SX126X_TCXO_VOLTAGE,
        (uint8_t)(tcxo_delay >> 16), (uint8_t)(tcxo_delay >> 8), (uint8_t)tcxo_delay
    };
    BAIL(cmd(CMD_SET_DIO3_AS_TCXO_CTRL, tcxo, sizeof(tcxo)));
#endif

    // Clear first: the chip latches an XOSC_START_ERR during its own power-up,
    // before DIO3 was configured, and device errors are sticky until cleared.
    // Checking before this point reports a failure that has already been fixed.
    const uint8_t clr[2] = { 0x00, 0x00 };
    BAIL(cmd(CMD_CLR_DEVICE_ERRORS, clr, sizeof(clr)));

    // Calibrate once the clock source is settled - the datasheet requires a
    // recalibration after DIO3/TCXO control is configured, because everything
    // before this point was calibrated against the RC oscillator.
    const uint8_t cal_all = 0x7F;
    BAIL(cmd(CMD_CALIBRATE, &cal_all, 1));
    vTaskDelay(pdMS_TO_TICKS(20));
    BAIL(wait_busy(200));

    // Now actively force the crystal up rather than waiting to discover it failed
    // as an unexplained TX timeout later. STDBY_XOSC is the only standby mode that
    // requires the oscillator, so reaching it proves the clock works.
    const uint8_t stdby_xosc = 0x01;
    BAIL(cmd(CMD_SET_STANDBY, &stdby_xosc, 1));
    vTaskDelay(pdMS_TO_TICKS(20));
    BAIL(wait_busy(200));

    const uint16_t osc_err = read_device_errors();
    uint8_t osc_status[1] = { 0 };
    xfer(CMD_GET_STATUS, NULL, osc_status, sizeof(osc_status));
    // Order matters: check the reply is real before checking what it says, or a
    // dead bus gets reported as a specific oscillator fault.
    if (reply_is_garbage(osc_status[0], osc_err)) {
        log_diagnostics("oscillator");
        unlock();
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (osc_err & ERR_XOSC_START) {
        log_diagnostics("oscillator");
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Oscillator started (mode %s%s)", chip_mode_str(osc_status[0]),
#if CONFIG_SX126X_TCXO
             ", TCXO on DIO3");
#else
             ", crystal");
#endif
    BAIL(set_standby());  // back to STDBY_RC for configuration

    const uint8_t pkt_lora = PACKET_TYPE_LORA;
    BAIL(cmd(CMD_SET_PACKET_TYPE, &pkt_lora, 1));

    // Errata 15.2: raise the Tx clamp threshold so an antenna mismatch cannot
    // damage the PA.
    uint8_t clamp = 0;
    BAIL(reg_read(REG_TX_CLAMP, &clamp));
    BAIL(reg_write(REG_TX_CLAMP, clamp | 0x1E));

    const uint8_t base_addr[2] = { 0x00, 0x00 };
    BAIL(cmd(CMD_SET_BUFFER_BASE_ADDR, base_addr, sizeof(base_addr)));

    // Confirm the chip is really answering: a live SX1262 round-trips a register.
    BAIL(reg_write(REG_SYNCWORD_MSB, 0x14));
    uint8_t probe = 0;
    BAIL(reg_read(REG_SYNCWORD_MSB, &probe));
    unlock();

    if (probe != 0x14) {
        ESP_LOGE(TAG, "Register readback failed: wrote 0x14, read 0x%02x", probe);
        // An absent or unpowered module leaves MISO at one rail. A module that
        // answers with the wrong bits is present and clocking, just not reliably -
        // completely different thing to go and check.
        if (probe == 0x00 || probe == 0xFF) {
            ESP_LOGE(TAG, "  MISO is stuck %s - nothing is driving it. Check MISO=GPIO%d,",
                     probe ? "high" : "low", CONFIG_SX126X_MISO_GPIO);
            ESP_LOGE(TAG, "  NSS=GPIO%d and 3V3 to the module.", CONFIG_SX126X_NSS_GPIO);
        } else {
            ESP_LOGE(TAG, "  The chip is answering but corrupting bits, so it is present");
            ESP_LOGE(TAG, "  and clocked - this is signal integrity, not a missing module.");
            ESP_LOGE(TAG, "  Shorten the SPI leads, check the ground return, or drop");
            ESP_LOGE(TAG, "  SPI_CLOCK_HZ (now %d) to 2000000 and retry.", SPI_CLOCK_HZ);
        }
        return ESP_ERR_NOT_FOUND;
    }

    s_ready = true;
    ESP_LOGI(TAG, "SX1262 up: NSS=%d SCK=%d MOSI=%d MISO=%d RST=%d BUSY=%d DIO1=%d, %s regulator",
             CONFIG_SX126X_NSS_GPIO, CONFIG_SX126X_SCK_GPIO, CONFIG_SX126X_MOSI_GPIO,
             CONFIG_SX126X_MISO_GPIO, CONFIG_SX126X_RST_GPIO, CONFIG_SX126X_BUSY_GPIO,
             CONFIG_SX126X_DIO1_GPIO, REGULATOR_NAME);
#if CONFIG_SX126X_RFSW_ENABLE
    ESP_LOGI(TAG, "RF switch enable on GPIO%d", CONFIG_SX126X_RFSW_GPIO);
#endif
    return ESP_OK;
}

esp_err_t sx126x_init(void)
{
    s_irq_sem = xSemaphoreCreateBinary();
    s_lock    = xSemaphoreCreateMutex();
    if (s_irq_sem == NULL || s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SX126X_RST_GPIO)
#if CONFIG_SX126X_RFSW_ENABLE
                        | (1ULL << CONFIG_SX126X_RFSW_GPIO)
#endif
                        ,
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    const gpio_config_t busy_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SX126X_BUSY_GPIO),
        .mode         = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&busy_cfg));

    const gpio_config_t dio1_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SX126X_DIO1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&dio1_cfg));

    esp_err_t isr = gpio_install_isr_service(0);
    if (isr != ESP_OK && isr != ESP_ERR_INVALID_STATE) {  // already installed is fine
        return isr;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_SX126X_DIO1_GPIO, dio1_isr, NULL));

    const spi_bus_config_t bus = {
        .miso_io_num     = CONFIG_SX126X_MISO_GPIO,
        .mosi_io_num     = CONFIG_SX126X_MOSI_GPIO,
        .sclk_io_num     = CONFIG_SX126X_SCK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 512,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST_ID, &bus, SPI_DMA_CH_AUTO));

    const spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode           = 0,
        .spics_io_num   = CONFIG_SX126X_NSS_GPIO,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HOST_ID, &dev, &s_spi));

    return chip_bringup();
}

// Called after every apply/tx/rx_start with whether that command round-trip
// succeeded. A radio that hasn't recovered on its own after a few consecutive
// failures gets a hard reset here rather than being left to fail forever (or,
// as before this fix, taking the whole board down via ESP_ERROR_CHECK).
static void note_result(bool ok)
{
    if (ok) {
        s_fail_streak = 0;
        return;
    }
    if (++s_fail_streak < RECOVER_AFTER) {
        return;
    }
    s_fail_streak = 0;
    ESP_LOGE(TAG, "%u consecutive radio failures - hard-resetting the chip", RECOVER_AFTER);
    if (chip_bringup() != ESP_OK) {
        ESP_LOGE(TAG, "Recovery failed; still stuck - check wiring/power");
        return;
    }
    // A hard reset forgets every setting; without this the chip comes back up on
    // the factory-default profile instead of the one actually in use, and looks
    // recovered while quietly deaf.
    sx126x_apply(&s_cfg);
}

// SX1262 high-power PA tiers. Using the tier matched to the requested power
// keeps efficiency sane instead of always running the +22 dBm setup.
static void pa_tier_for(int8_t dbm, uint8_t *duty, uint8_t *hp_max)
{
    if (dbm >= 22)      { *duty = 0x04; *hp_max = 0x07; }
    else if (dbm >= 20) { *duty = 0x03; *hp_max = 0x05; }
    else if (dbm >= 17) { *duty = 0x02; *hp_max = 0x03; }
    else                { *duty = 0x02; *hp_max = 0x02; }
}

esp_err_t sx126x_apply(const sx126x_cfg_t *cfg)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    sx126x_cfg_t c = *cfg;
    if (c.sf < 5)  c.sf = 5;
    if (c.sf > 12) c.sf = 12;
    if (c.cr < 5)  c.cr = 5;
    if (c.cr > 8)  c.cr = 8;
    if (c.tx_dbm < -9) c.tx_dbm = -9;
    if (c.tx_dbm > 22) c.tx_dbm = 22;
    if (c.preamble < 6) c.preamble = 6;

    lock();
    BAIL_NOTE(set_standby());

    const uint64_t frf = ((uint64_t)c.freq_hz << 25) / 32000000ULL;
    const uint8_t freq_p[4] = {
        (uint8_t)(frf >> 24), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf
    };
    BAIL_NOTE(cmd(CMD_SET_RF_FREQUENCY, freq_p, sizeof(freq_p)));

    // Image calibration is per band. 863-870 MHz covers the 865-867 MHz plan.
    uint8_t img[2];
    if (c.freq_hz >= 902000000)      { img[0] = 0xE1; img[1] = 0xE9; }
    else if (c.freq_hz >= 863000000) { img[0] = 0xD7; img[1] = 0xDB; }
    else if (c.freq_hz >= 779000000) { img[0] = 0xC1; img[1] = 0xC5; }
    else if (c.freq_hz >= 470000000) { img[0] = 0x75; img[1] = 0x81; }
    else                             { img[0] = 0x6B; img[1] = 0x6F; }
    BAIL_NOTE(cmd(CMD_CALIBRATE_IMAGE, img, sizeof(img)));

    uint8_t duty, hp_max;
    pa_tier_for(c.tx_dbm, &duty, &hp_max);
    const uint8_t pa[4] = { duty, hp_max, 0x00 /* SX1262 */, 0x01 };
    BAIL_NOTE(cmd(CMD_SET_PA_CONFIG, pa, sizeof(pa)));

    // SetPaConfig resets OCP; put it back to 140 mA so the HP PA is not current
    // limited at +22 dBm.
    BAIL_NOTE(reg_write(REG_OCP, 0x38));

    const uint8_t txp[2] = { (uint8_t)c.tx_dbm, RAMP_200U };
    BAIL_NOTE(cmd(CMD_SET_TX_PARAMS, txp, sizeof(txp)));

    // LDRO is derived, never configured: a mismatch between the two ends stops
    // anything from decoding.
    const uint8_t ldro = sx126x_ldro_required(c.sf, c.bw) ? 0x01 : 0x00;
    const uint8_t mod[4] = { c.sf, c.bw, (uint8_t)(c.cr - 4), ldro };
    BAIL_NOTE(cmd(CMD_SET_MODULATION_PARAMS, mod, sizeof(mod)));

    // Errata 15.1.2: modulation quality workaround, bit 2 of RegTxModulation.
    uint8_t txmod = 0;
    BAIL_NOTE(reg_read(REG_TX_MODULATION, &txmod));
    if (c.bw == SX126X_BW_500000) {
        BAIL_NOTE(reg_write(REG_TX_MODULATION, txmod & 0xFB));
    } else {
        BAIL_NOTE(reg_write(REG_TX_MODULATION, txmod | 0x04));
    }

    const uint8_t pkt[6] = {
        (uint8_t)(c.preamble >> 8), (uint8_t)c.preamble,
        LORA_HEADER_EXPLICIT,
        0xFF,                       // payload length, overwritten per transmit
        c.crc_on ? 0x01 : 0x00,
        0x00,                       // standard IQ
    };
    BAIL_NOTE(cmd(CMD_SET_PACKET_PARAMS, pkt, sizeof(pkt)));

    BAIL_NOTE(reg_write(REG_SYNCWORD_MSB, (uint8_t)((c.sync_word & 0xF0) | 0x04)));
    BAIL_NOTE(reg_write(REG_SYNCWORD_LSB, (uint8_t)(((c.sync_word & 0x0F) << 4) | 0x04)));

    const uint8_t irq[8] = {
        (uint8_t)(IRQ_ALL_USED >> 8), (uint8_t)IRQ_ALL_USED,  // global mask
        (uint8_t)(IRQ_ALL_USED >> 8), (uint8_t)IRQ_ALL_USED,  // DIO1
        0x00, 0x00,                                           // DIO2 is the RF switch
        0x00, 0x00,                                           // DIO3 unused
    };
    BAIL_NOTE(cmd(CMD_SET_DIO_IRQ_PARAMS, irq, sizeof(irq)));
    BAIL_NOTE(clear_irq(0xFFFF));

    s_cfg = c;
    unlock();
    note_result(true);

    ESP_LOGI(TAG, "profile: %.3f MHz SF%d BW%lu CR4/%d pre%d %+d dBm CRC%s LDRO%s",
             c.freq_hz / 1e6, c.sf, (unsigned long)sx126x_bw_hz(c.bw), c.cr,
             c.preamble, c.tx_dbm, c.crc_on ? "on" : "off", ldro ? "on" : "off");
    return ESP_OK;
}

// ------------------------------------------------------------------ tx/rx ----

// Payload length lives in SetPacketParams, so it has to be rewritten per packet.
static esp_err_t set_payload_len(uint8_t len)
{
    const uint8_t pkt[6] = {
        (uint8_t)(s_cfg.preamble >> 8), (uint8_t)s_cfg.preamble,
        LORA_HEADER_EXPLICIT,
        len,
        s_cfg.crc_on ? 0x01 : 0x00,
        0x00,
    };
    return cmd(CMD_SET_PACKET_PARAMS, pkt, sizeof(pkt));
}

esp_err_t sx126x_tx(const uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    lock();
    xSemaphoreTake(s_irq_sem, 0);  // drop any stale give from a previous packet

    BAIL_NOTE(set_standby());
    BAIL_NOTE(clear_irq(0xFFFF));
    BAIL_NOTE(set_payload_len(len));
    BAIL_NOTE(buf_write(0, buf, len));

    rf_switch(true);

    const uint32_t steps = timeout_steps(timeout_ms);
    const uint8_t p[3] = { (uint8_t)(steps >> 16), (uint8_t)(steps >> 8), (uint8_t)steps };
    const esp_err_t tx_err = cmd(CMD_SET_TX, p, sizeof(p));
    if (tx_err != ESP_OK) {
        rf_switch(false);  // don't leave the antenna switch armed for a TX that never started
        unlock();
        note_result(false);
        return tx_err;
    }
    unlock();

    // Wait off-lock so the HTTP task can still read cached state. A little slack
    // past the chip's own timeout so its IRQ wins the race normally.
    const BaseType_t got = xSemaphoreTake(s_irq_sem, pdMS_TO_TICKS(timeout_ms + 200));

    lock();
    uint16_t flags = 0;
    get_irq(&flags);
    const bool failed = (!got || (flags & IRQ_TX_DONE) == 0);
    if (failed) {
        ESP_LOGW(TAG, "TX timeout (irq=0x%04x)", flags);
        log_diagnostics("TX");
    }
    clear_irq(0xFFFF);
    rf_switch(false);
    set_standby();
    unlock();

    note_result(!failed);
    return failed ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t sx126x_rx_start(uint32_t timeout_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    lock();
    xSemaphoreTake(s_irq_sem, 0);
    BAIL_NOTE(clear_irq(0xFFFF));
    BAIL_NOTE(set_payload_len(0xFF));  // explicit header: accept any length

    rf_switch(false);

    const uint32_t steps = (timeout_ms == 0) ? RX_CONTINUOUS : timeout_steps(timeout_ms);
    const uint8_t p[3] = { (uint8_t)(steps >> 16), (uint8_t)(steps >> 8), (uint8_t)steps };
    esp_err_t err = cmd(CMD_SET_RX, p, sizeof(p));
    unlock();
    note_result(err == ESP_OK);
    return err;
}

int sx126x_rx_poll(uint8_t *buf, uint8_t max_len, sx126x_rxinfo_t *info)
{
    if (!s_ready) {
        return 0;
    }

    lock();
    uint16_t flags = 0;
    if (get_irq(&flags) != ESP_OK || (flags & (IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_HEADER_ERR)) == 0) {
        unlock();
        return 0;
    }

    if (flags & (IRQ_CRC_ERR | IRQ_HEADER_ERR)) {
        clear_irq(0xFFFF);
        unlock();
        return -1;
    }

    uint8_t st[4] = { 0 };
    xfer(CMD_GET_RX_BUFFER_STATUS, NULL, st, sizeof(st));
    uint8_t len    = st[1];  // st[0] is the status byte
    uint8_t offset = st[2];
    if (len > max_len) {
        len = max_len;
    }
    if (len > 0) {
        buf_read(offset, buf, len);
    }

    if (info != NULL) {
        uint8_t ps[4] = { 0 };
        xfer(CMD_GET_PACKET_STATUS, NULL, ps, sizeof(ps));
        info->rssi        = -((float)ps[1]) / 2.0f;
        info->snr         = ((float)(int8_t)ps[2]) / 4.0f;
        info->signal_rssi = -((float)ps[3]) / 2.0f;
        info->len         = len;
    }

    clear_irq(0xFFFF);
    unlock();
    return len;
}

int sx126x_rx_wait(uint8_t *buf, uint8_t max_len, sx126x_rxinfo_t *info, uint32_t timeout_ms)
{
    // Sleep until DIO1 fires so a multi-second wait at SF12 costs no SPI traffic.
    // The result is deliberately ignored: an edge that landed before the wait
    // started leaves a packet sitting in the buffer with no semaphore to take, so
    // poll on the timeout path too rather than reporting nothing.
    (void)xSemaphoreTake(s_irq_sem, pdMS_TO_TICKS(timeout_ms));
    return sx126x_rx_poll(buf, max_len, info);
}

esp_err_t sx126x_get_stats(sx126x_stats_t *out)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t r[7] = { 0 };
    lock();
    esp_err_t err = xfer(CMD_GET_STATS, NULL, r, sizeof(r));
    unlock();
    if (err == ESP_OK) {
        out->rx_ok   = ((uint16_t)r[1] << 8) | r[2];
        out->crc_err = ((uint16_t)r[3] << 8) | r[4];
        out->hdr_err = ((uint16_t)r[5] << 8) | r[6];
    }
    return err;
}

esp_err_t sx126x_reset_stats(void)
{
    const uint8_t zero[6] = { 0 };
    lock();
    esp_err_t err = cmd(CMD_RESET_STATS, zero, sizeof(zero));
    unlock();
    return err;
}

float sx126x_rssi_inst(void)
{
    if (!s_ready) {
        return 0.0f;
    }
    uint8_t r[2] = { 0 };
    lock();
    esp_err_t err = xfer(CMD_GET_RSSI_INST, NULL, r, sizeof(r));
    unlock();
    if (err != ESP_OK) {
        return 0.0f;
    }
    return -((float)r[1]) / 2.0f;
}

esp_err_t sx126x_standby(void)
{
    lock();
    rf_switch(false);
    esp_err_t err = set_standby();
    unlock();
    return err;
}

esp_err_t sx126x_sleep(void)
{
    const uint8_t p = 0x04;  // warm start, retain configuration
    lock();
    rf_switch(false);
    esp_err_t err = cmd(CMD_SET_SLEEP, &p, 1);
    unlock();
    return err;
}
