#pragma once

// Driver for the Semtech SX1262 (SX126x family) LoRa transceiver.
//
// Unlike the SX127x this replaces, the SX126x is command driven: every exchange
// is an opcode followed by parameter bytes, and the chip raises a BUSY line while
// it digests one. Nothing may be sent while BUSY is high, so every transaction in
// here goes through wait_busy() first.
//
// Wiring assumed by the Kconfig defaults (Seeed Wio-SX1262 on a XIAO ESP32C3):
//   MOSI D10/GPIO10  MISO D9/GPIO9  SCK D8/GPIO8  NSS D4/GPIO6
//   DIO1 D1/GPIO3    RST  D2/GPIO4  BUSY D3/GPIO5  RF_SW D5/GPIO7

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Bandwidth selector. Values are the raw SetModulationParams encodings, so they
// are stable across NVS saves and safe to put straight into the wire format.
typedef enum {
    SX126X_BW_7810   = 0x00,
    SX126X_BW_10420  = 0x08,
    SX126X_BW_15630  = 0x01,
    SX126X_BW_20830  = 0x09,
    SX126X_BW_31250  = 0x02,
    SX126X_BW_41670  = 0x0A,
    SX126X_BW_62500  = 0x03,
    SX126X_BW_125000 = 0x04,
    SX126X_BW_250000 = 0x05,
    SX126X_BW_500000 = 0x06,
} sx126x_bw_t;

typedef struct {
    uint32_t freq_hz;    // carrier, e.g. 866000000
    uint8_t  sf;         // spreading factor, 5..12
    uint8_t  bw;         // sx126x_bw_t
    uint8_t  cr;         // coding-rate denominator 5..8 -> 4/5 .. 4/8
    uint16_t preamble;   // preamble length in symbols
    int8_t   tx_dbm;     // -9..+22
    uint8_t  sync_word;  // 0x12 private network, 0x34 public/LoRaWAN
    bool     crc_on;
} sx126x_cfg_t;

// Per-packet link quality, from GetPacketStatus.
typedef struct {
    float   rssi;         // RssiPkt, dBm
    float   snr;          // SnrPkt, dB
    float   signal_rssi;  // SignalRssiPkt, dBm (after despreading; valid below the noise floor)
    uint8_t len;
} sx126x_rxinfo_t;

// Running counters from GetStats. Cleared by sx126x_reset_stats().
typedef struct {
    uint16_t rx_ok;
    uint16_t crc_err;
    uint16_t hdr_err;
} sx126x_stats_t;

// Bring up SPI + GPIOs, hardware-reset the chip and verify it responds. Returns
// ESP_ERR_TIMEOUT if BUSY never drops (almost always a wiring fault).
esp_err_t sx126x_init(void);

// Apply a full radio profile. Safe to call at any time; puts the chip in standby
// first and leaves it there.
esp_err_t sx126x_apply(const sx126x_cfg_t *cfg);

// Transmit and block until TxDone or timeout. After a few consecutive failures
// (this, rx_start, or apply - a wedged BUSY line, a dead SPI bus) the driver
// hard-resets and reconfigures the chip on its own before returning the error,
// so a transient wiring/power fault clears without a full reboot.
esp_err_t sx126x_tx(const uint8_t *buf, uint8_t len, uint32_t timeout_ms);

// Enter continuous receive. timeout_ms == 0 means no timeout. Same automatic
// recovery as sx126x_tx() applies here.
esp_err_t sx126x_rx_start(uint32_t timeout_ms);

// Non-blocking. Returns the payload length, 0 if nothing is waiting, or -1 on a
// CRC/header error (which is itself worth logging during a range test).
int sx126x_rx_poll(uint8_t *buf, uint8_t max_len, sx126x_rxinfo_t *info);

// Block in receive until a packet arrives or the timeout expires. Same returns as
// sx126x_rx_poll(), plus 0 on timeout.
int sx126x_rx_wait(uint8_t *buf, uint8_t max_len, sx126x_rxinfo_t *info, uint32_t timeout_ms);

esp_err_t sx126x_get_stats(sx126x_stats_t *out);
esp_err_t sx126x_reset_stats(void);

// Instantaneous wideband RSSI. Only meaningful while the chip is in RX; used for
// the receiver dashboard's noise-floor readout.
float sx126x_rssi_inst(void);

esp_err_t sx126x_standby(void);
esp_err_t sx126x_sleep(void);

// ---- Pure maths, no hardware access. Safe to call from any task. ----

// Bandwidth selector -> Hz.
uint32_t sx126x_bw_hz(uint8_t bw);

// Time on air in milliseconds for a payload of payload_len bytes under cfg.
float sx126x_airtime_ms(const sx126x_cfg_t *cfg, uint8_t payload_len);

// Effective payload throughput in bit/s (payload bits / time on air).
float sx126x_bitrate_bps(const sx126x_cfg_t *cfg, uint8_t payload_len);

// Estimated receiver sensitivity in dBm for cfg.
float sx126x_sensitivity_dbm(const sx126x_cfg_t *cfg);

// Minimum demodulator SNR in dB for a spreading factor (-7.5 at SF7 .. -20 at SF12).
float sx126x_snr_floor_db(uint8_t sf);

// True when the low-data-rate optimisation is required (symbol time > 16 ms).
// Both ends must agree, so this is derived rather than configured.
bool sx126x_ldro_required(uint8_t sf, uint8_t bw);
