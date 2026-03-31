// =============================================================================
// PMW3389 Optical Sensor Driver — RP2040 / Pico W
// =============================================================================
// Hardware:  Raspberry Pi Pico W (RP2040)
// Sensor:    PixArt PMW3389
// Interface: SPI0 @ 2MHz (1MHz during SROM upload)
// Framework: Pico SDK + TinyUSB
// =============================================================================

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "srom3389.h"

// -----------------------------------------------------------------------------
// Pin & peripheral config
// -----------------------------------------------------------------------------
#define PIN_LED     CYW43_WL_GPIO_LED_PIN
#define BTN_LEFT    19
#define BTN_RIGHT   14

#define SPI_PORT    spi0
#define PIN_SCK     2
#define PIN_MOSI    3
#define PIN_MISO    4
#define PIN_CS      5
#define PIN_MT      6   // Motion interrupt (active low, pulled up — not polled yet)
#define PIN_RST     7   // Hardware reset (active low)

// -----------------------------------------------------------------------------
// PMW3389 register addresses (direct register map)
// -----------------------------------------------------------------------------
#define REG_PRODUCT_ID      0x00
#define REG_REVISION_ID     0x01
#define REG_MOTION          0x02
#define REG_DELTA_X_L       0x03
#define REG_DELTA_X_H       0x04
#define REG_DELTA_Y_L       0x05
#define REG_DELTA_Y_H       0x06
#define REG_SQUAL           0x07`
#define REG_RAW_DATA_SUM    0x08
#define REG_MAX_RAW_DATA    0x09
#define REG_MIN_RAW_DATA    0x0A
#define REG_SHUTTER_L       0x0B
#define REG_SHUTTER_H       0x0C
#define REG_CONTROL         0x0D
#define REG_CONFIG1         0x0F   // Resolution_L (CPI lower byte)
#define REG_CONFIG2         0x10   // 0x00 = wired/rest-mode-off
#define REG_ANGLE_TUNE      0x3B
#define REG_SROM_ENABLE     0x13
#define REG_OBSERVATION     0x24
#define REG_SROM_ID         0x2A
#define REG_BURST_MOTION    0x50   // Burst read start command
#define REG_BURST_SROM      0x62   // SROM burst write command

// Motion register bit masks
#define MOT_BIT_MOTION      (1 << 7)  // Bit 7: motion detected
#define MOT_BIT_OBSERVATION (1 << 6)  // Bit 6: entered observation mode
#define MOT_BIT_LIFT        (1 << 3)  // Bit 3: sensor lifted (SQUAL too low)

// CPI configuration
// Formula: CPI = (REG_CONFIG1 + 1) * 50
// For 800 CPI: (800 / 50) - 1 = 15 = 0x0F
#define CPI_800   0x0F
#define CPI_1600  0x1F
#define CPI_3200  0x3F

// Expected SROM firmware signature
#define SROM_EXPECTED_ID    0xE8

// HID report ID
#define REPORT_ID_MOUSE     0

// -----------------------------------------------------------------------------
// Burst read data layout (12 bytes, in order from sensor)
// -----------------------------------------------------------------------------
// [0]  Motion        (MOT_BIT_MOTION | MOT_BIT_LIFT etc.)
// [1]  Observation
// [2]  Delta_X_L
// [3]  Delta_X_H
// [4]  Delta_Y_L
// [5]  Delta_Y_H
// [6]  SQUAL         (surface quality — must be > ~80 for valid tracking)
// [7]  Raw_Data_Sum
// [8]  Max_Raw_Data
// [9]  Min_Raw_Data
// [10] Shutter_Upper
// [11] Shutter_Lower
#define BURST_BYTES         12
#define BURST_IDX_MOTION    0
#define BURST_IDX_OBS       1
#define BURST_IDX_DX_L      2
#define BURST_IDX_DX_H      3
#define BURST_IDX_DY_L      4
#define BURST_IDX_DY_H      5
#define BURST_IDX_SQUAL     6
#define BURST_IDX_RAWSUM    7
#define BURST_IDX_MAXRAW    8
#define BURST_IDX_MINRAW    9
#define BURST_IDX_SHUT_H    10
#define BURST_IDX_SHUT_L    11

// =============================================================================
// SPI primitives
// =============================================================================

// Drain any stale bytes sitting in the RX FIFO before a new transaction.
// Prevents a dirty FIFO from corrupting the first byte of the next read.
static inline void pmw_spi_flush(void) {
    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }
    // NOTE: do NOT read ->sr here — it is read-only status and discarding it
    // does nothing useful. Removed the dead read that was in the original.
}

// Single-register read.
// Timing per PMW3389 datasheet:
//   tSRAD = 160us (address → data turnaround)
//   tSCLK-NCS-read = 1us min after last clock before CS high
//   tSRR / tSRW = 20us min (CS high between non-burst reads)
uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx_addr = reg & 0x7F;  // ensure write bit is clear
    uint8_t rx_garbage, rx_data;
    uint8_t tx_dummy = 0x00;

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);  // tNCS-SCLK: 120ns min — 1us is safe

    spi_write_read_blocking(SPI_PORT, &tx_addr, &rx_garbage, 1);
    sleep_us(160);  // tSRAD: 160us mandatory address-to-data delay

    spi_write_read_blocking(SPI_PORT, &tx_dummy, &rx_data, 1);
    sleep_us(1);    // tSCLK-NCS-read: 120ns min

    gpio_put(PIN_CS, 1);
    sleep_us(20);   // tSRR: 20us min between reads (tSRW=20us for write after read)

    return rx_data;
}

// Single-register write.
// Timing per PMW3389 datasheet:
//   tSCLK-NCS-write = 35us min (CS stays high after write)
//   tSWR = 20us min, tSWW = 20us min between writes
//   Using 145us post-CS for conservative margin (matches QMK)
void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), data };
    uint8_t rx[2];

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);  // tNCS-SCLK: 120ns min

    spi_write_read_blocking(SPI_PORT, tx, rx, 2);

    sleep_us(35);   // tSCLK-NCS-write: 35us min, hold before CS high
    gpio_put(PIN_CS, 1);
    sleep_us(145);  // tSWR/tSWW: conservative post-write idle time
}

// =============================================================================
// SROM firmware upload
// =============================================================================

void pmw_upload_srom(void) {
    // Drop to 1MHz during SROM upload
    spi_set_baudrate(SPI_PORT, 1000 * 1000);

    // Step 1: SROM enable sequence
    pmw_write_reg(REG_SROM_ENABLE, 0x1D);
    sleep_ms(10);
    pmw_write_reg(REG_SROM_ENABLE, 0x18);
    sleep_ms(10);

    // Step 2: Begin SROM burst write
    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);

    uint8_t burst_cmd = REG_BURST_SROM | 0x80;
    uint8_t rx_garbage;
    spi_write_read_blocking(SPI_PORT, &burst_cmd, &rx_garbage, 1);
    sleep_us(15);

    // Step 3: Clock out full SROM array
    for (int i = 0; i < firmware_length; i++) {
        spi_write_read_blocking(SPI_PORT, &srom_data[i], &rx_garbage, 1);
        sleep_us(15);
    }

    gpio_put(PIN_CS, 1);
    sleep_us(200);

    // Step 4: Restore baud rate BEFORE any register access
    spi_set_baudrate(SPI_PORT, 2000 * 1000);

    // Step 5: Clear observation mode — sensor enters obs mode during
    // SROM upload and MUST be explicitly cleared before anything else
    pmw_write_reg(0x24, 0x00);
    sleep_ms(10);
    uint8_t obs_check = pmw_read_reg(0x24);
    printf("[SROM] OBS after clear: 0x%02X (should be 0x00)\n", obs_check);

    // Step 6: Disable rest mode (only valid after obs mode cleared)
    pmw_write_reg(REG_CONFIG2, 0x00);
    sleep_ms(10);
    uint8_t cfg2_check = pmw_read_reg(REG_CONFIG2);
    printf("[SROM] Config2 after write: 0x%02X (should be 0x00)\n", cfg2_check);

    // Step 7: Verify SROM
    uint8_t srom_id = pmw_read_reg(REG_SROM_ID);
    printf("[SROM] SROM_ID = 0x%02X (expected 0x%02X)\n", srom_id, SROM_EXPECTED_ID);

    if (srom_id != SROM_EXPECTED_ID) {
        printf("[ERR] SROM UPLOAD FAILED — sensor will not track\n");
        //cyw43_arch_gpio_put(PIN_LED, 1);  // solid = fatal
    } else {
        printf("[OK] SROM UPLOAD SUCCESS\n");
    }
}

// =============================================================================
// Sensor initialization
// =============================================================================

void pmw_init(void) {
    // Hardware reset pulse (active low, 50ms each phase)
    gpio_put(PIN_RST, 0); sleep_ms(50);
    gpio_put(PIN_RST, 1); sleep_ms(50);

    // SPI reset sequence: toggle CS to reset the SPI state machine
    gpio_put(PIN_CS, 1); sleep_us(40);
    gpio_put(PIN_CS, 0); sleep_us(40);
    gpio_put(PIN_CS, 1); sleep_us(40);

    // Software power-up reset (datasheet section 8.1, step 2)
    pmw_write_reg(0x3A, 0x5A);  // REG_POWER_UP_RESET = 0x5A
    sleep_ms(50);               // mandatory 50ms post-reset wait

    // Clear motion registers (step 3: read and discard 0x02-0x06)
    (void)pmw_read_reg(REG_MOTION);
    (void)pmw_read_reg(REG_DELTA_X_L);
    (void)pmw_read_reg(REG_DELTA_X_H);
    (void)pmw_read_reg(REG_DELTA_Y_L);
    (void)pmw_read_reg(REG_DELTA_Y_H);

    // Upload SROM firmware (step 4)
    pmw_upload_srom();
    sleep_ms(10);

    // Set CPI (800 CPI default)
    pmw_write_reg(REG_CONFIG1, CPI_800);

    // Verify Config2 is still 0x00 (rest mode disabled) after full init
    uint8_t cfg2 = pmw_read_reg(REG_CONFIG2);
    if (cfg2 != 0x00) {
        printf("[WARN] Config2 = 0x%02X after init — forcing to 0x00\n", cfg2);
        sleep_ms(10);
        pmw_write_reg(REG_CONFIG2, 0x00);
        sleep_ms(10);
    } else {
        printf("[OK] Config2 = 0x00 (rest mode disabled)\n");
    }

    // Clear motion registers again post-SROM (second drain per QMK sequence)
    (void)pmw_read_reg(REG_MOTION);
    (void)pmw_read_reg(REG_DELTA_X_L);
    (void)pmw_read_reg(REG_DELTA_X_H);
    (void)pmw_read_reg(REG_DELTA_Y_L);
    (void)pmw_read_reg(REG_DELTA_Y_H);

    pmw_write_reg(REG_CONFIG1, CPI_800);
    sleep_ms(10);

    // Print startup diagnostic
    printf("[OK] PMW3389 init complete\n");
    printf("     Product ID : 0x%02X (expect 0x47)\n", pmw_read_reg(REG_PRODUCT_ID));
    printf("     Revision ID: 0x%02X (expect 0xB8)\n", pmw_read_reg(REG_REVISION_ID));
    printf("     SROM ID    : 0x%02X (expect 0xE8)\n", pmw_read_reg(REG_SROM_ID));
    printf("     Config1    : 0x%02X (expect 0x0F for 800 CPI)\n", pmw_read_reg(REG_CONFIG1));
    printf("     Config2    : 0x%02X (expect 0x00)\n", pmw_read_reg(REG_CONFIG2));
    printf("     SQUAL      : 0x%02X (>0x80 = good surface)\n", pmw_read_reg(REG_SQUAL));

    // Single short blink = init reached completion without hanging
    //cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(100);
    //cyw43_arch_gpio_put(PIN_LED, 0);
}

// =============================================================================
// HID task — 1ms polling loop
// =============================================================================

void hid_task(void) {
    const uint32_t interval_ms = 1;
    static uint32_t start_ms      = 0;
    static uint32_t last_debug_ms = 0;
    static uint8_t  prev_buttons  = 0;

    if (board_millis() - start_ms < interval_ms) return;
    start_ms += interval_ms;

    // -------------------------------------------------------------------------
    // Read buttons
    // -------------------------------------------------------------------------
    uint8_t current_buttons = 0;
    if (!gpio_get(BTN_LEFT))  current_buttons |= MOUSE_BUTTON_LEFT;
    if (!gpio_get(BTN_RIGHT)) current_buttons |= MOUSE_BUTTON_RIGHT;

    bool is_right_clicked = (current_buttons & MOUSE_BUTTON_RIGHT);

    // -------------------------------------------------------------------------
    // Burst motion read
    // Full CS assert → command → tSRAD_MOTBR → 12 bytes → CS deassert cycle.
    // Must be a single uninterrupted transaction per burst.
    // -------------------------------------------------------------------------
    uint8_t buf[BURST_BYTES] = {0};
    uint8_t zeros[BURST_BYTES] = {0};
    uint8_t cmd = REG_BURST_MOTION;
    uint8_t garbage;

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);  // tNCS-SCLK

    spi_write_read_blocking(SPI_PORT, &cmd, &garbage, 1);
    sleep_us(35);  // tSRAD_MOTBR: 35us min — was 50us, tightened to spec

    spi_write_read_blocking(SPI_PORT, zeros, buf, BURST_BYTES);

    gpio_put(PIN_CS, 1);
    sleep_us(20);  // tBEXIT: 500ns min — using 20us for safe margin

    // -------------------------------------------------------------------------
    // Parse burst data
    // Only emit deltas when:
    //   - MOT bit is set  (motion actually detected)
    //   - LIFT bit clear  (sensor is not lifted / SQUAL gate not tripped)
    // -------------------------------------------------------------------------
    int8_t report_x = 0;
    int8_t report_y = 0;

    uint8_t motion = buf[BURST_IDX_MOTION];
    bool has_motion = (motion & MOT_BIT_MOTION) && !(motion & MOT_BIT_LIFT);

    if (has_motion) {
        int16_t dx = (int16_t)((buf[BURST_IDX_DX_H] << 8) | buf[BURST_IDX_DX_L]);
        int16_t dy = (int16_t)((buf[BURST_IDX_DY_H] << 8) | buf[BURST_IDX_DY_L]);

        // Clamp to int8 range for HID mouse report
        report_x = (int8_t)(dx >  127 ?  127 : dx < -127 ? -127 : dx);
        report_y = (int8_t)(dy >  127 ?  127 : dy < -127 ? -127 : dy);
    }

    // -------------------------------------------------------------------------
    // Send HID report
    // -------------------------------------------------------------------------
    if (tud_hid_ready()) {
        bool buttons_changed = (current_buttons != prev_buttons);
        if (current_buttons || report_x || report_y || buttons_changed) {
            tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons,
                                 report_x, -report_y, 0, 0);
        }
    }
    prev_buttons = current_buttons;

    // -------------------------------------------------------------------------
    // Debug output — right click held, throttled to 100ms
    // Prints the full 12-byte burst so nothing is hidden.
    // -------------------------------------------------------------------------
    if (is_right_clicked && (board_millis() - last_debug_ms > 100)) {
        last_debug_ms = board_millis();

        uint8_t obs  = buf[BURST_IDX_OBS];
        uint8_t squal = buf[BURST_IDX_SQUAL];
        uint8_t rawsum = buf[BURST_IDX_RAWSUM];
        uint8_t maxraw = buf[BURST_IDX_MAXRAW];
        uint8_t minraw = buf[BURST_IDX_MINRAW];

        int16_t dx_full = (int16_t)((buf[BURST_IDX_DX_H] << 8) | buf[BURST_IDX_DX_L]);
        int16_t dy_full = (int16_t)((buf[BURST_IDX_DY_H] << 8) | buf[BURST_IDX_DY_L]);

        // Surface quality health indicator
        const char *squal_health = (squal >= 0x80) ? "GOOD" :
                                   (squal >= 0x40) ? "MARGINAL" : "BAD";

        // Lift status
        const char *lift_str = (motion & MOT_BIT_LIFT) ? "LIFTED" : "ON_SURFACE";

        printf("MOT:%02X[%s] OBS:%02X | dX:%5d dY:%5d | "
               "SQUAL:%02X(%s) Sum:%02X Max:%02X Min:%02X | Config2:%02X\n",
               motion, lift_str, obs,
               dx_full, dy_full,
               squal, squal_health, rawsum, maxraw, minraw,
               pmw_read_reg(REG_CONFIG2));  // Config2 sampled once per debug tick only

        // Warn if Observation mode was entered unexpectedly
        if (obs != 0x00) {
            printf("[WARN] Observation register non-zero: 0x%02X — sensor may be in obs mode\n", obs);
        }
    }
}

// =============================================================================
// Entry point
// =============================================================================

int main(void) {
    board_init();
    tusb_init();
    stdio_init_all();

    while (!tud_cdc_connected()) {
    tud_task();   // <--- This is the magic missing piece!
    sleep_ms(1);  // Keep this short so USB responds quickly
}
    

    // GPIO: buttons (input, pull-up)
    gpio_init(BTN_LEFT);  gpio_set_dir(BTN_LEFT,  GPIO_IN); gpio_pull_up(BTN_LEFT);
    gpio_init(BTN_RIGHT); gpio_set_dir(BTN_RIGHT, GPIO_IN); gpio_pull_up(BTN_RIGHT);

    // GPIO: sensor control lines
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS,  GPIO_OUT); gpio_put(PIN_CS,  1);  // CS idle high
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);  // RST idle high
    gpio_init(PIN_MT);  gpio_set_dir(PIN_MT,  GPIO_IN);  gpio_pull_up(PIN_MT);  // motion int

    // SPI: MODE 3 (CPOL=1, CPHA=1) @ 2MHz — PMW3389 requirement
    spi_init(SPI_PORT, 2000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    pmw_init();

    while (1) {
        tud_task();
        hid_task();
    }
}

// =============================================================================
// TinyUSB HID callbacks (required stubs)
// =============================================================================

uint16_t tud_hid_get_report_cb(
    uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(
    uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}