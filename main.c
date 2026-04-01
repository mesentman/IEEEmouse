// =============================================================================
// PMW3389 Optical Sensor Driver — RP2040 / Pico W
// =============================================================================
// Hardware:  Raspberry Pi Pico W (RP2040)
// Sensor:    PixArt PMW3389
// Interface: SPI0 @ 2MHz (1MHz during SROM upload)
// Framework: Pico SDK + TinyUSB
//
// SROM Init sequence per datasheet Section 5.0:
//   1. Write 0x00 to Config2 BEFORE SROM upload (disable rest mode)
//   2. Upload SROM
//   3. Read SROM_ID FIRST — before any other register access
//   4. Write 0x00 to Config2 again (wired mouse config)
//
// NOTE: OBS register = 0x7F is NORMAL.
//   Bit 6 = SROM_RUN (1 = SROM running = good)
//   Bits 5:0 = set once per frame (all set = frames actively captured = good)
//   Do NOT attempt to clear or warn on OBS = 0x7F.
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
#define PIN_MT      6
#define PIN_RST     7

// -----------------------------------------------------------------------------
// PMW3389 register addresses
// -----------------------------------------------------------------------------
#define REG_PRODUCT_ID      0x00
#define REG_REVISION_ID     0x01
#define REG_MOTION          0x02
#define REG_DELTA_X_L       0x03
#define REG_DELTA_X_H       0x04
#define REG_DELTA_Y_L       0x05
#define REG_DELTA_Y_H       0x06
#define REG_SQUAL           0x07
#define REG_RAW_DATA_SUM    0x08
#define REG_MAX_RAW_DATA    0x09
#define REG_MIN_RAW_DATA    0x0A
#define REG_SHUTTER_L       0x0B
#define REG_SHUTTER_H       0x0C
#define REG_CONTROL         0x0D
#define REG_CONFIG1         0x0F
#define REG_CONFIG2         0x10
#define REG_SROM_ENABLE     0x13
#define REG_OBSERVATION     0x24  
#define REG_SROM_ID         0x2A
#define REG_BURST_MOTION    0x50
#define REG_BURST_SROM      0x62

// Motion register bit masks
#define MOT_BIT_MOTION      (1 << 7)  // Bit 7: motion detected
#define MOT_BIT_LIFT        (1 << 3)  // Bit 3: sensor lifted (SQUAL too low)

// CPI: formula = (val + 1) * 50. 0x0F = 800 CPI
#define CPI_800             0x0F
#define CPI_1600            0x1F
#define CPI_3200            0x3F

#define SROM_EXPECTED_ID    0xE8
#define REPORT_ID_MOUSE     0

// -----------------------------------------------------------------------------
// Burst byte layout (12 bytes)
// -----------------------------------------------------------------------------
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

static inline void pmw_spi_flush(void) {
    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }
}

// tSRAD = 160us, tSRR/tSRW = 20us
uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx_addr = reg & 0x7F;
    uint8_t rx_garbage, rx_data;
    uint8_t tx_dummy = 0x00;

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);

    spi_write_read_blocking(SPI_PORT, &tx_addr, &rx_garbage, 1);
    sleep_us(160);  // tSRAD

    spi_write_read_blocking(SPI_PORT, &tx_dummy, &rx_data, 1);
    sleep_us(1);

    gpio_put(PIN_CS, 1);
    sleep_us(20);   // tSRR

    return rx_data;
}

// tSCLK-NCS-write = 35us, tSWW/tSWR = 145us after CS high (180us total = datasheet min)
void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), data };
    uint8_t rx[2];

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);

    spi_write_read_blocking(SPI_PORT, tx, rx, 2);

    sleep_us(35);
    gpio_put(PIN_CS, 1);
    sleep_us(145);
}

// =============================================================================
// SROM firmware upload
// Per datasheet Section 5.0 — order is critical
// =============================================================================

void pmw_upload_srom(void) {
    // -------------------------------------------------------------------------
    // STEP 1 (Datasheet Section 5.0, step 2):
    // Write 0x00 to Config2 BEFORE starting SROM upload to disable rest mode.
    // This is a datasheet requirement — rest mode must be off before upload.
    // -------------------------------------------------------------------------
    pmw_write_reg(REG_CONFIG2, 0x00);
    sleep_ms(10);
    printf("[SROM] Config2 pre-upload: 0x%02X (should be 0x00)\n",
           pmw_read_reg(REG_CONFIG2));
    stdio_flush();

    // Drop to 1MHz for SROM upload
    spi_set_baudrate(SPI_PORT, 1000 * 1000);

    // SROM enable sequence (datasheet steps 3-5)
    pmw_write_reg(REG_SROM_ENABLE, 0x1D);
    sleep_ms(10);
    pmw_write_reg(REG_SROM_ENABLE, 0x18);
    sleep_ms(10);

    // Begin SROM burst write
    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);

    uint8_t burst_cmd = REG_BURST_SROM | 0x80;
    uint8_t rx_garbage;
    spi_write_read_blocking(SPI_PORT, &burst_cmd, &rx_garbage, 1);
    sleep_us(15);

    // Clock out SROM array — service USB every 64 bytes to keep CDC alive
    for (int i = 0; i < firmware_length; i++) {
        spi_write_read_blocking(SPI_PORT, &srom_data[i], &rx_garbage, 1);
        sleep_us(15);
        if (i % 64 == 0) tud_task();
    }

    gpio_put(PIN_CS, 1);
    // Datasheet Figure 22: minimum 200us after CS high before reading SROM_ID
    sleep_us(200);

    // Restore operational baud rate
    spi_set_baudrate(SPI_PORT, 2000 * 1000);

    // -------------------------------------------------------------------------
    // STEP 2 (Datasheet Section 5.0, step 7):
    // Read SROM_ID FIRST — before any other register reads or writes.
    // This is explicitly required by the datasheet.
    // -------------------------------------------------------------------------
    uint8_t srom_id = pmw_read_reg(REG_SROM_ID);
    printf("[SROM] SROM_ID = 0x%02X (expected 0x%02X)\n", srom_id, SROM_EXPECTED_ID);
    stdio_flush();

    if (srom_id != SROM_EXPECTED_ID) {
        printf("[ERR] SROM UPLOAD FAILED — sensor will not track\n");
        stdio_flush();
        cyw43_arch_gpio_put(PIN_LED, 1);  // solid = fatal
        return;
    }
    printf("[OK] SROM UPLOAD SUCCESS\n");
    stdio_flush();

    // -------------------------------------------------------------------------
    // STEP 3 (Datasheet Section 5.0, step 8):
    // Write 0x00 to Config2 AFTER reading SROM_ID — wired mouse configuration.
    // Must come after SROM_ID read, not before.
    // -------------------------------------------------------------------------
    pmw_write_reg(REG_CONFIG2, 0x00);
    sleep_ms(10);
    uint8_t cfg2 = pmw_read_reg(REG_CONFIG2);
    printf("[SROM] Config2 post-upload: 0x%02X (should be 0x00)\n", cfg2);
    stdio_flush();

    if (cfg2 != 0x00) {
        printf("[WARN] Config2 did not stick — rest mode may be active\n");
        stdio_flush();
    }
}

// =============================================================================
// Sensor initialization
// Per datasheet Section 7.0 Power Up sequence
// =============================================================================

void pmw_init(void) {
    // Hardware reset (active low, 50ms each phase)
    gpio_put(PIN_RST, 0); sleep_ms(50);
    gpio_put(PIN_RST, 1); sleep_ms(50);

    // SPI port reset — drive NCS high then low (datasheet Section 7.0 step 2)
    gpio_put(PIN_CS, 1); sleep_us(40);
    gpio_put(PIN_CS, 0); sleep_us(40);
    gpio_put(PIN_CS, 1); sleep_us(40);

    // Software power-up reset (datasheet step 3)
    pmw_write_reg(0x3A, 0x5A);
    sleep_ms(50);  // mandatory 50ms (datasheet step 4)

    // Read and discard motion registers (datasheet step 5)
    (void)pmw_read_reg(REG_MOTION);
    (void)pmw_read_reg(REG_DELTA_X_L);
    (void)pmw_read_reg(REG_DELTA_X_H);
    (void)pmw_read_reg(REG_DELTA_Y_L);
    (void)pmw_read_reg(REG_DELTA_Y_H);

    // SROM upload (datasheet step 6)
    pmw_upload_srom();
    sleep_ms(10);

    // Set CPI (datasheet step 7 — load configuration)
    pmw_write_reg(REG_CONFIG1, CPI_800);
    sleep_ms(10);

    // Second motion register drain post-SROM (QMK practice)
    (void)pmw_read_reg(REG_MOTION);
    (void)pmw_read_reg(REG_DELTA_X_L);
    (void)pmw_read_reg(REG_DELTA_X_H);
    (void)pmw_read_reg(REG_DELTA_Y_L);
    (void)pmw_read_reg(REG_DELTA_Y_H);

    // Startup diagnostic
    printf("[OK] PMW3389 init complete\n");
    printf("     Product ID : 0x%02X (expect 0x47)\n", pmw_read_reg(REG_PRODUCT_ID));
    printf("     Revision ID: 0x%02X (expect 0xB8)\n", pmw_read_reg(REG_REVISION_ID));
    printf("     SROM ID    : 0x%02X (expect 0xE8)\n", pmw_read_reg(REG_SROM_ID));
    printf("     Config1    : 0x%02X (expect 0x0F = 800 CPI)\n", pmw_read_reg(REG_CONFIG1));
    printf("     Config2    : 0x%02X (expect 0x00)\n", pmw_read_reg(REG_CONFIG2));
    printf("     OBS        : 0x%02X (0x7F = normal: SROM running + frames active)\n",
           pmw_read_reg(REG_OBSERVATION));
    printf("     SQUAL      : 0x%02X (>0x80 = good surface)\n", pmw_read_reg(REG_SQUAL));
    stdio_flush();

    // Single short blink = init complete
    cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(100);
    cyw43_arch_gpio_put(PIN_LED, 0);
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

    uint8_t current_buttons = 0;
    if (!gpio_get(BTN_LEFT))  current_buttons |= MOUSE_BUTTON_LEFT;
    if (!gpio_get(BTN_RIGHT)) current_buttons |= MOUSE_BUTTON_RIGHT;
    bool is_right_clicked = (current_buttons & MOUSE_BUTTON_RIGHT);

    // Burst motion read
    uint8_t buf[BURST_BYTES] = {0};
    uint8_t zeros[BURST_BYTES] = {0};
    uint8_t cmd = REG_BURST_MOTION;
    uint8_t garbage;

    pmw_spi_flush();
    gpio_put(PIN_CS, 0);
    sleep_us(1);
    spi_write_read_blocking(SPI_PORT, &cmd, &garbage, 1);
    sleep_us(35);   // tSRAD_MOTBR
    spi_write_read_blocking(SPI_PORT, zeros, buf, BURST_BYTES);
    gpio_put(PIN_CS, 1);
    sleep_us(20);   // tBEXIT

    // Parse — only emit deltas when MOT set AND LIFT clear
    int8_t report_x = 0;
    int8_t report_y = 0;
    uint8_t motion = buf[BURST_IDX_MOTION];
    bool has_motion = (motion & MOT_BIT_MOTION) && !(motion & MOT_BIT_LIFT);

    if (has_motion) {
        int16_t dx = (int16_t)((buf[BURST_IDX_DX_H] << 8) | buf[BURST_IDX_DX_L]);
        int16_t dy = (int16_t)((buf[BURST_IDX_DY_H] << 8) | buf[BURST_IDX_DY_L]);
        report_x = (int8_t)(dx >  127 ?  127 : dx < -127 ? -127 : dx);
        report_y = (int8_t)(dy >  127 ?  127 : dy < -127 ? -127 : dy);
    }

    // HID report
    if (tud_hid_ready()) {
        bool buttons_changed = (current_buttons != prev_buttons);
        if (current_buttons || report_x || report_y || buttons_changed) {
            tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons,
                                 report_x, -report_y, 0, 0);
        }
    }
    prev_buttons = current_buttons;

    // Debug — right click held, 100ms throttle
    if (is_right_clicked && (board_millis() - last_debug_ms > 100)) {
        last_debug_ms = board_millis();

        uint8_t squal  = buf[BURST_IDX_SQUAL];
        uint8_t rawsum = buf[BURST_IDX_RAWSUM];
        uint8_t maxraw = buf[BURST_IDX_MAXRAW];
        uint8_t minraw = buf[BURST_IDX_MINRAW];
        int16_t dx_full = (int16_t)((buf[BURST_IDX_DX_H] << 8) | buf[BURST_IDX_DX_L]);
        int16_t dy_full = (int16_t)((buf[BURST_IDX_DY_H] << 8) | buf[BURST_IDX_DY_L]);

        const char *squal_health = (squal >= 0x80) ? "GOOD" :
                                   (squal >= 0x40) ? "MARGINAL" : "BAD";
        const char *lift_str = (motion & MOT_BIT_LIFT) ? "LIFTED" : "ON_SURFACE";

        // OBS: bit6=SROM_RUN, bits5:0=frame activity — 0x7F is normal
        uint8_t obs = buf[BURST_IDX_OBS];
        const char *srom_run = (obs & 0x40) ? "SROM_RUN:YES" : "SROM_RUN:NO(!)";

        printf("MOT:%02X[%s] %s | dX:%5d dY:%5d | "
               "SQUAL:%02X(%s) Sum:%02X Max:%02X Min:%02X | CFG2:%02X\n",
               motion, lift_str, srom_run,
               dx_full, dy_full,
               squal, squal_health, rawsum, maxraw, minraw,
               pmw_read_reg(REG_CONFIG2));
    }
}

// =============================================================================
// Entry point
// =============================================================================

int main(void) {
    board_init();
    tusb_init();
    cyw43_arch_init();
    stdio_init_all();

    // Wait for USB CDC then give 5s to open serial monitor
    while (!tud_cdc_connected()) {
        tud_task();
        sleep_ms(1);
    }
    sleep_ms(5000);

    gpio_init(BTN_LEFT);  gpio_set_dir(BTN_LEFT,  GPIO_IN); gpio_pull_up(BTN_LEFT);
    gpio_init(BTN_RIGHT); gpio_set_dir(BTN_RIGHT, GPIO_IN); gpio_pull_up(BTN_RIGHT);

    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS,  GPIO_OUT); gpio_put(PIN_CS,  1);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);
    gpio_init(PIN_MT);  gpio_set_dir(PIN_MT,  GPIO_IN);  gpio_pull_up(PIN_MT);

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
// TinyUSB HID callbacks
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