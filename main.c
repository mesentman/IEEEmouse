#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "srom3389.h"

// --- Pin Definitions ---
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

#define REPORT_ID_MOUSE 0

// --- SPI Helpers ---
uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &tx, 1);
    sleep_us(150);
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    gpio_put(PIN_CS, 1);
    sleep_us(200);
    return rx;
}

void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg | 0x80, data };
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx, 2);
    gpio_put(PIN_CS, 1);
    sleep_us(200);
}

// --- Motion Burst Read ---
void pmw_read_motion(int16_t *dx, int16_t *dy) {
    uint8_t buf[12] = {0};
    uint8_t cmd = 0x50; // Motion burst register
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    sleep_us(150);
    spi_read_blocking(SPI_PORT, 0x00, buf, 12);
    gpio_put(PIN_CS, 1);
    sleep_us(500);

    // buf[0] = Motion register
    // buf[2] = DX_L, buf[3] = DX_H
    // buf[4] = DY_L, buf[5] = DY_H
    if (buf[0] & 0x80) {
        cyw43_arch_gpio_put(PIN_LED, 1);
        *dx = (int16_t)((buf[3] << 8) | buf[2]);
        *dy = (int16_t)((buf[5] << 8) | buf[4]);
    } else {
        cyw43_arch_gpio_put(PIN_LED, 0);
        *dx = 0;
        *dy = 0;
    }
}

// --- SROM Upload ---
void pmw_upload_srom() {
    pmw_write_reg(0x10, 0x00); // Disable rest mode
    pmw_write_reg(0x13, 0x1d); // SROM_Enable
    sleep_ms(10);

    // Write 0x18 to SROM_Enable AND LATCH IT by letting CS go high
    pmw_write_reg(0x13, 0x18);
    sleep_us(15); // Tiny buffer before burst

    // Now pull CS low to begin the SROM burst
    gpio_put(PIN_CS, 0);
    uint8_t burst_cmd = 0x62 | 0x80;
    spi_write_blocking(SPI_PORT, &burst_cmd, 1);
    sleep_us(15);

    // Send the firmware
    for (int i = 0; i < 4094; i++) {
        spi_write_blocking(SPI_PORT, &srom_data[i], 1);
        sleep_us(15);
    }

    // End the burst
    gpio_put(PIN_CS, 1);
    sleep_ms(1);

    uint8_t srom_id = pmw_read_reg(0x2A);
    cyw43_arch_gpio_put(PIN_LED, 0);
    sleep_ms(500);

    if (srom_id != 0xE8) {
        // FAIL: 1 long 3-second flash
        cyw43_arch_gpio_put(PIN_LED, 1);
        sleep_ms(3000);
        cyw43_arch_gpio_put(PIN_LED, 0);
    } else {
        // SUCCESS: 3 slow flashes
        for (int i = 0; i < 3; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(500);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        }
    }
}

// --- Sensor Init ---
void pmw_init() {
    // Tripwire: solid LED means init started
    cyw43_arch_gpio_put(PIN_LED, 1);
    sleep_ms(1000);

    // Hardware reset
    gpio_put(PIN_RST, 0);
    sleep_ms(50);
    gpio_put(PIN_RST, 1);
    sleep_ms(50);

    uint8_t product_id = pmw_read_reg(0x00);

    if (product_id != 0x00 && product_id != 0xFF) {
        // Power-up reset
        pmw_write_reg(0x3A, 0x5A);
        sleep_ms(50);

        // Clear startup garbage
        pmw_read_reg(0x02);
        pmw_read_reg(0x03);
        pmw_read_reg(0x04);
        pmw_read_reg(0x05);
        pmw_read_reg(0x06);

        // Upload firmware
        pmw_upload_srom();

        // Configure sensor
        pmw_write_reg(0x10, 0x00); // Disable rest mode
        sleep_ms(10);
        pmw_write_reg(0x50, 0x00); // Set motion burst mode
        sleep_ms(10);
        pmw_write_reg(0x0F, 0x10); // 800 DPI (value * 50 = DPI)

        // Clear motion registers after init
        pmw_read_reg(0x02);
        pmw_read_reg(0x03);
        pmw_read_reg(0x04);
        pmw_read_reg(0x05);
        pmw_read_reg(0x06);

        // 5 fast flashes = overall success
        cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        for (int i = 0; i < 5; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(200);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(200);
        }
    } else {
        // 10 rapid blips = SPI failure
        for (int i = 0; i < 10; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(50);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(50);
        }
    }

    cyw43_arch_gpio_put(PIN_LED, 0);
}

// --- HID Task ---
void hid_task(void);

int main(void) {
    board_init();
    tusb_init();
    cyw43_arch_init();

    // Buttons
    gpio_init(BTN_LEFT);  gpio_set_dir(BTN_LEFT, GPIO_IN);  gpio_pull_up(BTN_LEFT);
    gpio_init(BTN_RIGHT); gpio_set_dir(BTN_RIGHT, GPIO_IN); gpio_pull_up(BTN_RIGHT);

    // Sensor pins
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);  gpio_put(PIN_CS, 1);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);
    gpio_init(PIN_MT);  gpio_set_dir(PIN_MT, GPIO_IN);   gpio_pull_up(PIN_MT);

    // SPI (Mode 3: CPOL=1, CPHA=1 at 2MHz)
    spi_init(SPI_PORT, 2000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    pmw_init();
    stdio_init_all();

    while (1) {
        tud_task();
        hid_task();
    }
}

/*void hid_task(void) {
    const uint32_t interval_ms = 2;
    static uint32_t start_ms = 0;
    static uint8_t previous_buttons = 0;

    if (board_millis() - start_ms < interval_ms) return;
    start_ms += interval_ms;
    if (!tud_hid_ready()) return;

    uint8_t current_buttons = 0;
    if (!gpio_get(BTN_LEFT))  current_buttons |= MOUSE_BUTTON_LEFT;
    if (!gpio_get(BTN_RIGHT)) current_buttons |= MOUSE_BUTTON_RIGHT;

    uint8_t buf[12] = {0};
    uint8_t cmd = 0x50;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    sleep_us(150);
    spi_read_blocking(SPI_PORT, 0x00, buf, 12);
    gpio_put(PIN_CS, 1);
    sleep_us(200);

    if (buf[0] & 0x80) {
        int16_t dx = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t dy = (int16_t)((buf[5] << 8) | buf[4]);
        int8_t report_x = dx > 127 ? 127 : (dx < -127 ? -127 : (int8_t)dx);
        int8_t report_y = dy > 127 ? 127 : (dy < -127 ? -127 : (int8_t)dy);
        tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, report_x, -report_y, 0, 0);
    }

    previous_buttons = current_buttons;
}
*/ 
void hid_task(void) {
    const uint32_t interval_ms = 2000;
    static uint32_t start_ms = 0;
    if (board_millis() - start_ms < interval_ms) return;
    start_ms += interval_ms;

    uint8_t buf[12] = {0};
    uint8_t cmd = 0x50;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    sleep_us(150);
    spi_read_blocking(SPI_PORT, 0x00, buf, 12);
    gpio_put(PIN_CS, 1);
    sleep_us(500);

    // Flash once per bit that is set in buf[0]
    for (int bit = 7; bit >= 0; bit--) {
        if (buf[0] & (1 << bit)) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(300);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(300);
        } else {
            sleep_ms(600); // gap for zero bit
        }
    }
    sleep_ms(1000); // pause between readings
}

// --- TinyUSB Callbacks ---
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) bufsize;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) reqlen;
    return 0;
}