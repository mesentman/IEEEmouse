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
#define PIN_LED      CYW43_WL_GPIO_LED_PIN
#define BTN_LEFT     19
#define BTN_RIGHT    14
#define SPI_PORT     spi0
#define PIN_SCK      2
#define PIN_MOSI     3
#define PIN_MISO     4
#define PIN_CS       5
#define PIN_MT       6
#define PIN_RST      7

#define REPORT_ID_MOUSE 0

// ---------------------------------------------------------------------------
// SPI Helpers (Patched with QMK Strict Timings)
// ---------------------------------------------------------------------------

uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    gpio_put(PIN_CS, 0);
    sleep_us(1); // FIX: tNCS-SCLK delay to let sensor wake up to CS drop
    
    spi_write_blocking(SPI_PORT, &tx, 1);
    sleep_us(160);  // FIX: tSRAD minimum wait before reading
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    
    gpio_put(PIN_CS, 1);
    sleep_us(20);   // FIX: tSRW/tSRR delay between commands
    return rx;
}

void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg | 0x80, data };
    gpio_put(PIN_CS, 0);
    sleep_us(1); // FIX: tNCS-SCLK delay
    
    spi_write_blocking(SPI_PORT, tx, 2);
    sleep_us(35);   // FIX: tSCLK-NCS delay 
    
    gpio_put(PIN_CS, 1);
    sleep_us(145);  // FIX: tSWW/tSWR delay between commands
}

// ---------------------------------------------------------------------------
// SROM Upload (Patched Upload Sequence)
// ---------------------------------------------------------------------------

void pmw_upload_srom() {
    pmw_write_reg(0x10, 0x00); // Disable rest mode
    pmw_write_reg(0x13, 0x1d); // SROM_Enable: set download-enable bit
    sleep_ms(10);

    pmw_write_reg(0x13, 0x18); // SROM_Enable: latch start-download bit
    sleep_ms(10);

    gpio_put(PIN_CS, 0);
    sleep_us(1); // tNCS-SCLK

    uint8_t burst_cmd = 0x62 | 0x80; // Write to SROM_Load_Burst (0x62)
    spi_write_blocking(SPI_PORT, &burst_cmd, 1);
    sleep_us(15);

    // Blast the firmware array
    for (int i = 0; i < firmware_length; i++) {
        spi_write_blocking(SPI_PORT, &srom_data[i], 1);
        sleep_us(15);
    }
    sleep_us(35);
    gpio_put(PIN_CS, 1); 
    sleep_ms(10);

    // Verify upload succeeded by reading back SROM_ID
    uint8_t srom_id = pmw_read_reg(0x2A);
    printf("\n>>> SROM ID READ: %02X <<<\n", srom_id);

    cyw43_arch_gpio_put(PIN_LED, 0);
    sleep_ms(500);

    if (srom_id == 0x00) {
        // FAIL: Upload failed. 1 long 3-second flash
        printf("SROM UPLOAD FAILED!\n");
        cyw43_arch_gpio_put(PIN_LED, 1);
        sleep_ms(3000);
        cyw43_arch_gpio_put(PIN_LED, 0);
    } else {
        // SUCCESS: 3 slow flashes
        printf("SROM UPLOAD SUCCESS (ID: %02X)!\n", srom_id);
        for (int i = 0; i < 3; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(500);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        }
    }
}

// ---------------------------------------------------------------------------
// Sensor Init (Patched Shutdown/Wakeup Dance)
// ---------------------------------------------------------------------------

void pmw_init() {
    cyw43_arch_gpio_put(PIN_LED, 1);
    sleep_ms(1000);

    // 1. Hardware reset
    gpio_put(PIN_RST, 0); sleep_ms(50);
    gpio_put(PIN_RST, 1); sleep_ms(50);

    // 2. SPI Sync & Shutdown Dance
    gpio_put(PIN_CS, 1); sleep_us(40);
    gpio_put(PIN_CS, 0); sleep_us(40);
    gpio_put(PIN_CS, 1); sleep_us(40);
    
    pmw_write_reg(0x3B, 0xB6); // Shutdown
    sleep_ms(300);             // Let it power down
    
    gpio_put(PIN_CS, 0); sleep_us(40); // Sync SPI again
    gpio_put(PIN_CS, 1); sleep_us(40);
    
    // 3. Power-up reset
    pmw_write_reg(0x3A, 0x5A); 
    sleep_ms(50);              // Wait for boot (tRESET)

    // 4. CRITICAL: Read 0x02 to check the "Observation" bits
    // This clears the power-up fault and prepares the SROM engine.
    uint8_t obs = pmw_read_reg(0x02);
    
    // Clear the rest of the status registers
    pmw_read_reg(0x03);
    pmw_read_reg(0x04);
    pmw_read_reg(0x05);
    pmw_read_reg(0x06);

    // 5. Upload firmware
    // Note: Make sure you added the 35us delay at the end of the 
    // for loop in your pmw_upload_srom() as we discussed!
    pmw_upload_srom();
    sleep_ms(10); 

    // 6. Final Configuration
    pmw_write_reg(0x10, 0x00); // Disable rest mode
    sleep_ms(1);
    pmw_write_reg(0x0F, 0x10); // 800 DPI

    // 7. Clear motion registers one last time to start fresh
    pmw_read_reg(0x02);
    pmw_read_reg(0x03);
    pmw_read_reg(0x04);
    pmw_read_reg(0x05);
    pmw_read_reg(0x06);

    // Success Flashes
    cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
    for (int i = 0; i < 5; i++) {
        cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(200);
        cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(200);
    }
}

// ---------------------------------------------------------------------------
// Standard HID Task
// ---------------------------------------------------------------------------

void hid_task(void) {
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
    sleep_us(1); // tNCS-SCLK delay
    spi_write_blocking(SPI_PORT, &cmd, 1);
    sleep_us(35);  // tSRAD_MOTBR wait
    spi_read_blocking(SPI_PORT, 0x00, buf, 12);
    gpio_put(PIN_CS, 1);
    sleep_us(1);  // tBEXIT conservative wait

    int8_t report_x = 0;
    int8_t report_y = 0;

    if (buf[0] & 0x80) {
        int16_t dx = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t dy = (int16_t)((buf[5] << 8) | buf[4]);
        report_x = (int8_t)(dx > 127 ? 127 : dx < -127 ? -127 : dx);
        report_y = (int8_t)(dy > 127 ? 127 : dy < -127 ? -127 : dy);
    }

    if (current_buttons || report_x || report_y || current_buttons != previous_buttons) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, report_x, -report_y, 0, 0);
    }

    previous_buttons = current_buttons;
}

// ---------------------------------------------------------------------------
// Entry Point
// ---------------------------------------------------------------------------

int main(void) {
    board_init();
    tusb_init();
    cyw43_arch_init();
    stdio_init_all(); 

    // Button inputs
    gpio_init(BTN_LEFT);  gpio_set_dir(BTN_LEFT, GPIO_IN);  gpio_pull_up(BTN_LEFT);
    gpio_init(BTN_RIGHT); gpio_set_dir(BTN_RIGHT, GPIO_IN); gpio_pull_up(BTN_RIGHT);

    // Sensor control pins
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT); gpio_put(PIN_CS, 1);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);
    gpio_init(PIN_MT);  gpio_set_dir(PIN_MT, GPIO_IN);  gpio_pull_up(PIN_MT);

    // SPI: Mode 3 (CPOL=1, CPHA=1) at 2MHz
    spi_init(SPI_PORT, 2000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    pmw_init();

    while (1) {
        tud_task();
        
        bool is_right_clicked = !gpio_get(BTN_RIGHT); 

        if (is_right_clicked) {
            // RIGHT CLICK DEBUG TESTER
            uint8_t buf[12] = {0};
            uint8_t cmd = 0x50; // Burst read command
            
            gpio_put(PIN_CS, 0);
            sleep_us(1); // tNCS-SCLK delay
            spi_write_blocking(SPI_PORT, &cmd, 1);
            sleep_us(35); // tSRAD_MOTBR wait
            spi_read_blocking(SPI_PORT, 0x00, buf, 12); 
            gpio_put(PIN_CS, 1);
            sleep_us(1); // tBEXIT wait
            
            printf("RAW BURST | 0:%02X  1:%02X  2:%02X  3:%02X  4:%02X  5:%02X  SQ:%d\n", 
                   buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
            
            sleep_ms(100); // Throttle debug output
        } else {
            // Only run normal HID tasks if we aren't holding right click to debug
            hid_task();
        }
    }
}

// ---------------------------------------------------------------------------
// TinyUSB Callbacks
// ---------------------------------------------------------------------------

void tud_hid_set_report_cb(
    uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}

uint16_t tud_hid_get_report_cb(
    uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}