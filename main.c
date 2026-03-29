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


uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx_addr = reg & 0x7F;
    uint8_t rx_garbage;
    uint8_t rx_data;
    uint8_t tx_dummy = 0x00;

    gpio_put(PIN_CS, 0);
    sleep_us(5); // tNCS-SCLK delay to let sensor wake up to CS drop
    
    // 1. Send address and actively catch the RX garbage byte
    spi_write_read_blocking(SPI_PORT, &tx_addr, &rx_garbage, 1);
    
    // 2. Wait the required tSRAD
    sleep_us(160);  

    // 3. Send dummy byte to clock in the actual register data
    spi_write_read_blocking(SPI_PORT, &tx_dummy, &rx_data, 1);

    gpio_put(PIN_CS, 1);
    sleep_us(20);   // tSRW/tSRR delay between commands
    
    return rx_data;
}

void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg | 0x80, data };
    gpio_put(PIN_CS, 0);
    sleep_us(1); // tNCS-SCLK delay
    
    spi_write_blocking(SPI_PORT, tx, 2);
    sleep_us(35);   // tSCLK-NCS delay 
    
    gpio_put(PIN_CS, 1);
    sleep_us(145);  // tSWW/tSWR delay between commands
}


// SROM Upload

void pmw_upload_srom() {
    spi_set_baudrate(SPI_PORT, 1000 * 1000);

    // Disable REST mode before uploading to prevent sleep interruptions
    pmw_write_reg(0x10, 0x20); 
    sleep_ms(10);

    pmw_write_reg(0x13, 0x1d);
    sleep_ms(10);

    pmw_write_reg(0x13, 0x18);
    sleep_ms(10);

    gpio_put(PIN_CS, 0);
    sleep_us(1);

    uint8_t burst_cmd = 0x62 | 0x80;
    spi_write_blocking(SPI_PORT, &burst_cmd, 1);
    sleep_us(15);

    for (int i = 0; i < firmware_length; i++) {
        spi_write_blocking(SPI_PORT, &srom_data[i], 1);
        sleep_us(15);
    }

    // Pull CS HIGH first to trigger the CRC calculation
    gpio_put(PIN_CS, 1); 
    sleep_us(200);      // CRC check window
    
    spi_set_baudrate(SPI_PORT, 2000 * 1000);

    // Arm the motion DSP immediately after upload, before anything else
    pmw_write_reg(0x10, 0x00); // Config2: Normal tracking
    sleep_ms(10);

    uint8_t srom_id = pmw_read_reg(0x2A);
    printf("\n>>> SROM ID READ: %02X <<<\n", srom_id);

    if (srom_id == 0x00) {
        // Only 0x00 is a true failure
        printf("SROM UPLOAD FAILED!\n");
        cyw43_arch_gpio_put(PIN_LED, 1);
        sleep_ms(3000);
        cyw43_arch_gpio_put(PIN_LED, 0);
    } else {
        // Any non-zero ID (including 0xE8) is success
        printf("SROM UPLOAD SUCCESS (ID: %02X)!\n", srom_id);
        for (int i = 0; i < 3; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(500);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        }
    }
}


// Sensor Init

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

    // 4. Clear power-up fault and prepare the SROM engine
    pmw_read_reg(0x02);
    pmw_read_reg(0x03);
    pmw_read_reg(0x04);
    pmw_read_reg(0x05);
    pmw_read_reg(0x06);

    // 5. Upload firmware
    pmw_upload_srom();
    sleep_ms(10); 

    // 6. Final Configuration
    sleep_ms(1);
    pmw_write_reg(0x0F, 0x10); // 800 DPI
    pmw_write_reg(0x63, 0x00); 

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

// Standard HID Task

void hid_task(void) {
    const uint32_t interval_ms = 2; // 500Hz polling
    static uint32_t start_ms = 0;
    static uint32_t last_debug_ms = 0;
    static uint8_t previous_buttons = 0;

    if (board_millis() - start_ms < interval_ms) return;
    start_ms += interval_ms;

    // 1. Read Buttons
    uint8_t current_buttons = 0;
    bool is_right_clicked = !gpio_get(BTN_RIGHT); 

    if (!gpio_get(BTN_LEFT))  current_buttons |= MOUSE_BUTTON_LEFT;
    if (is_right_clicked)     current_buttons |= MOUSE_BUTTON_RIGHT;

    // 2. Read Motion Burst (Only do this ONCE per cycle!)
    uint8_t buf[12] = {0};
    uint8_t cmd = 0x50; // Burst read command
    uint8_t garbage_rx;
    
    gpio_put(PIN_CS, 0);
    sleep_us(1); // tNCS-SCLK delay
    
    // Send burst command, actively catching and discarding the garbage byte
    spi_write_read_blocking(SPI_PORT, &cmd, &garbage_rx, 1);
    
    sleep_us(50);  // tSRAD_MOTBR wait
    
    // Read the actual 12 payload bytes
    spi_read_blocking(SPI_PORT, 0x00, buf, 12);
    
    gpio_put(PIN_CS, 1);
    sleep_us(2);  // tBEXIT conservative wait

    // 3. Parse Motion
    int8_t report_x = 0;
    int8_t report_y = 0;

    if (buf[0] & 0x80) { // Check the valid motion bit
        int16_t dx = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t dy = (int16_t)((buf[5] << 8) | buf[4]);
        // Clamp values to int8_t limits
        report_x = (int8_t)(dx > 127 ? 127 : dx < -127 ? -127 : dx);
        report_y = (int8_t)(dy > 127 ? 127 : dy < -127 ? -127 : dy);
    }

    // 4. Send USB HID Report
    if (tud_hid_ready()) {
        if (current_buttons || report_x || report_y || current_buttons != previous_buttons) {
            tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, report_x, -report_y, 0, 0);
        }
    }
    previous_buttons = current_buttons;

    // 5. Serial Debugging (Triggered by holding Right Click)
    if (is_right_clicked && (board_millis() - last_debug_ms > 100)) {
        uint8_t motion = pmw_read_reg(0x02);
        uint8_t xl = pmw_read_reg(0x03);
        uint8_t xh = pmw_read_reg(0x04);
        uint8_t yl = pmw_read_reg(0x05);
        uint8_t yh = pmw_read_reg(0x06);
        uint8_t SQUAL = pmw_read_reg(0x07);

        printf("DIRECT | MOT:%02X dX:%d dY:%d SQUAL:%02X\n",
        motion,
        (int16_t)((xh << 8) | xl),
        (int16_t)((yh << 8) | yl),
        SQUAL);
        
    last_debug_ms = board_millis();
    }
}

// Entry Point

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

    // SPI: Mode 3 (CPOL=1, CPHA=1) at 2MHz initially
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

// TinyUSB Callbacks

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