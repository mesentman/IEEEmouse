#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" // For Pico W onboard LED
#include "hardware/spi.h"
#include "srom3389.h" // The thing that actually makes the PMW3389 sensor work
// --- Pin Definitions ---
#define PIN_LED       CYW43_WL_GPIO_LED_PIN
#define BTN_LEFT      19
#define BTN_RIGHT     14

#define SPI_PORT      spi0
#define PIN_SCK       2
#define PIN_MOSI      3
#define PIN_MISO      4
#define PIN_CS        5
#define PIN_MT        6
#define PIN_RST       7

#define REPORT_ID_MOUSE 0

// --- PMW3389 Registers ---
#define REG_Motion    0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06

// --- SPI Helper Functions ---
/*uint8_t pmw_read_reg(uint8_t reg) {
    uint8_t tx = reg & 0x7F; // MSB must be 0 for Read
    uint8_t rx = 0;
    
    gpio_put(PIN_CS, 0); // Select the sensor
    spi_write_blocking(SPI_PORT, &tx, 1);
    
    sleep_us(150); // Required delay (tSRAD) between writing address and reading data
    
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1); // Send dummy byte to clock out data
    gpio_put(PIN_CS, 1); // Deselect the sensor
    
    sleep_us(200); 
    return rx;
}
*/
    uint8_t pmw_read_reg(uint8_t reg) {
    gpio_put(PIN_CS, 0); // Select the sensor
    
    // 1. Send the register address with the top bit set to 0 (which means "READ")
    uint8_t addr = reg & 0x7F; 
    spi_write_blocking(SPI_PORT, &addr, 1);
    
    // 2. CRITICAL PIXART DELAY: Give the sensor 2 microseconds to fetch the data!
    sleep_us(2); 
    
    // 3. Read the actual data (we send a dummy 0x00 byte to keep the clock ticking)
    uint8_t data = 0;
    spi_read_blocking(SPI_PORT, 0x00, &data, 1); 
    
    gpio_put(PIN_CS, 1); // Deselect the sensor
    
    // 4. ANOTHER CRITICAL DELAY: The sensor needs 20 microseconds to breathe before the next command
    sleep_us(20); 
    
    return data;
}
void pmw_write_reg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg | 0x80, data }; // MSB must be 1 for Write
    
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx, 2);
    gpio_put(PIN_CS, 1);
    
    sleep_us(200); // Required delay (tSWW) between writes
}
void pmw_upload_srom() {
    // 1. Tell the sensor to prepare for a firmware download
    pmw_write_reg(0x10, 0x00); // Disables rest mode
    pmw_write_reg(0x13, 0x1d); // SROM_Enable register
    sleep_ms(10);
    pmw_write_reg(0x13, 0x18); // Start the burst mode!
    
    // 2. Send the entire array over SPI
    gpio_put(PIN_CS, 0); // Select the sensor
    
    uint8_t srom_reg = 0x62 | 0x80; // SROM_Load_Burst register (0x62) + Write bit (0x80)
    spi_write_blocking(SPI_PORT, &srom_reg, 1); 
    sleep_us(15); // The sensor needs exactly 15 microseconds to prepare
    
    // Fire the firmware over the wire, one byte at a time
    for (int i = 0; i < 4096; i++) { 
        spi_write_blocking(SPI_PORT, &srom_data[i], 1);
        sleep_us(15); // Critical delay
    }
    
    gpio_put(PIN_CS, 1); // Deselect the sensor
    sleep_ms(1);
    
    // 3. The Verification
    uint8_t srom_id = pmw_read_reg(0x2A); 
    
    // Turn off the LED for a second so we know the upload finished
    cyw43_arch_gpio_put(PIN_LED, 0); 
    sleep_ms(1000); 

    if (srom_id == 0x00) {
        // FAILURE: 1 Long, sad 3-second flash (Upload completely failed)
        cyw43_arch_gpio_put(PIN_LED, 1); 
        sleep_ms(3000);
        cyw43_arch_gpio_put(PIN_LED, 0);
    } 
    else {
        // SUCCESS! Any non-zero value means the firmware successfully loaded.
        // 3 Slow, happy flashes!
        for(int i=0; i<3; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(500);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        }
    } 
}
    void pmw_read_motion(int16_t *dx, int16_t *dy) {
    // 1. Read the Motion register (0x02) to lock the movement data in place
    uint8_t motion = pmw_read_reg(0x02);
    
    // Check if the 7th bit is a 1 (which means "Motion Occurred")
    if (motion & 0x80) {

        cyw43_arch_gpio_put(PIN_LED,1);
        // 2. You MUST read all 4 of these registers in this exact order
        uint8_t xl = pmw_read_reg(0x03); // Delta_X_L
        uint8_t xh = pmw_read_reg(0x04); // Delta_X_H
        uint8_t yl = pmw_read_reg(0x05); // Delta_Y_L
        uint8_t yh = pmw_read_reg(0x06); // Delta_Y_H
        
        // 3. Combine the High and Low bytes into true 16-bit numbers
        *dx = (int16_t)((xh << 8) | xl);
        *dy = (int16_t)((yh << 8) | yl);
    } else {
        // No movement happened, send zeros so we don't drift!
        cyw43_arch_gpio_put(PIN_LED,0);
        *dx = 0;
        *dy = 0;
    }
}
/*void pmw_init() {
    // Hardware reset pulse
    gpio_put(PIN_RST, 0);
    sleep_ms(50);
    gpio_put(PIN_RST, 1);
    sleep_ms(50);
    
    // Read Product ID (Register 0x00)
    uint8_t product_id = pmw_read_reg(0x00);
    
    // If it reads 0x47, the SPI is working! Flash the LED.
    /*if (product_id == 0x47) {
        for (int i=0; i<5; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1);
            sleep_ms(100);
            cyw43_arch_gpio_put(PIN_LED, 0);
            sleep_ms(100);
        }
    }
   // DEBUG: If we get ANY response that isn't 0 or 255, flash once long!
    if (product_id != 0x00 && product_id != 0xFF) {
        cyw43_arch_gpio_put(PIN_LED, 1);
        sleep_ms(2000); // 2 second solid light means "I see SOMETHING"
        cyw43_arch_gpio_put(PIN_LED, 0);
    } 
    // If it's still 0 or 255, flash 10 tiny rapid blips (Error code)
    else {
        for (int i=0; i<10; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(50);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(50);
        }
    }
}
*/
void pmw_init() {
    // 1. TRIPWIRE: Turn LED on immediately so we know the function started!
    cyw43_arch_gpio_put(PIN_LED, 1);
    sleep_ms(1000); // Keep it on for 1 second so you can clearly see it
    
    // 2. Hardware reset pulse
    gpio_put(PIN_RST, 0);
    sleep_ms(50);
    gpio_put(PIN_RST, 1);
    sleep_ms(50);
    
    // 3. The Dangerous Part: Reading the SPI
    uint8_t product_id = pmw_read_reg(0x00);
    
    // 4. The Results
    if (product_id != 0x00 && product_id != 0xFF) {
        
        // --- THE WAKE UP SEQUENCE ---
        pmw_write_reg(0x3A, 0x5A); // Force Power-Up Reset
        sleep_ms(50); // Wait for it to wake up
        
        // Clear out the startup garbage data so the MT pin resets to HIGH
        pmw_read_reg(0x02);
        pmw_read_reg(0x03);
        pmw_read_reg(0x04);
        pmw_read_reg(0x05);
        pmw_read_reg(0x06);

        // --- UPLOAD THE SROM BRAIN ---
        pmw_upload_srom(); // <-- We inject our new function right here!

        // --- ENABLE THE SENSOR ---
        pmw_write_reg(0x20, 0x00); // Turn off power-saving rest mode
        sleep_ms(10);

        // Turn off, then flash 5 times for overall success!
        cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(500);
        for(int i=0; i<5; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(200);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(200);
        }
        pmw_write_reg(0x0F, 0x10); // PMW calcuates DPI by (Value * 50) so 0x10 is 16 * 50 = 800 DPI
    } 
    else {
        // Flash 10 tiny blips for failure
        for (int i=0; i<10; i++) {
            cyw43_arch_gpio_put(PIN_LED, 1); sleep_ms(50);
            cyw43_arch_gpio_put(PIN_LED, 0); sleep_ms(50);
        }
    }
    cyw43_arch_gpio_put(PIN_LED, 0); // Turn off the LED for good
}

// --- Task Definitions ---
void hid_task(void);

int main(void) {
    // Initialize standard board and USB
    board_init();
    tusb_init();
    
    // Initialize Pico W LED backend
    cyw43_arch_init();

    // Setup Buttons (pull-up resistors)
    gpio_init(BTN_LEFT);  gpio_set_dir(BTN_LEFT, GPIO_IN);  gpio_pull_up(BTN_LEFT);
    gpio_init(BTN_RIGHT); gpio_set_dir(BTN_RIGHT, GPIO_IN); gpio_pull_up(BTN_RIGHT);

    // Setup PMW3389 Hardware Pins
    gpio_init(PIN_CS);    gpio_set_dir(PIN_CS, GPIO_OUT);   gpio_put(PIN_CS, 1);
    gpio_init(PIN_RST);   gpio_set_dir(PIN_RST, GPIO_OUT);  gpio_put(PIN_RST, 1);
    gpio_init(PIN_MT);    gpio_set_dir(PIN_MT, GPIO_IN);    gpio_pull_up(PIN_MT); // MT is Active Low

    // Setup SPI0 hardware block (Mode 3: CPOL=1, CPHA=1 at 2MHz is standard for PixArt)
    spi_init(SPI_PORT, 2000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // Initialize the Optical Sensor
   
    pmw_init(); // Initialize the PMW3389 sensor (moved outside the loop for testing purposes)
    while (1) {
        tud_task(); // TinyUSB device task
        hid_task(); // Our mouse logic
    }
}

// --- HID Task ---
void hid_task(void) {
    // Poll interval (e.g., 2ms for 500Hz polling rate)
    const uint32_t interval_ms = 2;
    static uint32_t start_ms = 0;
    static uint8_t previous_buttons = 0;

    if ( board_millis() - start_ms < interval_ms) return; 
    start_ms += interval_ms;

    if ( !tud_hid_ready() ) return;

    // 1. Read the Clickers
    uint8_t current_buttons = 0;
    if ( !gpio_get(BTN_LEFT) )  current_buttons |= MOUSE_BUTTON_LEFT;
    if ( !gpio_get(BTN_RIGHT) ) current_buttons |= MOUSE_BUTTON_RIGHT;

    // 2. Read the Motion
    int16_t dx = 0;
    int16_t dy = 0;

    // The MT pin goes LOW when the sensor sees movement
    if ( !gpio_get(PIN_MT) ) {
        uint8_t motion = pmw_read_reg(0x02); // Read Motion register to latch the deltas
        // Must read registers in this exact order: XL, XH, YL, YH
        uint8_t xl = pmw_read_reg(REG_Delta_X_L);
        uint8_t xh = pmw_read_reg(REG_Delta_X_H);
        uint8_t yl = pmw_read_reg(REG_Delta_Y_L);
        uint8_t yh = pmw_read_reg(REG_Delta_Y_H);

        // Combine High and Low bytes into 16-bit integers
        dx = (int16_t)((xh << 8) | xl);
        dy = (int16_t)((yh << 8) | yl);
    }

    // 3. Prepare the USB Report
    // Standard TinyUSB Mouse endpoints use 8-bit values (-127 to +127). 
    // High-DPI sensors can overshoot this, so we clamp it safely.
    int8_t report_x = dx > 127 ? 127 : (dx < -127 ? -127 : (int8_t)dx);
    int8_t report_y = dy > 127 ? 127 : (dy < -127 ? -127 : (int8_t)dy);

    // Grab the acutal locked movement data
    pmw_read_motion(&dx, &dy);
    // We only send a USB update if something actually changed to save bandwidth
    if ( dx != 0 || dy != 0 || current_buttons != previous_buttons ) {
        // Notice we are sending report_x, and -report_y (optical sensors usually have inverted Y axes)
        tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, report_x, -report_y, 0, 0);
        
        // Flash the Pico W LED when we send data!
        cyw43_arch_gpio_put(PIN_LED, 1); 
    } else {
        cyw43_arch_gpio_put(PIN_LED, 0);
    }

    previous_buttons = current_buttons;
}

// --- TinyUSB Required Callbacks ---
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) bufsize;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void) instance; (void) report_id; (void) report_type; (void) buffer; (void) reqlen;
    return 0;
}