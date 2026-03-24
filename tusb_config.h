#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE
#define CFG_TUSB_OS               OPT_OS_PICO

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC               1 // Enable USB CDC (Serial Port) support for debug printf() output over USB1
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               1  // This tells it we are a Mouse/Keyboard!
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

// CDC FIFO sizes (This is what the compiler is complaining about!)
#define CFG_TUD_CDC_RX_BUFSIZE    64
#define CFG_TUD_CDC_TX_BUFSIZE    64

// CDC Endpoint size
#define CFG_TUD_CDC_EP_BUFSIZE    64

// HID Endpoint size
#define CFG_TUD_HID_EP_BUFSIZE    16

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */