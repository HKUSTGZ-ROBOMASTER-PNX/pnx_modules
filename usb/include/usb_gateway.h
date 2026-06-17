#pragma once

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PNX_USB_MSG_RX 0x01U
#define PNX_USB_MSG_TX 0x02U

typedef struct {
  const char* topic_name;
  uint16_t wire_size;
  uint8_t rx_header;
  uint8_t tx_header;
  uint8_t use_crc16;
  uint8_t direction;
} UsbMessageDesc;

void pnx_usb_gateway_init(void);
void pnx_usb_gateway_register(const UsbMessageDesc* desc);
void pnx_usb_gateway_start(void* ux_byte_pool);

void pnx_usb_cdc_activate(void* cdc_acm_instance);
void pnx_usb_cdc_deactivate(void* cdc_acm_instance);
void pnx_usb_cdc_parameter_change(void* cdc_acm_instance);

void pnx_usb_gateway_rx_thread_entry(ULONG thread_input);
void pnx_usb_gateway_tx_thread_entry(ULONG thread_input);

void USBX_APP_Device_Init(void);

#ifdef __cplusplus
}
#endif
