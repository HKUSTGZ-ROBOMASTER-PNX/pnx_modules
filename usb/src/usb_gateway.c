#include "usb_gateway.h"

#include "app_usbx_device.h"
#include "crc_c.h"
#include "main.h"
#include "om.h"
#include "tx_api.h"
#include "usb_otg.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_dcd_stm32.h"

#include <string.h>

#define PNX_USB_MAX_MESSAGES 8U
#define PNX_USB_CDC_READ_SIZE 64U

typedef struct {
  UsbMessageDesc desc;
  om_topic_t* topic;
  om_suber_t* suber;
} UsbMessageEntry;

static UX_SLAVE_CLASS_CDC_ACM* g_cdc_acm = UX_NULL;
static UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER g_line_coding = {
    115200, 0x00, 0x00, 0x08};

static UsbMessageEntry g_entries[PNX_USB_MAX_MESSAGES];
static uint8_t g_entry_count = 0;

static TX_THREAD g_cdc_read_thread;
static TX_THREAD g_cdc_write_thread;

static uint8_t g_rx_buffer[512];

static UsbMessageEntry* find_or_create_topic(const UsbMessageDesc* desc) {
  for (uint8_t i = 0; i < g_entry_count; ++i) {
    if (strncmp(g_entries[i].desc.topic_name, desc->topic_name, 32) == 0) {
      return &g_entries[i];
    }
  }
  if (g_entry_count >= PNX_USB_MAX_MESSAGES) {
    return UX_NULL;
  }

  UsbMessageEntry* entry = &g_entries[g_entry_count++];
  entry->desc = *desc;
  entry->topic = om_find_topic(desc->topic_name, 0);
  if (entry->topic == UX_NULL) {
    entry->topic = om_create_topic((char*)desc->topic_name, desc->wire_size);
    if (entry->topic != UX_NULL && om_add_topic(entry->topic) != OM_OK) {
      entry->topic = UX_NULL;
    }
  }
  if ((desc->direction & PNX_USB_MSG_TX) && entry->topic != UX_NULL) {
    entry->suber = om_subscribe(entry->topic);
  }
  return entry;
}

void pnx_usb_gateway_init(void) {
  memset(g_entries, 0, sizeof(g_entries));
  g_entry_count = 0;
  g_cdc_acm = UX_NULL;
}

void pnx_usb_gateway_register(const UsbMessageDesc* desc) {
  if (desc == UX_NULL || desc->topic_name == UX_NULL || desc->wire_size == 0) {
    return;
  }
  (void)find_or_create_topic(desc);
}

void pnx_usb_cdc_activate(void* cdc_acm_instance) {
  g_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*)cdc_acm_instance;
  if (ux_device_class_cdc_acm_ioctl(g_cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                    &g_line_coding) != UX_SUCCESS) {
    Error_Handler();
  }
}

void pnx_usb_cdc_deactivate(void* cdc_acm_instance) {
  (void)cdc_acm_instance;
  g_cdc_acm = UX_NULL;
}

void pnx_usb_cdc_parameter_change(void* cdc_acm_instance) {
  (void)cdc_acm_instance;
  ULONG request;
  UX_SLAVE_TRANSFER* transfer_request;
  UX_SLAVE_DEVICE* device;

  device = &_ux_system_slave->ux_system_slave_device;
  transfer_request = &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;
  request = *(transfer_request->ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request) {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING:
      if (ux_device_class_cdc_acm_ioctl(g_cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &g_line_coding) != UX_SUCCESS) {
        Error_Handler();
      }
      if (g_line_coding.ux_slave_class_cdc_acm_parameter_baudrate < 9600) {
        g_line_coding.ux_slave_class_cdc_acm_parameter_baudrate = 9600;
      }
      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING:
      if (ux_device_class_cdc_acm_ioctl(g_cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &g_line_coding) != UX_SUCCESS) {
        Error_Handler();
      }
      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:
    default:
      break;
  }
}

static UsbMessageEntry* match_rx_message(const uint8_t* data, ULONG length) {
  for (uint8_t i = 0; i < g_entry_count; ++i) {
    const UsbMessageDesc* desc = &g_entries[i].desc;
    if ((desc->direction & PNX_USB_MSG_RX) == 0) {
      continue;
    }
    if (length < desc->wire_size) {
      continue;
    }
    if (desc->rx_header != 0 && data[0] != desc->rx_header) {
      continue;
    }
    if (desc->use_crc16 && pnx_crc16_verify(data, desc->wire_size) != 0) {
      continue;
    }
    return &g_entries[i];
  }
  return UX_NULL;
}

void pnx_usb_gateway_rx_thread_entry(ULONG thread_input) {
  (void)thread_input;
  UX_SLAVE_DEVICE* device = &_ux_system_slave->ux_system_slave_device;
  uint8_t publish_buf[256];

  while (1) {
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (g_cdc_acm != UX_NULL)) {
      ULONG actual_length = 0;
      ux_device_class_cdc_acm_read(g_cdc_acm, g_rx_buffer, PNX_USB_CDC_READ_SIZE, &actual_length);

      UsbMessageEntry* entry = match_rx_message(g_rx_buffer, actual_length);
      if (entry != UX_NULL && entry->topic != UX_NULL) {
        memcpy(publish_buf, g_rx_buffer, entry->desc.wire_size);
        om_publish(entry->topic, publish_buf, entry->desc.wire_size, true, false);
      }
    }
    tx_thread_sleep(2);
  }
}

void pnx_usb_gateway_tx_thread_entry(ULONG thread_input) {
  (void)thread_input;
  UX_SLAVE_DEVICE* device = &_ux_system_slave->ux_system_slave_device;
  uint8_t tx_buf[256];

  tx_thread_sleep(10);
  while (1) {
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (g_cdc_acm != UX_NULL)) {
      for (uint8_t i = 0; i < g_entry_count; ++i) {
        UsbMessageEntry* entry = &g_entries[i];
        if ((entry->desc.direction & PNX_USB_MSG_TX) == 0 || entry->suber == UX_NULL) {
          continue;
        }
        if (!om_suber_available(entry->suber)) {
          continue;
        }
        if (om_suber_export(entry->suber, tx_buf, false) != OM_OK) {
          continue;
        }
        if (entry->desc.tx_header != 0) {
          tx_buf[0] = entry->desc.tx_header;
        }
        if (entry->desc.use_crc16) {
          pnx_crc16_append(tx_buf, entry->desc.wire_size);
        }
        ULONG actual_length = 0;
        ux_device_class_cdc_acm_write(g_cdc_acm, tx_buf, entry->desc.wire_size, &actual_length);
      }
    }
    tx_thread_sleep(2);
  }
}

void pnx_usb_gateway_start(void* ux_byte_pool) {
  TX_BYTE_POOL* pool = (TX_BYTE_POOL*)ux_byte_pool;
  UCHAR* pointer = UX_NULL;

  if (tx_byte_allocate(pool, (VOID**)&pointer, UX_DEVICE_APP_THREAD_STACK_SIZE, TX_NO_WAIT) !=
      TX_SUCCESS) {
    return;
  }
  if (tx_thread_create(&g_cdc_read_thread, "usb_cdc_rx", pnx_usb_gateway_rx_thread_entry, 0, pointer,
                       UX_DEVICE_APP_THREAD_STACK_SIZE, 5, 5, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS) {
    return;
  }

  if (tx_byte_allocate(pool, (VOID**)&pointer, UX_DEVICE_APP_THREAD_STACK_SIZE, TX_NO_WAIT) !=
      TX_SUCCESS) {
    return;
  }
  if (tx_thread_create(&g_cdc_write_thread, "usb_cdc_tx", pnx_usb_gateway_tx_thread_entry, 0,
                       pointer, UX_DEVICE_APP_THREAD_STACK_SIZE, 11, 11, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS) {
    return;
  }
}

void USBX_APP_Device_Init(void) {
  MX_USB_OTG_HS_PCD_Init();

  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, 0x200);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0, 0x10);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x10);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 2, 0x20);

  ux_dcd_stm32_initialize((ULONG)USB_OTG_HS, (ULONG)&hpcd_USB_OTG_HS);
  HAL_PCD_Start(&hpcd_USB_OTG_HS);
}
