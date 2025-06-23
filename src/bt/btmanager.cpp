#include "btmanager.h"
#include <cstdio>
#include <cstring>
#include <cassert>
#include "btstack.h"
//#include "pico/btstack_cyw43.h"
#include "temp_sensor.h"
#include "pico/btstack_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define APP_AD_FLAGS 0x06
static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x17, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);

#define HEARTBEAT_PERIOD_MS 1000

using namespace Codebrane;

static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;
hci_con_handle_t con_handle;
uint16_t current_temp = 222;
int le_notification_enabled;

// Static member must be defined outside class
BTManager* BTManager::instance = nullptr;

BTManager::BTManager() {
    instance = this;
}

void BTManager::init() {
    l2cap_init();
    sm_init();
    att_server_init(profile_data, attReadCallbackWrapper, attWriteCallbackWrapper);
    hci_event_callback_registration.callback = &BTManager::btstackPacketHandlerWrapper;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(BTManager::btstackPacketHandlerWrapper);
    heartbeat.process = &BTManager::heartbeatHandlerWrapper;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);
}

void BTManager::btstackPacketHandlerWrapper(uint8_t packet_type, uint16_t channel,
    uint8_t* packet, uint16_t size) {    
    if (instance) {
        instance->handlePacket(packet_type, channel, packet, size);
    }
}
void BTManager::handlePacket(uint8_t packet_type, uint16_t channel,
    uint8_t* packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);

    bd_addr_t local_addr;
    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type) {
        case BTSTACK_EVENT_STATE: {
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // ble limitation
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);
            break;
        }
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = 0;
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, (uint8_t*)&current_temp, sizeof(current_temp));
            break;
        default:
            break;
    }

    // This is now your instance method—do what you want!
    // if (packet_type == HCI_EVENT_PACKET) {
    //     uint8_t event = packet[0];
    //     if (event == BTSTACK_EVENT_STATE) {
    //         uint8_t state = packet[2];
    //         if (state == HCI_STATE_WORKING) {
    //             // Bluetooth is up!
    //         }
    //     }
    // }
}

uint16_t BTManager::attReadCallbackWrapper(hci_con_handle_t connection_handle, uint16_t att_handle,
        uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    if (instance) {
        return instance->attReadCallback(connection_handle, att_handle, offset, buffer, buffer_size);
    }
    else {
        return -1;
    }
}
uint16_t BTManager::attReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE){
        return att_read_callback_handle_blob((const uint8_t *)&current_temp, sizeof(current_temp), offset, buffer, buffer_size);
    }
    return 0;
}

int BTManager::attWriteCallbackWrapper(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    if (instance) {
        return instance->attWriteCallback(connection_handle, att_handle, transaction_mode, offset,
            buffer, buffer_size);
    }
    else {
        return -1;
    }
}
int BTManager::attWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle,
    uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    
    if (att_handle != ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) return 0;
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    con_handle = connection_handle;
    if (le_notification_enabled) {
        att_server_request_can_send_now_event(con_handle);
    }
    return 0;
}

void BTManager::heartbeatHandlerWrapper(btstack_timer_source_t *ts) {
    if (instance) {
        instance->heartbeatHandler(ts);
    }
}
void BTManager::heartbeatHandler(btstack_timer_source_t *ts) {
    static uint32_t counter = 0;
    counter++;

    // Update the temp every 10s
   if (counter % 10 == 0) {
       att_server_request_can_send_now_event(con_handle);
   }

    // Invert the led
    static int led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

void BTManager::turnOnBT() {
    hci_power_control(HCI_POWER_ON);
}

void BTManager::turnOffBT() {
    hci_power_control(HCI_POWER_OFF);
}

void BTManager::send() {
  att_server_request_can_send_now_event(con_handle);
}
