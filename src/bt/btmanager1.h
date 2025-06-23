#ifndef BT_H
#define BT_H

// #include "FreeRTOS.h"
// #include "task.h"

#include <string>

#include "btstack_run_loop.h"
#include "hci.h"

// extern "C" void sendDataTask(void* pvParameters);

namespace Codebrane {

class BTManager {
public:
    BTManager();
    void init();
    void turnOnBT();
    void turnOffBT();
    void send();
    void handlePacket(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
    uint16_t attReadCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
    int attWriteCallback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
    void heartbeatHandler(btstack_timer_source_t *ts);

private:
    int secretValue;
    static void btstackPacketHandlerWrapper(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
    static uint16_t attReadCallbackWrapper(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
    static int attWriteCallbackWrapper(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
    static void heartbeatHandlerWrapper(btstack_timer_source_t *ts);
    static BTManager* instance;  // Global reference for the static wrapper
};

} // namespace Codebrane

#endif // BT_H
