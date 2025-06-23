#include "FreeRTOS.h"
#include "semphr.h"

class MutexGuard {
public:
    explicit MutexGuard(SemaphoreHandle_t handle) : _handle(handle) {
        xSemaphoreTake(_handle, portMAX_DELAY);
    }

    ~MutexGuard() {
        xSemaphoreGive(_handle);
    }

    // Prevent copying
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;

private:
    SemaphoreHandle_t _handle;
};
