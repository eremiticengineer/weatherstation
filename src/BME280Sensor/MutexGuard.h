#ifndef MUTEX_GUARD_H
#define MUTEX_GUARD_H

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

    // Disable copying
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;

private:
    SemaphoreHandle_t _handle;
};

#endif // MUTEX_GUARD_H
