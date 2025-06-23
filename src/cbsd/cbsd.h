#ifndef SD_H
#define SD_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include <string>

#include "ff_headers.h"
#include "ff_sddisk.h"
#include "ff_stdio.h"
#include "ff_utils.h"

//#include "hw_config.h"

extern "C" void writeToSDTask(void* pvParameters);

namespace Codebrane {

class CBSD {
public:
    CBSD();
    void init();
    //void writeAfterInit();
    void writeAfterInit(const std::string& data);
    void deinit();
    void writeDataToDisk();
    void write(const std::string& data);
    void write2();
    void mount();
    void unmount();
    void writeData(const std::string& data);
private:
    FF_Disk_t *pxDisk;
};

} // namespace Codebrane

#endif // SD_H
