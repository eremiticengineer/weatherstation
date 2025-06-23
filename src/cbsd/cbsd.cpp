#include "cbsd.h"
#include <vector>

// See FatFs - Generic FAT Filesystem Module, "Application Interface",
// http://elm-chan.org/fsw/ff/00index_e.html

// https://electronics.stackexchange.com/questions/373793/using-c-objects-within-freertos-tasks
// https://forums.freertos.org/t/freertos-with-c-writing-a-class-with-a-task-as-a-method/18862/3
// https://forums.freertos.org/t/using-the-queue-with-c-classes/18959/3

using namespace Codebrane;

static inline void stop() {
    fflush(stdout);
    __breakpoint();
}

void writeToSDTask(void* pvParameters) {
    while(true) {
        printf("here1/n");
        CBSD* pCBSD = static_cast<CBSD*>(pvParameters);
        printf("here2/n");
        // pCBSD->write();
        printf("here3/n");
        delete pCBSD;
        printf("here4/n");
        //vTaskDelete(NULL);
        vTaskDelay(1000);
    }
}

CBSD::CBSD() {}



//spi_t* spi_p2;

void CBSD::writeDataToDisk() {
    spi_t* spi_p = new spi_t();
    spi_p->hw_inst = spi1;  // RP2040 SPI component
    spi_p->miso_gpio = 12;
    spi_p->sck_gpio = 10;   // GPIO number (not Pico pin number)
    spi_p->mosi_gpio = 11;

    sd_spi_if_t *spi_if_p = new sd_spi_if_t();
    spi_if_p->spi = spi_p;  // Pointer to the SPI driving this card
    spi_if_p->ss_gpio = 12;   // The SPI slave select GPIO for this SD card

    sd_card_t *sd_card_p = new sd_card_t();
    sd_card_p->device_name = "sd0";  // Name used to mount device
    sd_card_p->mount_point = "/sd0";
    sd_card_p->type = SD_IF_SPI;
    sd_card_p->spi_if_p = spi_if_p;
    sd_card_p->use_card_detect = false;
    //sd_card_p->card_detect_gpio = 9;
    sd_card_p->card_detected_true = 0;  // What the GPIO read returns when a card is present
    sd_card_p->card_detect_use_pull = true;
    sd_card_p->card_detect_pull_hi = true;

    FF_Disk_t *pxDisk = FF_SDDiskInit(sd_card_p->device_name);
    FF_Error_t xError = FF_SDDiskMount(pxDisk);
    if (FF_isERR(xError) != pdFALSE) {
        printf("========================= cannot mount\n");
    }
    FF_FS_Add(sd_card_p->mount_point, pxDisk);
    std::string filename = std::string(sd_card_p->mount_point) + "/filename2.txt";
    FF_FILE *pxFile = ff_fopen(filename.c_str(), "a");
    if (!pxFile) {
        printf("========================= cannot open file\n");
    }
    ff_fprintf(pxFile, "Hello, world!\n");
    ff_fclose(pxFile);
    printf("WRITTEN DATA -----------------------------------\n");

    FF_FS_Remove(sd_card_p->mount_point);
    FF_Unmount(pxDisk);
    FF_SDDiskDelete(pxDisk);

    delete sd_card_p;
    delete spi_if_p;
    delete spi_p;
}


spi_t* spi_p2;
sd_spi_if_t *spi_if_p2;
sd_card_t *sd_card_p2;
FF_Disk_t *pxDisk2;
void CBSD::init() {
    spi_p2 = new spi_t();
    spi_p2->hw_inst = spi1;  // RP2040 SPI component
    spi_p2->miso_gpio = 12;
    spi_p2->sck_gpio = 10;   // GPIO number (not Pico pin number)
    spi_p2->mosi_gpio = 11;

    spi_if_p2 = new sd_spi_if_t();
    spi_if_p2->spi = spi_p2;  // Pointer to the SPI driving this card
    spi_if_p2->ss_gpio = 13;   // The SPI slave select GPIO for this SD card

    sd_card_p2 = new sd_card_t();
    sd_card_p2->device_name = "sd0";  // Name used to mount device
    sd_card_p2->mount_point = "/sd0";
    sd_card_p2->type = SD_IF_SPI;
    sd_card_p2->spi_if_p = spi_if_p2;
    sd_card_p2->use_card_detect = false;
    //sd_card_p->card_detect_gpio = 9;
    sd_card_p2->card_detected_true = 0;  // What the GPIO read returns when a card is present
    sd_card_p2->card_detect_use_pull = true;
    sd_card_p2->card_detect_pull_hi = true;

    pxDisk2 = FF_SDDiskInit(sd_card_p2->device_name);
    FF_SDDiskMount(pxDisk2);
    FF_FS_Add(sd_card_p2->mount_point, pxDisk2);
}
// void CBSD::writeAfterInit() {
//     std::string filename = std::string(sd_card_p2->mount_point) + "/filename3.txt";
//     FF_FILE *pxFile = ff_fopen(filename.c_str(), "a");
//     if (!pxFile) {
//         printf("========================= cannot open file\n");
//     }
//     ff_fprintf(pxFile, "Hello, world3!\n");
//     ff_fclose(pxFile);
//     printf("WRITTEN DATA -----------------------------------\n");
// }
void CBSD::writeAfterInit(const std::string& data) {
    std::string filename = std::string(sd_card_p2->mount_point) + "/weather.txt";
    FF_FILE *pxFile = ff_fopen(filename.c_str(), "a");
    ff_fprintf(pxFile, data.c_str());
    ff_fprintf(pxFile, "\n");
    ff_fclose(pxFile);
    printf("WRITTEN DATA -----------------------------------\n");
}
void CBSD::deinit() {
    FF_FS_Remove(sd_card_p2->mount_point);
    FF_Unmount(pxDisk2);
    FF_SDDiskDelete(pxDisk2);

    delete sd_card_p2;
    delete spi_if_p2;
    delete spi_p2;
}









void CBSD::mount() {
    printf("mount\n");
    pxDisk = FF_SDDiskInit("sd0");
    FF_SDDiskMount(pxDisk);
    FF_FS_Add("/sd0", pxDisk);
}

void CBSD::unmount() {
    printf("unmount\n");
    FF_FS_Remove("/sd0");
    FF_Unmount(pxDisk);
    // mount() will fail after this is called
    // https://github.com/carlk3/FreeRTOS-FAT-CLI-for-RPi-Pico/issues/48
    //FF_SDDiskDelete(pxDisk);
}

void CBSD::write(const std::string& data) {
    printf("write >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    FF_FILE *pxFile = ff_fopen("/sd0/weather.txt", "a");
    ff_fprintf(pxFile, data.c_str());
    ff_fprintf(pxFile, "\n");
    ff_fclose(pxFile);
}

void CBSD::write2() {
    FF_Disk_t *pxDisk = FF_SDDiskInit("sd0");
    configASSERT(pxDisk);
    FF_Error_t xError = FF_SDDiskMount(pxDisk);
    if (FF_isERR(xError) != pdFALSE) {
        FF_PRINTF("FF_SDDiskMount: %s\n",
                  (const char *)FF_GetErrMessage(xError));
        stop();
    }
    FF_FS_Add("/sd0", pxDisk);
    
    FF_FILE *pxFile = ff_fopen("/sd0/filename.txt", "a");
    if (!pxFile) {
        FF_PRINTF("ff_fopen failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    if (ff_fprintf(pxFile, "Weather station data on disk!\n") < 0) {
        FF_PRINTF("ff_fprintf failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    if (-1 == ff_fclose(pxFile)) {
        FF_PRINTF("ff_fclose failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    FF_FS_Remove("/sd0");
    FF_Unmount(pxDisk);
    FF_SDDiskDelete(pxDisk);
    puts("Goodbye, world!");

    //vTaskDelete(NULL);
}

void CBSD::writeData(const std::string& data) {
    FF_Disk_t *pxDisk = FF_SDDiskInit("sd0");
    configASSERT(pxDisk);
    FF_Error_t xError = FF_SDDiskMount(pxDisk);
    if (FF_isERR(xError) != pdFALSE) {
        FF_PRINTF("FF_SDDiskMount: %s\n",
                  (const char *)FF_GetErrMessage(xError));
        stop();
    }
    FF_FS_Add("/sd0", pxDisk);
    
    FF_FILE *pxFile = ff_fopen("/sd0/filename.txt", "a");
    if (!pxFile) {
        FF_PRINTF("ff_fopen failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    //if (ff_fprintf(pxFile, data.c_str()) < 0) {
    if (ff_fprintf(pxFile, "Weather station data on disk!\n") < 0) {
        FF_PRINTF("ff_fprintf failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    if (-1 == ff_fclose(pxFile)) {
        FF_PRINTF("ff_fclose failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    FF_FS_Remove("/sd0");
    FF_Unmount(pxDisk);
    FF_SDDiskDelete(pxDisk);
    puts("Goodbye, world!");
}
