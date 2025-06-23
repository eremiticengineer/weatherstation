cd lib
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
git clone --recurse-submodules https://github.com/carlk3/FreeRTOS-FAT-CLI-for-RPi-Pico.git

cp /home/pi/dev/pico/pico-examples/freertos/FreeRTOSConfig_examples_common.h ./FreeRTOSConfig.h

include this in CMakeLists.txt
  lib/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake
or get it from:
https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/main/portable/ThirdParty/GCC/RP2040
