#ifndef DS3231_H
#define DS3231_H

#include "pico/stdlib.h"
#include "pico/util/datetime.h"
#include "hardware/i2c.h"
#include <cstdint>

class DS3231 {
public:
    DS3231(i2c_inst_t* i2c, uint8_t address = 0x68);

    void init();
    bool readTime(struct tm& time);
    bool setTime(const struct tm& time);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;

    uint8_t bcdToDec(uint8_t val);
    uint8_t decToBcd(uint8_t val);
};

#endif
