#include "DS3231.h"

DS3231::DS3231(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c), _address(address) {}

void DS3231::init() {
    // Assumes I2C has already been initialized and SDA/SCL pins configured externally.
}

uint8_t DS3231::bcdToDec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t DS3231::decToBcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

bool DS3231::readTime(struct tm& time) {
    uint8_t buffer[7];
    uint8_t reg = 0x00;

    if (i2c_write_blocking(_i2c, _address, &reg, 1, true) != 1) return false;
    if (i2c_read_blocking(_i2c, _address, buffer, 7, false) != 7) return false;

    time.tm_sec  = bcdToDec(buffer[0] & 0x7F);
    time.tm_min  = bcdToDec(buffer[1]);
    time.tm_hour = bcdToDec(buffer[2] & 0x3F);
    time.tm_wday = bcdToDec(buffer[3]) - 1;  // RTC: 1=Sun, tm: 0=Sun
    time.tm_mday = bcdToDec(buffer[4]);
    time.tm_mon  = bcdToDec(buffer[5] & 0x1F) - 1;
    time.tm_year = bcdToDec(buffer[6]) + 100; // tm_year is years since 1900

    return true;
}

bool DS3231::setTime(const struct tm& time) {
    uint8_t buffer[8];
    buffer[0] = 0x00;
    buffer[1] = decToBcd(time.tm_sec);
    buffer[2] = decToBcd(time.tm_min);
    buffer[3] = decToBcd(time.tm_hour);
    buffer[4] = decToBcd(time.tm_wday + 1);  // tm: 0=Sun, RTC: 1=Sun
    buffer[5] = decToBcd(time.tm_mday);
    buffer[6] = decToBcd(time.tm_mon + 1);
    buffer[7] = decToBcd(time.tm_year - 100);

    return i2c_write_blocking(_i2c, _address, buffer, 8, false) == 8;
}
