#ifndef VEML7700_H
#define VEML7700_H

#include "hardware/i2c.h"

class VEML7700 {
public:
    VEML7700(i2c_inst_t *i2c, uint8_t address = 0x10, uint sda_pin = 2, uint scl_pin = 3);

    bool begin();
    bool readLux(float &lux);
    void configure(); // default settings

private:
    i2c_inst_t *i2c;
    uint8_t i2c_address;
    uint sda_pin, scl_pin;

    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegister(uint8_t reg, uint16_t &value);
};

#endif // VEML7700_H
