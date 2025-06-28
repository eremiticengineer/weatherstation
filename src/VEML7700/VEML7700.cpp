#include "VEML7700.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cmath>

#define VEML7700_ALS_CONF    0x00
#define VEML7700_ALS_DATA    0x04

VEML7700::VEML7700(i2c_inst_t *i2c_inst, uint8_t address, uint sda, uint scl)
    : i2c(i2c_inst), i2c_address(address), sda_pin(sda), scl_pin(scl) {}

bool VEML7700::begin() {
    // i2c_init(i2c, 100 * 1000); // 100 kHz
    // gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    // gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    // gpio_pull_up(sda_pin);
    // gpio_pull_up(scl_pin);

    configure();

    // Read from sensor to check it's responding
    uint16_t dummy;
    return readRegister(VEML7700_ALS_CONF, dummy);
}

// void VEML7700::configure() {
//     // Set ALS gain = 1x, integration time = 100 ms, ALS enable
//     uint16_t config = 0x0000;
//     config |= (0x00 << 11); // Gain 1x
//     config |= (0x02 << 6);  // IT 100 ms
//     config |= (0x00 << 1);  // ALS power on

//     writeRegister(VEML7700_ALS_CONF, config);
// }

void VEML7700::configure() {
    // Gain = 1x      (00 << 11)
    // IT = 100ms     (110 << 6)
    // Persistence = 1 (00 << 4)
    // Power on       (0 << 1)
    // ALS enable     (0 << 0)

    uint16_t config = 0x0000;
    config |= (0x0 << 11); // gain: 00 = 1x
    config |= (0x6 << 6);  // IT: 110 = 100ms
    config |= (0x0 << 4);  // persistence: 00 = 1 sample
    config |= (0x0 << 1);  // power mode: 0 = on
    config |= (0x0 << 0);  // ALS enable: 0 = ALS on

    writeRegister(VEML7700_ALS_CONF, config);
}

bool VEML7700::readLux(float &lux) {
    uint16_t raw;
    if (!readRegister(VEML7700_ALS_DATA, raw)) {
        return false;
    }

    // According to datasheet, lux = raw * (0.0576) for gain=1x, IT=100ms
    lux = raw * 0.0576f;
    return true;
}

bool VEML7700::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = value & 0xFF;
    buf[2] = value >> 8;
    return i2c_write_blocking(i2c, i2c_address, buf, 3, false) == 3;
}

bool VEML7700::readRegister(uint8_t reg, uint16_t &value) {
    if (i2c_write_blocking(i2c, i2c_address, &reg, 1, true) != 1) {
        return false;
    }

    uint8_t buf[2];
    if (i2c_read_blocking(i2c, i2c_address, buf, 2, false) != 2) {
        return false;
    }

    value = buf[0] | (buf[1] << 8);
    return true;
}
