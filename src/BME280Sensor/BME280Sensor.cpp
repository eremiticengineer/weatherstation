#include "BME280Sensor.h"
#include "MutexGuard.h"
#include "pico/stdlib.h"
#include <cstring>

BME280Sensor::BME280Sensor(i2c_inst_t* i2c, uint8_t address, uint sda_pin, uint scl_pin)
    : _i2c(i2c), _address(address), _sda_pin(sda_pin), _scl_pin(scl_pin) {}

bool BME280Sensor::init() {
    i2c_init(_i2c, 100 * 1000);
    gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_sda_pin);
    gpio_pull_up(_scl_pin);

    if (readRegister(0xD0) != 0x60) return false;  // Check chip ID

    writeRegister(0xF2, 0x01); // humidity oversampling x1
    writeRegister(0xF4, 0x27); // temp/press oversampling x1, normal mode
    writeRegister(0xF5, 0xA0); // config

    return readCalibrationData();
}

bool BME280Sensor::readCalibrationData() {
    uint8_t calib1[26];
    readRegisters(0x88, calib1, 26);

    dig_T1 = calib1[1] << 8 | calib1[0];
    dig_T2 = calib1[3] << 8 | calib1[2];
    dig_T3 = calib1[5] << 8 | calib1[4];
    dig_P1 = calib1[7] << 8 | calib1[6];
    dig_P2 = calib1[9] << 8 | calib1[8];
    dig_P3 = calib1[11] << 8 | calib1[10];
    dig_P4 = calib1[13] << 8 | calib1[12];
    dig_P5 = calib1[15] << 8 | calib1[14];
    dig_P6 = calib1[17] << 8 | calib1[16];
    dig_P7 = calib1[19] << 8 | calib1[18];
    dig_P8 = calib1[21] << 8 | calib1[20];
    dig_P9 = calib1[23] << 8 | calib1[22];
    dig_H1 = readRegister(0xA1);

    uint8_t calib2[7];
    readRegisters(0xE1, calib2, 7);
    dig_H2 = calib2[1] << 8 | calib2[0];
    dig_H3 = calib2[2];
    dig_H4 = (calib2[3] << 4) | (calib2[4] & 0x0F);
    dig_H5 = (calib2[5] << 4) | (calib2[4] >> 4);
    dig_H6 = calib2[6];

    return true;
}

bool BME280Sensor::readSensor(float& temperature, float& pressure, float& humidity) {

    // MutexGuard lock(_mutex); // Automatically takes the mutex
    uint8_t data[8];
    readRegisters(0xF7, data, 8);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = ((int32_t)data[6] << 8) | data[7];

    int32_t t = compensateTemperature(adc_T);
    uint32_t p = compensatePressure(adc_P);
    uint32_t h = compensateHumidity(adc_H);

    temperature = t / 100.0f;
    pressure = p / 100.0f;
    humidity = h / 1024.0f;

    // if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    //     uint8_t data[8];
    //     readRegisters(0xF7, data, 8);

    //     int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    //     int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    //     int32_t adc_H = ((int32_t)data[6] << 8) | data[7];

    //     int32_t t = compensateTemperature(adc_T);
    //     uint32_t p = compensatePressure(adc_P);
    //     uint32_t h = compensateHumidity(adc_H);

    //     temperature = t / 100.0f;
    //     pressure = p / 100.0f;
    //     humidity = h / 1024.0f;

    //     xSemaphoreGive(_mutex);
    // }    

    return true;
}

uint8_t BME280Sensor::readRegister(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(_i2c, _address, &reg, 1, true);
    i2c_read_blocking(_i2c, _address, &val, 1, false);
    return val;
}

void BME280Sensor::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    i2c_write_blocking(_i2c, _address, &reg, 1, true);
    i2c_read_blocking(_i2c, _address, buf, len, false);
}

void BME280Sensor::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(_i2c, _address, buf, 2, false);
}

// Compensation formulas (from datasheet)

int32_t BME280Sensor::compensateTemperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                   ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

uint32_t BME280Sensor::compensatePressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)(p >> 8);
}

uint32_t BME280Sensor::compensateHumidity(int32_t adc_H) {
    int32_t v_x1 = t_fine - ((int32_t)76800);
    v_x1 = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
               (((int32_t)dig_H5) * v_x1)) + ((int32_t)16384)) >> 15) *
             (((((((v_x1 * ((int32_t)dig_H6)) >> 10) *
                  (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (uint32_t)(v_x1 >> 12);
}
