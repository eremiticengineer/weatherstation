#ifndef BME280_SENSOR_H
#define BME280_SENSOR_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"

class BME280Sensor {
public:
    BME280Sensor(i2c_inst_t* i2c, uint8_t address, uint sda_pin, uint scl_pin);
    bool init();
    bool readSensor(float& temperature, float& pressure, float& humidity);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    uint _sda_pin, _scl_pin;
    SemaphoreHandle_t _mutex;

    // Compensation parameters
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1, dig_H3;
    int8_t dig_H6;
    int16_t dig_H2, dig_H4, dig_H5;

    int32_t t_fine;

    bool readCalibrationData();
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buf, size_t len);
    void writeRegister(uint8_t reg, uint8_t value);
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
    uint32_t compensateHumidity(int32_t adc_H);
};

#endif // BME280_SENSOR_H
