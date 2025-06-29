#ifndef CBLORA_H
#define CBLORA_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

extern "C" void writeToSDTask(void* pvParameters);

namespace Codebrane {

typedef enum {fmIdle, fmLaunched, fmDescending, fmLanding, fmLanded} TFlightMode;
struct TGPS
{
	long Time;							// Time as read from GPS, as an integer but 12:13:14 is 121314
	long SecondsInDay;					// Time in seconds since midnight.  Used for APRS timing, and LoRa timing in TDM mode
	int Hours, Minutes, Seconds;
	float Longitude, Latitude;
	long Altitude, MinimumAltitude, MaximumAltitude, PreviousAltitude;
	unsigned int Satellites;
	int Speed;
	int Direction;
	float AscentRate;
	float BatteryVoltage;
	float InternalTemperature;
	float ExternalTemperature;
	float Pressure;
	float Humidity;
	TFlightMode FlightMode;
	float PredictedLongitude, PredictedLatitude;
	float CDA;
	int UseHostPosition;
	int TimeTillLanding;
	float PredictedLandingSpeed;
};

// RFM98 registers
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_AD 0x0E
#define REG_FIFO_RX_BASE_AD 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_MODEM_CONFIG 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_HOP_PERIOD 0x24
#define REG_FREQ_ERROR 0x28
#define REG_DETECT_OPT 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41

// FSK stuff
#define REG_PREAMBLE_MSB_FSK 0x25
#define REG_PREAMBLE_LSB_FSK 0x26
#define REG_PACKET_CONFIG1 0x30
#define REG_PAYLOAD_LENGTH_FSK 0x32
#define REG_FIFO_THRESH 0x35
#define REG_FDEV_MSB 0x04
#define REG_FDEV_LSB 0x05
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_BITRATE_MSB 0x02
#define REG_BITRATE_LSB 0x03
#define REG_IRQ_FLAGS2 0x3F

// MODES
#define RF98_MODE_RX_CONTINUOUS 0x85
#define RF98_MODE_TX 0x83
#define RF98_MODE_SLEEP 0x80
#define RF98_MODE_STANDBY 0x81

#define PAYLOAD_LENGTH 255

// Modem Config 1
#define EXPLICIT_MODE 0x00
#define IMPLICIT_MODE 0x01

#define ERROR_CODING_4_5 0x02
#define ERROR_CODING_4_6 0x04
#define ERROR_CODING_4_7 0x06
#define ERROR_CODING_4_8 0x08

#define BANDWIDTH_7K8 0x00
#define BANDWIDTH_10K4 0x10
#define BANDWIDTH_15K6 0x20
#define BANDWIDTH_20K8 0x30
#define BANDWIDTH_31K25 0x40
#define BANDWIDTH_41K7 0x50
#define BANDWIDTH_62K5 0x60
#define BANDWIDTH_125K 0x70
#define BANDWIDTH_250K 0x80
#define BANDWIDTH_500K 0x90

// Modem Config 2

#define SPREADING_6 0x60
#define SPREADING_7 0x70
#define SPREADING_8 0x80
#define SPREADING_9 0x90
#define SPREADING_10 0xA0
#define SPREADING_11 0xB0
#define SPREADING_12 0xC0

#define CRC_OFF 0x00
#define CRC_ON 0x04

// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG 0x09
#define PA_MAX_BOOST 0x8F // 100mW (max 869.4 - 869.65)
#define PA_LOW_BOOST 0x81
#define PA_MED_BOOST 0x8A
#define PA_MAX_UK 0x88 // 10mW (max 434)
#define PA_OFF_BOOST 0x00
#define RFO_MIN 0x00

// 20DBm
#define REG_PA_DAC 0x4D
#define PA_DAC_20 0x87

// LOW NOISE AMPLIFIER
#define REG_LNA 0x0C
#define LNA_MAX_GAIN 0x23 // 0010 0011
#define LNA_OFF_GAIN 0x00

class CBLora {
public:
    CBLora();
    void setupLora(float Frequency, int Mode, char *Callsign);
    void checkLora(struct TGPS *GPS);
    void sendLoraPacket(unsigned char *buffer, int Length, int CallingPacket);

private:
};

} // namespace Codebrane

#endif // CBLORA_H
