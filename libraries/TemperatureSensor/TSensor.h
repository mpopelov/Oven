/**
 * @file TSensor.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief An adaptation of MAX31855 temperature sensor library from different sources
 * @version 0.1
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TSensor_h
#define TSensor_h

#include <SPI.h>

#define TSENSOR_SPI_FREQUENCY   5000000 // SPI frequency in Hz, 5Mhz max according to MAX31855 specification
#define TSENSOR_CONVERSION_TIME 100     // milliseconds, time required for MAX31855 to complete measurement

#define TSENSOR_ERR_NC          0b001   // bit mask - chip error: thermocouple not connected
#define TSENSOR_ERR_GND         0b010   // bit mask - chip error: thermocouple shorted to ground
#define TSENSOR_ERR_VCC         0b100   // bit mask - chip error; thermocouple shorted to VCC


class TSensor
{
private:
    uint32_t    valRaw; // raw value read from the chip
    int32_t     valProbe; // probe temperature decoded from Raw
    int32_t     valAmbient; // ambien temperature decoded from Raw value
    uint8_t     _error; // error value decoded from chip Raw value
    uint8_t     _cs; // CS pin to be used for tackling the chip
    bool        _reversed; // a flag to indicate that K-thermocouple pins have been reversed (reverse polarity)
public:
    TSensor(uint8_t cs, bool reversed) : valRaw(0), valProbe(0), valAmbient(0), _error(0), _cs(cs), _reversed(reversed)
    {
        pinMode(_cs, OUTPUT);    // Make the chip select pin output
        digitalWrite(_cs, HIGH); // High means ignore master
        SPI.begin();
    };
    uint8_t     readChip(); // reads raw value from chip over SPI bus
    uint32_t    getRaw() { return valRaw; }
    int32_t     getProbe() { return valProbe; }
    int32_t     getAmbient() { return valAmbient; }
    uint8_t     getError() {return _error; }
};

#endif
