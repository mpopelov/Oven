/**
 * @file TSensor.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief An adaptation of MAX31855 K temperature sensor library from different sources
 * @details Code adapted from great libraries by
 *          Adafruit: https://github.com/adafruit/Adafruit-MAX31855-library.git
 *          enjoyneering: https://github.com/enjoyneering/MAX31855.git
 *          Linearization code adapted from https://github.com/heypete/MAX31855-Linearization.git originally by heypete
 *          More details on linearization: https://blog.heypete.com/2016/09/09/max31855-temperature-linearization/
 * @version 0.1
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TSensor_h
#define TSensor_h

#include <math.h>
#include <Arduino.h>
#include <SPI.h>

#define TSENSOR_SPI_FREQUENCY   5000000     // SPI frequency in Hz, 5Mhz max according to MAX31855 specification
#define TSENSOR_CONVERSION_TIME 100         // milliseconds, time required for MAX31855 to complete measurement

#define TSENSOR_RES_PROBE       0.25        // chip PROBE ADC temperature resolution = 0.25 C / step
#define TSENSOR_RES_AMBIENT     0.0625      // chip AMBIENT ADC temperature resolution = 0.0625 / step

#define TSENSOR_SENSITIVITY_MV  0.041276    // chip sensitivity in mV/C - from MAX31855 datasheet for K-type sensor

#define TSENSOR_ERR_NC          0b001       // bit mask - chip error: thermocouple not connected
#define TSENSOR_ERR_GND         0b010       // bit mask - chip error: thermocouple shorted to ground
#define TSENSOR_ERR_VCC         0b100       // bit mask - chip error; thermocouple shorted to VCC


class TSensor
{
 public:
    TSensor(uint8_t cs, bool reversed) : valRaw(0), valProbe(NAN), valAmbient(NAN), _error(0), _cs(cs), _reversed(reversed)
    {
        pinMode(_cs, OUTPUT);    // Make the chip select pin output
        digitalWrite(_cs, HIGH); // High means ignore master, chip starts measuring
        SPI.begin();
    }

    uint8_t     readChip(); // reads raw value from chip over SPI bus and parses values. returns 0 on success or error code if any
    uint32_t    getRaw() { return valRaw; } // getter for raw value
    double      getProbe() { return valProbe; } // gets probe temperature from last reading upscaled 1000 times
    double      getAmbient() { return valAmbient; } // gets ambient temperature from last reading upscaled 1000 times
    uint8_t     getError() {return _error; } // returns error bits from last reading
    double      getProbeLinearized(); // returnes linearized probe temperature reading for K type thermocouple

 private:
    uint32_t    valRaw; // raw value read from the chip
    double      valProbe; // probe temperature decoded from Raw
    double      valAmbient; // ambien temperature decoded from Raw value
    uint8_t     _error; // error value decoded from chip Raw value
    uint8_t     _cs; // CS pin to be used for tackling the chip
    bool        _reversed; // a flag to indicate that K-thermocouple pins have been reversed (reverse polarity)

    // below are the the static arrays of coefficients derived from NIST dtabase found at
    // http://srdata.nist.gov/its90/download/type_k.tab
    static const double c_positive[];
    static const double c_a0;
    static const double c_a1;
    static const double c_a2;
    static const double c_negative[];
    static const double d_200_0[];
    static const double d_0_500[];
    static const double d_500_1372[];
};

// number of positive and negative coeffitients (arrays aligned to 11) - meaning one extra step in calculation for positives
#define TSENSOR_COUNT_CCOEF 11
// number of voltage-to-temperature coeffitients
#define TSENSOR_COUNT_DCOEF 10

// voltage ranges to apply correct sets of coeffitients
#define TSENSOR_VOLTAGE_0       0.0
#define TSENSOR_VOLTAGE_500     20.644
#define TSENSOR_VOLTAGE_1372    54.886

#endif
