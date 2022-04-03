/**
 * @file TSensor.cpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief TSensor implementation
 * @version 0.1
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "TSensor.h"

/**
 * @brief Reads raw temperature value from MAX31855 chip over SPI and updates all temperature values
 * 
 * @return uint8_t 
 */
uint8_t TSensor::readChip()
{
    // TODO:
    //
    // 1) use 1nF capacitor on thermocouple input - test if fluctuations are still rather big
    // 2) try toggling measurement at right before reading to prevent accumulated error that may be introduced during long measurement intervals
    // 3) implement linearisation / temperature compensation for better measurements
    // 4) implement average calculation based on 3/5 consecutive readings
    //
    int32_t dataBuffer = 0; // buffer for reading from chip

    SPI.beginTransaction(SPISettings(TSENSOR_SPI_FREQUENCY,MSBFIRST,SPI_MODE0)); // Start transaction at 5MHz MSB
    digitalWrite(_cs,LOW); // trigger data transfer from MAX31855
    
    // read 4 data bytes
    dataBuffer   = SPI.transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= SPI.transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= SPI.transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= SPI.transfer(0);

    digitalWrite(_cs,HIGH);         // deactivate MAX31855 to no longer send anything on SPI
    SPI.endTransaction();

    // save raw value
    valRaw = dataBuffer;

    // try decoding error code
    _error = (uint32_t)dataBuffer & B111; // Mask fault code bits

    // read temperatures
    valAmbient = INT32_MAX; // set ambient to max value to indicate an error
    valProbe = INT32_MAX;
    if(!_error){
        // there were no error - can try to decode

        // 1) mask out the ambient temperature value and decode it
        dataBuffer = (dataBuffer & 0xFFFF) >> 4;
        if(dataBuffer & 0x2000) dataBuffer |= 0xFFFF000;  // 2s complement bits if negative
        valAmbient = (int32_t)dataBuffer*(int32_t)625/(int32_t)10; // Sensitivity is 0.0625 C

        // 2) restore raw value read
        dataBuffer = valRaw;

        // 3) mask out and decode probe temperature
        dataBuffer = dataBuffer >> 18;
        if(dataBuffer & 0x2000) dataBuffer |= 0xFFFE000; // 2s complement bits if negative
        valProbe = (int32_t)dataBuffer * (int32_t)250;   // Sensitivity is 0.25 C
        // 4) adjust probe temperature in case its connection is reversed
        if(_reversed){
            valProbe = (valAmbient - valProbe) + valAmbient;
        }
    }

    return _error; // return error value explicitly to be able to check the results
}