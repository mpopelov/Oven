/**
 * @file TSensor.cpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief TSensor implementation
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


#include "TSensor.h"

/**
 * @brief Reads raw temperature value from MAX31855 chip over SPI and updates all temperature values
 * 
 * @return 0 on success or error bits if set while reding
 */
uint8_t TSensor::readChip()
{
    // TODO:
    //
    // 1) use 10nF/0.01mF capacitor on thermocouple input - test if fluctuations are still rather big
    // 2) try toggling measurement at right before reading to prevent accumulated error that may be introduced during long measurement intervals
    // + 3) implement linearisation / temperature compensation for better measurements
    // 4) implement average calculation based on 3/5 consecutive readings
    //
    int32_t dataBuffer = 0; // buffer for reading from chip

    _spi->beginTransaction(SPISettings(TSENSOR_SPI_FREQUENCY, MSBFIRST, SPI_MODE0)); // Start transaction at 5MHz MSB
    digitalWrite(_cs,LOW); // trigger data transfer from MAX31855
    
    // read 4 data bytes
    dataBuffer   = _spi->transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= _spi->transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= _spi->transfer(0);
    dataBuffer <<= 8;
    dataBuffer  |= _spi->transfer(0);

    // TODO: replace it with a single call to use DMA:
    //dataBuffer = _spi->transfer32(0);

    digitalWrite(_cs,HIGH);         // deactivate MAX31855 to no longer send anything on SPI
    _spi->endTransaction();

    // save raw value
    valRaw = dataBuffer;

    // try decoding error code
    _error = (uint32_t)dataBuffer & B111; // Mask fault code bits

    // read temperatures
    valAmbient = NAN;   // set ambient temperature to NAN to indicate an error
    valProbe = NAN;     // set probe temperature to NAN to indicate an error 
    if(!_error){
        // there were no error - can try to decode

        // 1) mask out the ambient temperature value and decode it
        dataBuffer = (dataBuffer & 0xFFFF) >> 4;
        if(dataBuffer & 0x2000) dataBuffer |= 0xFFFF000;    // 2s complement bits if negative
        valAmbient = dataBuffer * TSENSOR_RES_AMBIENT;      // Sensitivity is 0.0625 C

        // 2) restore raw value read
        dataBuffer = valRaw;

        // 3) mask out and decode probe temperature
        dataBuffer = dataBuffer >> 18;
        if(dataBuffer & 0x2000) dataBuffer |= 0xFFFE000;    // 2s complement bits if negative
        valProbe = dataBuffer * TSENSOR_RES_PROBE;          // Sensitivity is 0.25 C
        // 4) adjust probe temperature in case its connection is reversed
        if(_reversed){
            valProbe = (valAmbient - valProbe) + valAmbient;
        }
    }

    return _error; // return error value explicitly to be able to check the results
}

/**
 * @brief linearizes K type thermocouple sensor temperature populated by readChip
 * @details 
 * 
 * @return temperature value in Celicium
 */
double TSensor::getProbeLinearized()
{
    // Initialize variables.
    double thermocoupleVoltage= 0;
    double internalVoltage = 0;
    double correctedTemp = 0;
    double totalVoltage = 0;

    const double* coeff_array = nullptr; // pointer to appropriate static array of coeffitients
    double powT = 1.0; // variable to speed up calculation of pow(x,i) by accumulating value in the loop

    // Check to make sure thermocouple temperature is meaningful.
    if (isnan(valProbe)) return NAN;
    
    // Step 1. Subtract cold junction temperature from the raw thermocouple temperature
    // Step 2. Calculate thermocouple voltage based on chip sensitivity
    thermocoupleVoltage = (valProbe - valAmbient) * TSENSOR_SENSITIVITY_MV;  // C * mv/C = mV

    // Step 3. Calculate the cold junction equivalent thermocouple voltage.
    // Coefficients and equations available from http://srdata.nist.gov/its90/download/type_k.tab
    if(valAmbient >= 0){
        coeff_array = c_positive;
    } else{
        coeff_array = c_negative;
    }

    // From NIST:
    // - positive range: E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2)
    // - negative range: E = sum(i=0 to n) c_i t^i
    // where E is the thermocouple voltage in mV and t is the temperature in degrees C.
    // In this case, E is the cold junction equivalent thermocouple voltage.
    // Alternative form: C0 + C1*internalTemp + C2*internalTemp^2 + C3*internalTemp^3 + ... + C10*internaltemp^10 + A0*e^(A1*(internalTemp - A2)^2)
    // This loop sums up the c_i t^i components.
    for( int i=0; i < TSENSOR_COUNT_CCOEF; i++){
        // original code: internalVoltage += c[i] * pow(valAmbient, i);
        internalVoltage += coeff_array[i] * powT;
        powT *= valAmbient;
    }

    // one more step needed for positive ambient temperature - adding exponential component a0 exp(a1 (t - a2)^2)
    if(valAmbient >= 0) internalVoltage += c_a0 * exp(c_a1 * pow((valAmbient - c_a2), 2));

    // Step 4. Add the cold junction equivalent thermocouple voltage calculated in step 3 to the thermocouple voltage calculated in step 2.
    totalVoltage = thermocoupleVoltage + internalVoltage;

    // Step 5. Use the result of step 4 and the NIST voltage-to-temperature (inverse) coefficients
    // to calculate the cold junction compensated, linearized temperature value.
    // The equation is in the form correctedTemp = d_0 + d_1*E + d_2*E^2 + ... + d_n*E^n,
    // where E is the totalVoltage in mV and correctedTemp is in degrees C.
    // NIST uses different coefficients for different temperature subranges: (-200 to 0C), (0 to 500C) and (500 to 1372C).

    // if voltage is beyond top boundary - return NAN
    if(totalVoltage >= TSENSOR_VOLTAGE_1372) return NAN;

    // tune pointer to the correct array
    if(totalVoltage >= TSENSOR_VOLTAGE_500){
        coeff_array = d_500_1372;
    } else if(totalVoltage >= TSENSOR_VOLTAGE_0){
        coeff_array = d_0_500;
    } else{
        coeff_array = d_200_0;
    }

    // Apply transformation of voltage to temperature.
    // Apply optimization: accumulate totalVoltage^i instead of using pow() every step.
    // As number^[i=0] = 1 start with 1 and multiply at the end of each step to prepare for the next one
    powT = 1.0;
    for(int i=0; i < TSENSOR_COUNT_DCOEF; i++){
        // original code: correctedTemp += d[i] * pow(totalVoltage, i);
        correctedTemp += coeff_array[i] * powT;
        powT *= totalVoltage;
    }

    // return corrected temperature
    return correctedTemp;
}


/**
 * @brief Below are definitions for static member arrays of coeffitients
 * 
 */


// positive AMBIENT voltage linearization coeffitients initializer from NIST database
const double TSensor::c_positive[] = {-0.176004136860E-01,
                                       0.389212049750E-01,
                                       0.185587700320E-04,
                                      -0.994575928740E-07,
                                       0.318409457190E-09,
                                      -0.560728448890E-12,
                                       0.560750590590E-15,
                                      -0.320207200030E-18,
                                       0.971511471520E-22,
                                      -0.121047212750E-25,
                                       0.000000000000E+00};

// additional exponential coeffitients for positive voltage range
const double TSensor::c_a0 =  0.118597600000E+00;
const double TSensor::c_a1 = -0.118343200000E-03;
const double TSensor::c_a2 =  0.126968600000E+03;

// negative AMBIENT voltage linearization coeffitients initializer from NIST database
const double TSensor::c_negative[] = {0.000000000000E+00,
                                      0.394501280250E-01,
                                      0.236223735980E-04,
                                     -0.328589067840E-06,
                                     -0.499048287770E-08,
                                     -0.675090591730E-10,
                                     -0.574103274280E-12,
                                     -0.310888728940E-14,
                                     -0.104516093650E-16,
                                     -0.198892668780E-19,
                                     -0.163226974860E-22};

// voltage-to-temperature coeffitients for -200 to 0 range
const double TSensor::d_200_0[] = {0.0000000E+00,
                                   2.5173462E+01,
                                  -1.1662878E+00,
                                  -1.0833638E+00,
                                  -8.9773540E-01,
                                  -3.7342377E-01,
                                  -8.6632643E-02,
                                  -1.0450598E-02,
                                  -5.1920577E-04,
                                   0.0000000E+00};

// voltage-to-temperature coeffitients for 0 to 500 range
const double TSensor::d_0_500[] = {0.000000E+00,
                                   2.508355E+01,
                                   7.860106E-02,
                                  -2.503131E-01,
                                   8.315270E-02,
                                  -1.228034E-02,
                                   9.804036E-04,
                                  -4.413030E-05,
                                   1.057734E-06,
                                  -1.052755E-08};

// voltage-to-temperature coeffitients for 500 to 1372 range
const double TSensor::d_500_1372[] = {-1.318058E+02,
                                       4.830222E+01,
                                      -1.646031E+00,
                                       5.464731E-02,
                                      -9.650715E-04,
                                       8.802193E-06,
                                      -3.110810E-08,
                                       0.000000E+00,
                                       0.000000E+00,
                                       0.000000E+00};