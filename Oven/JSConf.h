/**
 * @file JSconf.hpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief Classes and constants relevant for controller configuration (in-memory and JSON file)
 * @version 0.1
 * @date 2024-01-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#ifndef JSconf_h
#define JSconf_h

#include <Arduino.h>

#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>

#include <TProgram.h>

#define DEFAULT_TFT_POLL 300    // default TFT touch screen polling interval
#define DEFAULT_PID_POLL 1000   // default PID polling interval
#define DEFAULT_PID_PRM  1.0    // defailt value for PID coeffitients
#define DEFAULT_MAX_PROGRAMS 10 // maximum number of different programs a controller can save im memory

class JSConf {
  public:
    static const char* TOKEN_TFT;                      // TFT configuration object and touchscreen configuration data array names
    static const char* TOKEN_POLL;                     // TFT touchscreen poll interval

    static const char* TOKEN_WIFI;                     // WiFi connection configuration object name
    static const char* TOKEN_WIFI_SSID;                // WiFi SSID
    static const char* TOKEN_WIFI_KEY;                 // WiFi PSK
    static const char* TOKEN_WIFI_IP;                  // IP address (if to be static)

    static const char* TOKEN_PID;                      // PID control configuration object name
    static const char* TOKEN_PID_KP;                   // PID proportional coeffitient
    static const char* TOKEN_PID_KI;                   // PID integral coeffitient
    static const char* TOKEN_PID_KD;                   // PID differential coeffitient
    static const char* TOKEN_PID_TOLERANCE;            // PID tolerance value

    static const char* TOKEN_PROGRAMS;                 // Programs array
    static const char* TOKEN_PROGRAM_NAME;             // Program name
    static const char* TOKEN_PROGRAM_STEPS;            // Program steps array
    static const char* TOKEN_PROGRAM_STEP_TSTART;      // Program step start temperature
    static const char* TOKEN_PROGRAM_STEP_TEND;        // Program step end temperature
    static const char* TOKEN_PROGRAM_STEP_DURATION;    // Program step duration

    static const char* FILE_CONFIGURATION;
    static const char* FILE_PROGRAMS;

    int nPrograms = 0;
    TProgram Programs[DEFAULT_MAX_PROGRAMS];
    
    JSConf(){}
    void UpdateRunningConfig(JsonObject& joConfig, bool startup);
    void UpdateRunningPrograms(JsonArray& jaPrograms, bool startup);
    void BuildRunningConfig(JsonObject& joConfig);
    void BuildRunningPrograms(JsonArray& jaPrograms);

    struct _sTFT {
        unsigned long poll = DEFAULT_TFT_POLL;  // poll touch screen that often (in ms)
        union _uData {
            uint32_t tft[3] = {0, 0, 0};
            uint16_t raw[6];
        } data;
    } TFT;

    struct _sWiFi {
        String SSID;
        String KEY;
        String IP;
    } WiFi;

    struct _sPID {
        unsigned long poll = DEFAULT_PID_POLL;  // only measure temperature that often (in ms)
        double KP = DEFAULT_PID_PRM;
        double KI = DEFAULT_PID_PRM;
        double KD = DEFAULT_PID_PRM;
        double TOL = DEFAULT_PID_PRM;
    } PID;
};



#endif