/**
 * @brief JSconf.cpp
 * 
 */

#include "JSconf.h"

const char* JSConf::TOKEN_POLL                  = "poll";

const char* JSConf::TOKEN_TFT                   = "TFT";

const char* JSConf::TOKEN_WIFI                  = "WiFi";
const char* JSConf::TOKEN_WIFI_SSID             = "SSID";
const char* JSConf::TOKEN_WIFI_KEY              = "KEY";
const char* JSConf::TOKEN_WIFI_IP               = "IP";

const char* JSConf::TOKEN_PID                   = "PID";
const char* JSConf::TOKEN_PID_KP                = "KP";
const char* JSConf::TOKEN_PID_KI                = "KI";
const char* JSConf::TOKEN_PID_KD                = "KD";
const char* JSConf::TOKEN_PID_TOLERANCE         = "TOL";

const char* JSConf::TOKEN_PROGRAMS              = "Programs";
const char* JSConf::TOKEN_PROGRAM_NAME          = "Name";
const char* JSConf::TOKEN_PROGRAM_STEPS         = "steps";
const char* JSConf::TOKEN_PROGRAM_STEP_TSTART   = "tStart";
const char* JSConf::TOKEN_PROGRAM_STEP_TEND     = "tEnd";
const char* JSConf::TOKEN_PROGRAM_STEP_DURATION = "duration";

const char* JSConf::FILE_CONFIGURATION          = "/oven.json";
const char* JSConf::FILE_PROGRAMS               = "/programs.json";

/**
 * @brief Updates running controller configuration from a given JsonObject
 * 
 * @param joConfig JSON object containing configuration parameters
 * @param startup indicates whether configuration is initialized at startup or to be updated while controller is running
 */
void JSConf::UpdateRunningConfig(JsonObject& joConfig, bool startup){
  
  // a. read TFT parameters
  if( JsonObject obj = joConfig[TOKEN_TFT] ){
    // TFT section is present in the document
    
    // set TFT touch sensor polling interval
    this->TFT.poll = obj[TOKEN_POLL] | DEFAULT_TFT_POLL;

    // calibration data should only be set during startup
    if(startup){
    // try to read TS parameters saved earlier
      if( JsonArray arr = obj[TOKEN_TFT] ){
        if( arr.size() == 3 ){
          // try parse data from JSON config
          this->TFT.data.tft[0] = arr[0] | 0;
          this->TFT.data.tft[1] = arr[1] | 0;
          this->TFT.data.tft[2] = arr[2] | 0;
        } // else - leave calibration data as is
      }
    }
  }
  if(!startup) yield();
        
  // b. read WiFi parameters
  if( JsonObject obj = joConfig[TOKEN_WIFI] ){
    // WiFi section is present in the document
    this->WiFi.SSID = obj[TOKEN_WIFI_SSID].as<const char*>();
    this->WiFi.KEY = obj[TOKEN_WIFI_KEY].as<const char *>();
  }
  if(!startup) yield();

  // c. read PID parameters
  if( JsonObject obj = joConfig[TOKEN_PID] ){
    // PID section is present in the document
    this->PID.poll = obj[TOKEN_POLL] | DEFAULT_PID_POLL;
    this->PID.KP = obj[TOKEN_PID_KP] | DEFAULT_PID_PRM;
    this->PID.KI = obj[TOKEN_PID_KI] | DEFAULT_PID_PRM;
    this->PID.KD = obj[TOKEN_PID_KD] | DEFAULT_PID_PRM;
    this->PID.TOL = obj[TOKEN_PID_TOLERANCE] | DEFAULT_PID_PRM;
  }
}

/**
 * @brief Updates running controller programs from a given JsonArray
 * 
 * @param jaPrograms JSON array containing programs
 * @param startup indicates whether configuration is initialized at startup or to be updated while controller is running
 */
void JSConf::UpdateRunningPrograms(JsonArray& jaPrograms, bool startup){
  int nPrograms = jaPrograms.size();
  // limit the number of programs that might be in the configuration file to built-in max number of programs
  nPrograms = nPrograms > DEFAULT_MAX_PROGRAMS ? DEFAULT_MAX_PROGRAMS : nPrograms;

  // array contains elements - read up to a maximum of DEFAULT_MAX_PROGRAMS
  for(int i = 0; i < nPrograms; i++){
    // program might be malformed in JSON for some reason - make sure it is not pointing anywhere
    if( JsonObject pobj = jaPrograms[i] ){
      // try reading steps - create program only in case steps are defined
      if(JsonArray sarr = pobj[TOKEN_PROGRAM_STEPS]){
        // steps are defined - create program and try populating it with steps
        int nSteps = sarr.size();

        // make sure to read to a maximum of TPGM_STEPS_MAX
        nSteps = nSteps > TPGM_STEPS_MAX ? TPGM_STEPS_MAX : nSteps;
        // set program name
        this->Programs[i].SetName(pobj[TOKEN_PROGRAM_NAME] | "");

        // read and add every step we can accomodate
        for( int j = 0; j < nSteps; j++){
          if( JsonObject sobj = sarr[j] ){
            // add step in case it is represented as a valid JSON object
            this->Programs[i].AddStep(sobj[TOKEN_PROGRAM_STEP_TSTART] | 0.0,
                                              sobj[TOKEN_PROGRAM_STEP_TEND] | 0.0,
                                              sobj[TOKEN_PROGRAM_STEP_DURATION] | 0 );
          }
        }
        this->Programs[i].Reset(); // reset program to ready state
      }
    }

    // let controller do some stuff while reading programs might take a while
    if(!startup) yield();
  } // done reading programs

  // adjust number of programs read
  this->nPrograms = nPrograms;
}

/**
 * @brief Given a reference to JSON object populates it with current running configuration
 * 
 * @param joConfig a valid JsonObject that will be modified and filled with configuration values
 */
void JSConf::BuildRunningConfig(JsonObject& joConfig){
  // a valid JsonObject is expected with enough space allocated to hold configuration
  //
  // NB! when "String" values are added to JsonObject/Array - use c_str() (i.e. const char*) to prevent ArduinoJson from copying those.
  //     It should be safe to do so: configuration string values are not modified from Async*** library thread.
  //     As we are on the loop thread - nothing else can modify these strings.
  //     As we are serializing into the Async*** buffer in the end there is no worry values will change before actually sent over WiFi

  // add TFT information
  JsonObject joTFT = joConfig[TOKEN_TFT].to<JsonObject>();
  joTFT[TOKEN_POLL] = this->TFT.poll;
  JsonArray jaTFT = joTFT[TOKEN_TFT].to<JsonArray>();
  jaTFT.add(this->TFT.data.tft[0]);
  jaTFT.add(this->TFT.data.tft[1]);
  jaTFT.add(this->TFT.data.tft[2]);

  // add WiFi information
  JsonObject joWiFi = joConfig[TOKEN_WIFI].to<JsonObject>();
  joWiFi[TOKEN_WIFI_SSID] = this->WiFi.SSID.c_str();
  joWiFi[TOKEN_WIFI_KEY] = this->WiFi.KEY.c_str();

  // add PID information
  JsonObject joPID = joConfig[TOKEN_PID].to<JsonObject>();
  joPID[TOKEN_POLL] = this->PID.poll;
  joPID[TOKEN_PID_KP] = this->PID.KP;
  joPID[TOKEN_PID_KI] = this->PID.KI;
  joPID[TOKEN_PID_KD] = this->PID.KD;
  joPID[TOKEN_PID_TOLERANCE] = this->PID.TOL;

  // end building up configuration
}

/**
 * @brief Given a reference to valid JsonArray populates it with currently loaded oven programs
 * 
 * @param jaPrograms a valid JsonArray that will be modified and filled with information about programs
 */
void JSConf::BuildRunningPrograms(JsonArray& jaPrograms){
  // add programs array if there are any programs in currently loaded configuration
  if(this->nPrograms > 0){
    // crate object in array for each program
    for(int i = 0; i < this->nPrograms; i++){
      JsonObject joProgram = jaPrograms.add<JsonObject>();

      // set current program name
      joProgram[TOKEN_PROGRAM_NAME] = this->Programs[i].GetName(); // prevent copying string

      // add steps array
      JsonArray jaSteps = joProgram[TOKEN_PROGRAM_STEPS].to<JsonArray>();

      // for each step create nested object and set values
      for(int j = 0; j < this->Programs[i].GetStepsTotal(); j++){
        // create step and set values
        JsonObject joStep = jaSteps.add<JsonObject>();
        TProgramStep* currStep = this->Programs[i].GetStep(j);

        joStep[TOKEN_PROGRAM_STEP_TSTART] = currStep->GetTStart();
        joStep[TOKEN_PROGRAM_STEP_TEND] = currStep->GetTEnd();
        joStep[TOKEN_PROGRAM_STEP_DURATION] = currStep->GetDuration();
      }
    }
  }
}
