/**
 * @file Oven.ino
 * @author MikeP (mpopelov@gmail.com)
 * @brief Oven project - a controller for electrical heat oven
 * @details Implementation of a controller for electrical heat oven.
 *          Controller is able to execute production programs stored in memory.
 *          Each program defines a list of steps identifying desired temperatures at the start and end of each step along with step duration.
 *          Linear temperature change over step duration is assumed.
 *          Controller can be managed using TFT touchscreen to select/start/stop programs stored in memory.
 *          Configuring controller parameters and available programs is done via web interface.
 *          Web interface also allows the same level of control as TFT touchscreen.
 *          To be able to access web interface of the controller it should join WiFi network.
 * @version 0.1
 * @date 2022-06-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include <SPI.h>                  // SPI for accessing devices on SPI bus
#include <TFT_eSPI.h>             // included for TFT support
#include <LittleFS.h>             // included for file system support

#include <ESP8266WiFi.h>          // included for WiFi support
#include <ESPAsyncTCP.h>          // included for async TCP communication
#include <ESPAsyncWebServer.h>    // included for HTTP and WebSocket support

#include <ArduinoJson.h>          // included for JSON support

#include <DeskTop.h>              // included for simple desktop class library
#include <TSensor.h>              // included for K-type temperature sensor support
#include <PIDControl.h>           // included for PID-control implementation
#include <TProgram.h>             // included for base classes describing controller programs

/**
 * Fonts to be used for rendering on TFT display
 */
#define FSN1 FreeSans9pt7b      // for status text and small labels
#define FSB1 FreeSansBold9pt7b  // bold font for small text
#define FSN2 FreeSans12pt7b     // for medium sized text
#define FSB2 FreeSansBold12pt7b // for medium sized text in bold
#define FSB3 FreeSansBold18pt7b
#define FSB4 FreeSansBold24pt7b

#define DEFAULT_TFT_POLL 300    // default TFT touch screen polling interval
#define DEFAULT_PID_POLL 1000   // default PID polling interval
#define DEFAULT_PID_PRM  1.0    // defailt value for PID coeffitients

/**
 * Global string constants
 */
// file names and paths (also for web server)
static const char FILE_CONFIGURATION[] PROGMEM = "oven.json";
static const char FILE_WEB_INDEX[]             = "index.html";
static const char PATH_WEB_ROOT[]              = "/";
static const char PATH_WEB_HEAP[]              = "/heap";
static const char PATH_WEB_WS[] PROGMEM        = "/ws";
static const char FILE_WEB_CT_TXT[] PROGMEM    = "text/plain";

// button text strings
static const char BTN_START[] PROGMEM         = "Start";
static const char BTN_STOP[] PROGMEM          = "Stop";
static const char BTN_PROG[] PROGMEM          = "Prog";
// label text strings
static const char LBL_EMPTY[] PROGMEM         = "---";
static const char LBL_TEMPREMPTY[] PROGMEM    = "----C";
static const char LBL_TIMEREMPTY[] PROGMEM    = "00:00:00";
static const char LBL_PROG[] PROGMEM          = "Program: ";
static const char LBL_STEP[] PROGMEM          = "Step: ";
static const char LBL_OF[] PROGMEM            = " of ";
static const char LBL_STEPTARGET[] PROGMEM    = "Step target: ";
static const char LBL_STEPTIME[] PROGMEM      = "Step remaining: ";
static const char LBL_TIMEREMAINING[] PROGMEM = "Program remaining: ";
static const char LBL_DEGC[] PROGMEM = " C";

// JSON message / configuration elements and defines
#define JSON_BUFFER_MAX_SIZE 8192   // maximum message/buffer size
#define JSON_DOCUMENT_MAX_SIZE 8192 // maximum size of JSON document allowed

static const char JSCONF_POLL[] PROGMEM                   = "poll";

static const char JSCONF_TFT[] PROGMEM                    = "TFT";

static const char JSCONF_WIFI[] PROGMEM                   = "WiFi";
static const char JSCONF_WIFI_SSID[] PROGMEM              = "SSID";
static const char JSCONF_WIFI_KEY[] PROGMEM               = "KEY";
static const char JSCONF_WIFI_IP[] PROGMEM                = "IP";

static const char JSCONF_PID[] PROGMEM                    = "PID";
static const char JSCONF_PID_KP[] PROGMEM                 = "KP";
static const char JSCONF_PID_KI[] PROGMEM                 = "KI";
static const char JSCONF_PID_KD[] PROGMEM                 = "KD";
static const char JSCONF_PID_TOLERANCE[] PROGMEM          = "TOL";

static const char JSCONF_PROGRAMS[] PROGMEM               = "Programs";
static const char JSCONF_PROGRAM_NAME[] PROGMEM           = "Name";
static const char JSCONF_PROGRAM_STEPS[] PROGMEM          = "steps";
static const char JSCONF_PROGRAM_STEP_TSTART[] PROGMEM    = "tStart";
static const char JSCONF_PROGRAM_STEP_TEND[] PROGMEM      = "tEnd";
static const char JSCONF_PROGRAM_STEP_DURATION[] PROGMEM  = "duration";

static const char JSON_RESPONSE_TEMPLATE_OK[]             = "{\"id\":\"OK\",\"details\":\"%s\"}";
static const char JSON_RESPONSE_TEMPLATE_ERR[]            = "{\"id\":\"ERR\",\"details\":\"%s\"}";


/**
 * @brief Global configuration
 * 
 */
struct _sConfiguration {

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

  int nPrograms = 0;
  TProgram** Programs = nullptr;

} Configuration;

/**
 * @brief Global controller state
 * 
 */
struct _sState {

  double tProbe = NAN;                        // current temperature as reported by probe
  double tAmbient = NAN;                      // current ambient temperature
  double tSP = NAN;                           // current set point temperature
  double U = 0.0;                             // current controlling signal value

  TProgram* volatile ActiveProgram = nullptr; // current active program

  unsigned long ticks_TS = 0;                 // timestamp of last touchscreen polling
  unsigned long ticks_TSENSOR = 0;            // timestamp of last temperature sensor polling
  unsigned long ticks_PGM = 0;                // time elapsed since the start of current program step
  unsigned long ticks_PGM_Total = 0;          // time elapsed since current program start

  volatile bool StartStop = false;            // a flag to signal main loop to start (true) or stop (false) selected program.
  bool isPgmRunning = false;                  // gloabl flag for signalling controller program state
  bool isRelayOn = false;                     // flag to indicate that the relay is turned on

} State;

/**
 * @brief Structure to handle incoming JSON messages
 * @details Structure holds all the information relevant to process request and prepare response
 *          over WebSocket.
 *          Status field is used to mark message valid for processing in the main loop
 *          WebSocket event handler is allowed to set Status to any value.
 *          main loop is only allowed to reset status to EMPTY
 *          field is set volatile in order to prevent optimization/caching across different threads
 * 
 */
typedef enum {WSJSONMSG_EMPTY = 0, WSJSONMSG_INPROGRESS, WSJSONMSG_READY} WsJSONMsgStatus;
struct _sWsJSONMsg {
  volatile WsJSONMsgStatus status = WSJSONMSG_EMPTY; // mark volatile to make sure it is read explicitly upon access as structure can be accessed from another thread
  uint32_t      client_id = 0; // requesting client Id
  size_t        datalen = 0; // current data length 
  char          buffer[JSON_BUFFER_MAX_SIZE];
} gi_WsJSONMsg;

/**
 * Global instances
 */
TFT_eSPI          gi_Tft{};                         // TFT display driver
LittleFSConfig    gi_FSConfig{};                    // file system configuration

TSensor           gi_TS{D2, false};                 // MAX31855 K-type temperature sensor instance
// Enable only one PID control instance
PIDControlBasic   gp_PID{};
//PIDControlSimple  gp_PID{};
//PIDControlIIR     gp_PID{};

AsyncWebServer    gi_WebServer{80};                 // a web server instance
AsyncWebSocket    gi_WebSocket{FPSTR(PATH_WEB_WS)}; // web socket for communicating with OvenWEB

StaticJsonDocument<JSON_DOCUMENT_MAX_SIZE> gi_JDoc;



/**
 * Classes for GUI windows used in sketch
 */

/**
 * @brief Initialization splash screen.
 * @details Shown during controller initialization to indicate current progress.
 *          - Shows progress bar
 *          - Shows current step explanation
 */
class cSplashScreenWindow : public DTWindow {
  public:
  cSplashScreenWindow(TFT_eSPI& gfx) :
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
  pbrProgress( gfx, 20, 116, 280,  8, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTPROGRESSBAR_BRDR_ON, DT_C_BACKGROUND, DT_C_GREY, DT_C_LIGHTGREEN ),
    lblStatus( gfx, 10, 126, 300, 25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND,  DT_C_RED, DT_C_LIGHTGREEN, &FSN1, (char*) nullptr)
  {
    lblStatus.reserve(32); // pre-allocate some space in order to avoid extra memory allocation for underlying string
    AddControl(&pbrProgress);
    AddControl(&lblStatus);
  }

  // controls
  DTProgressBar pbrProgress;
  DTLabel       lblStatus;
};

/**
 * @brief Window that allows selecting programs from memory.
 * @details Class is to be instantiated every time user clicks button in main window.
 *          Child select control is populated with available programs in constructor.
 *          TODO: implement handling of changes from web interface that may occur while this window is active
 *                so there is no discrepancy in what programs are available in memory and shown to user on TFT.
 * 
 */
class cPgmSelectWindow : public DTWindow {
  public:
  cPgmSelectWindow(TFT_eSPI& gfx, DTDelegate callback) :
  DTWindow( gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
      btnUp( gfx, 270,   0,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, F("UP"), DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Up>(this)),
      btnOk( gfx, 270,  51,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, F("OK"), DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Ok>(this)),
  btnCancel( gfx, 270, 102,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_RED, DT_C_BACKGROUND, &FSB2, F("X"), DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Cancel>(this)),
    btnDown( gfx, 270, 153,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, F("DN"), DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Down>(this)),
   selProgs( gfx,   0,   0, 269, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_LIGHTGREEN, DT_C_BACKGROUND, DT_C_BACKGROUND, DT_C_LIGHTGREEN, &FSN1),
  _Result(false), _callback(callback)
  {
    AddControl(&btnUp);
    AddControl(&btnOk);
    AddControl(&btnCancel);
    AddControl(&btnDown);
    AddControl(&selProgs);
    if(Configuration.nPrograms > 0 && Configuration.Programs != nullptr)
      for(int idx=0; idx < Configuration.nPrograms; idx++) selProgs.AddItem(idx, Configuration.Programs[idx]->GetName());
  }

  // callbacks for buttons
  void OnButton_Up() { selProgs.MovePrev(); }
  void OnButton_Down() { selProgs.MoveNext(); }
  void OnButton_Ok() { _Result = true; _callback(); }
  void OnButton_Cancel() { _Result = false; _callback(); }

  // public controls
  DTSelect selProgs;
  DTButton btnUp;
  DTButton btnOk;
  DTButton btnCancel;
  DTButton btnDown;

  // a variable to tell the result: ok/cancel
  bool _Result;
  // a callback to tell parent we are done
  DTDelegate _callback;
};

/**
 * @brief Main window class showing temperature, running program and a few buttons to have control over the device
 * 
 */
class cMainWindow : public DTWindow {
  public:
  cMainWindow(TFT_eSPI& gfx) :
  _PgmSelectWindowOn(false), SWnd(nullptr),
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
  // initialize child elements
              btnProg( gfx, 250,   0,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, FPSTR(BTN_PROG), DTDelegate::create<cMainWindow,&cMainWindow::OnButton_PGM>(this)),
             btnStart( gfx, 250,  51,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, FPSTR(BTN_START), DTDelegate::create<cMainWindow,&cMainWindow::OnButton_STRTSTP>(this)),
  // program details - Y offset 0
           lblProgram( gfx,   0,   0,  80,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_PROG)),
       lblProgramName( gfx,  80,   0, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_EMPTY)),
  // step details - Y offset 25
              lblStep( gfx,   0,  25,  50,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_STEP)),
        lblStepNumber( gfx,  50,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_EMPTY)),
            lblStepOf( gfx,  80,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_OF)),
         lblStepTotal( gfx, 110,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_EMPTY)),
  // temperature main display - Y offset 50
             lblTempr( gfx,   0,  50, 210, 110, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSB4, FPSTR(LBL_TEMPREMPTY)),
       lblTemprTarget( gfx, 135, 135,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_RED, &FSN1, FPSTR(LBL_TEMPREMPTY)),
  // step timing values - Y offset 160
          lblStepTime( gfx,   0, 160, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_STEPTIME)),
     lblStepTimeValue( gfx, 170, 160,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_TIMEREMPTY)),
  // displaying program timer - Y offset 185
       lblProgramTime( gfx,   0, 185, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_TIMEREMAINING)),
  lblProgramTimeValue( gfx, 170, 185,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, FPSTR(LBL_TIMEREMPTY)),
  // status label - Y offset 210
            lblStatus( gfx,   0, 210, 320,  30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, F("Initializing..."))
  {
    // Pre-allocate some space for labels that will have changing content
    // and their content is bigger than SSO buffer of String class.
    // Static text labels are ok as they are going to remain as is and most often fit into SSO buffer.
    // buttons should be ok with SSO built into String class on ESP8266

    lblProgramName.reserve(32);
    lblStepNumber.reserve(12);
    lblStatus.reserve(64);

    // Add controls to handling stack
    AddControl(&lblProgram);
    AddControl(&lblProgramName);
    AddControl(&lblStep);
    AddControl(&lblStepNumber);
    AddControl(&lblStepOf);
    AddControl(&lblStepTotal);
    AddControl(&lblTemprTarget);
    AddControl(&lblTempr);
    AddControl(&lblStepTime);
    AddControl(&lblStepTimeValue);
    AddControl(&lblProgramTime);
    AddControl(&lblProgramTimeValue);
    AddControl(&lblStatus);
    AddControl(&btnProg);
    AddControl(&btnStart);
  }

  // callback for select program window to indicate its result
  void OnSelect()
  {
    // either OK or Cancel was clicked - check the result and trigger child window deletion.
    _PgmSelectWindowOn = false;
    if(SWnd->_Result){
      // new program was selected
      // update active program with the one selected only if controller is not in running mode
      if(!State.isPgmRunning){
        uint16_t _pgmIdx = SWnd->selProgs.GetSelected();
        if(_pgmIdx < Configuration.nPrograms){
          // delete active program if it was there
          if(State.ActiveProgram != nullptr) delete State.ActiveProgram;

          // create a copy of selected program
          State.ActiveProgram = new TProgram(*Configuration.Programs[_pgmIdx]); // invoke copy constructor

          // set selected program details
          lblProgramName = State.ActiveProgram->GetName();
          lblStepTotal = String(State.ActiveProgram->GetStepsTotal());
          lblStepNumber = String(State.ActiveProgram->GetStepsCurrent());

          // show initial duration of selected program
          char buff[12];
          unsigned long t = State.ActiveProgram->GetDurationTotal();
          snprintf(buff, 12, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
          lblProgramTimeValue = buff;
          // end of adjusting controls
        }
      }
    }else{
      // just cancelled
    }
  }
  
  // callback for program select button
  void OnButton_PGM()
  {
    // make this window not visible and create child window
    Visible(false);
    _PgmSelectWindowOn = true;
    SWnd = new cPgmSelectWindow(_gfx, DTDelegate::create<cMainWindow,&cMainWindow::OnSelect>(this));
    SWnd->Invalidate();
  }

  // callback for start/stop button
  void OnButton_STRTSTP() {
    if(State.StartStop){
      // true - signal main loop to stop the program
      State.StartStop = false;
    }else{
      // false - signal main loop to start the program
      if(State.ActiveProgram != nullptr){
        // only start things if there is an active program selected
        // raise the flag and signal main loop to start executing
        State.StartStop = true;
      }
    }
  }

  // overload HandleEvent function
  virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed)
  {
    if(_PgmSelectWindowOn && SWnd != nullptr){
      //pass event to select window
      bool res = SWnd->HandleEvent(x, y, pressed);
      // select might have processed the event and could have signalled we shall destroy it
      if(!_PgmSelectWindowOn){
        // flag was re-set meaning we shall destroy child window
        delete SWnd;
        SWnd = nullptr;
        Visible(true);
        Invalidate();
      }
      return res;
    }else{
      // pass events to our own controls
      return DTWindow::HandleEvent(x, y, pressed);
    }
  }

  // overload Render function
  virtual void Render(bool parentCleared)
  {
    // properly route rendering call
    if(_PgmSelectWindowOn && SWnd != nullptr){
      SWnd->Render(parentCleared);
    }else{
      DTWindow::Render(parentCleared);
    }
  }
  
  /*
  / Members holding all control elements
  */
  // buttons
  DTButton btnProg;
  DTButton btnStart;
  
  // labels
  DTLabel lblTempr;             // current measured temperature
  DTLabel lblTemprTarget;       // current target temperature (SetPoint)
  DTLabel lblStatus;            // status text label at the bottom of the screen
  DTLabel lblProgram;           // Label "Program; "
  DTLabel lblProgramName;       // actual program name
  DTLabel lblStep;              // Label "Step: "
  DTLabel lblStepNumber;        // current step number
  DTLabel lblStepOf;            // Label "of: "
  DTLabel lblStepTotal;         // total number of steps

  DTLabel lblStepTime;          // total number of steps
  DTLabel lblStepTimeValue;     // total number of steps
  DTLabel lblProgramTime;       // total number of steps
  DTLabel lblProgramTimeValue;  // total number of steps

  bool _PgmSelectWindowOn;      // flag to indicate program selection window is active
  cPgmSelectWindow* SWnd;       // child window for selecting programs
};
cMainWindow wnd{gi_Tft}; // main window instance

/**
 * @brief Updates running controller configuration from a given JsonObject
 * 
 * @param joConfig JSON object containing configuration parameters
 * @param startup indicates whether configuration is initialized at startup or to be updated while controller is running
 */
void UpdateRunningConfig(JsonObject& joConfig, bool startup){
  
  // a. read TFT parameters
  if( JsonObject obj = joConfig[FPSTR(JSCONF_TFT)] ){
    // TFT section is present in the document
    
    // set TFT touch sensor polling interval
    Configuration.TFT.poll = obj[FPSTR(JSCONF_POLL)] | DEFAULT_TFT_POLL;

    // calibration data should only be set during startup
    if(startup){
    // try to read TS parameters saved earlier
      if( JsonArray arr = obj[FPSTR(JSCONF_TFT)] ){
        if( arr.size() == 3 ){
          // try parse data from JSON config
          Configuration.TFT.data.tft[0] = arr[0] | 0;
          Configuration.TFT.data.tft[1] = arr[1] | 0;
          Configuration.TFT.data.tft[2] = arr[2] | 0;
        } // else - leave calibration data as is
      }
    }
  }
  if(!startup) yield();
        
  // b. read WiFi parameters
  if( JsonObject obj = joConfig[FPSTR(JSCONF_WIFI)] ){
    // WiFi section is present in the document
    Configuration.WiFi.SSID = obj[FPSTR(JSCONF_WIFI_SSID)].as<const char*>();
    Configuration.WiFi.KEY = obj[FPSTR(JSCONF_WIFI_KEY)].as<const char *>();
  }
  if(!startup) yield();

  // c. read PID parameters
  if( JsonObject obj = joConfig[FPSTR(JSCONF_PID)] ){
    // PID section is present in the document
    Configuration.PID.poll = obj[FPSTR(JSCONF_POLL)] | DEFAULT_PID_POLL;
    Configuration.PID.KP = obj[FPSTR(JSCONF_PID_KP)] | DEFAULT_PID_PRM;
    Configuration.PID.KI = obj[FPSTR(JSCONF_PID_KI)] | DEFAULT_PID_PRM;
    Configuration.PID.KD = obj[FPSTR(JSCONF_PID_KD)] | DEFAULT_PID_PRM;
    Configuration.PID.TOL = obj[FPSTR(JSCONF_PID_TOLERANCE)] | DEFAULT_PID_PRM;
  }
  if(!startup) yield();

  // d. read programs
  if( JsonArray parr = joConfig[FPSTR(JSCONF_PROGRAMS)] ){
    // Programs section is present in the document
    // build a separate list of new incoming programs to replace in configuration afterwards
    int nPrograms = parr.size();
    if(nPrograms > 0){
      // array contains elements
      TProgram** Programs = new TProgram*[nPrograms];

      // try reading all of them
      for(int i = 0; i < nPrograms; i++){
        // program might be malformed in JSON for some reason - make sure it is not pointing anywhere
        Programs[i] = nullptr;
        if( JsonObject pobj = parr[i] ){
          // try reading steps - create program only in case steps are defined
          if(JsonArray sarr = pobj[FPSTR(JSCONF_PROGRAM_STEPS)]){
            // steps are defined - create program and try populating it with steps
            int nSteps = sarr.size();
            // create new program
            TProgram* p = new TProgram( nSteps, pobj[FPSTR(JSCONF_PROGRAM_NAME)] | String(F("InvalidName")) );
            // read and add every step
            for( int j = 0; j < nSteps; j++){
              if( JsonObject sobj = sarr[j] ){
                // add step in case it is represented as a valid JSON object
                p->AddStep( sobj[FPSTR(JSCONF_PROGRAM_STEP_TSTART)] | 0.0,
                            sobj[FPSTR(JSCONF_PROGRAM_STEP_TEND)] | 0.0,
                            sobj[FPSTR(JSCONF_PROGRAM_STEP_DURATION)] | 0 );
              }
            }
            p->Reset(); // reset program to ready state
            Programs[i] = p; // add it to an array of programs
          }
          // at this pont Programs[i] is either nullptr or a valid object
          if(!startup) yield();
        }
      }

      // now replace programs in configuration object
      // TODO: introduce sync lock in order to avoid problems when replacing programs
      // while user might be selecting them on TFT screen
      int old_nPrograms = Configuration.nPrograms;
      TProgram** old_Programs = Configuration.Programs;

      Configuration.nPrograms = nPrograms;
      Configuration.Programs = Programs;

      if(old_nPrograms > 0){
        for(int i = 0; i < old_nPrograms; i++) delete old_Programs[i];
        delete Configuration.Programs;
      }
    } // done reading programs
  }
}

/**
 * @brief Given a reference to JSON object populates it with current running configuration
 * 
 * @param joConfig a valid JsonObject that will be modified and filled with configuration values
 */
void BuildRunningConfig(JsonObject& joConfig){
  // a valid JsonObject is expected with enough space allocated to hold configuration
  //
  // NB! when "String" values are added to JsonObject/Array - use c_str() (i.e. const char*) to prevent ArduinoJson from copying those.
  //     It should be safe to do so: configuration string values are not modified from Async*** library thread.
  //     As we are on the loop thread - nothing else can modify these strings.
  //     As we are serializing into the Async*** buffer in the end there is no worry values will change before actually sent over WiFi

  // add TFT information
  JsonObject joTFT = joConfig.createNestedObject(FPSTR(JSCONF_TFT));
  joTFT[FPSTR(JSCONF_POLL)] = Configuration.TFT.poll;
  JsonArray jaTFT = joTFT.createNestedArray(FPSTR(JSCONF_TFT));
  jaTFT.add(Configuration.TFT.data.tft[0]);
  jaTFT.add(Configuration.TFT.data.tft[1]);
  jaTFT.add(Configuration.TFT.data.tft[2]);

  // add WiFi information
  JsonObject joWiFi = joConfig.createNestedObject(FPSTR(JSCONF_WIFI));
  joWiFi[FPSTR(JSCONF_WIFI_SSID)] = Configuration.WiFi.SSID.c_str(); // prevent copying string
  joWiFi[FPSTR(JSCONF_WIFI_KEY)] = Configuration.WiFi.KEY.c_str(); // prevent copying string

  // add PID information
  JsonObject joPID = joConfig.createNestedObject(FPSTR(JSCONF_PID));
  joPID[FPSTR(JSCONF_POLL)] = Configuration.PID.poll;
  joPID[FPSTR(JSCONF_PID_KP)] = Configuration.PID.KP;
  joPID[FPSTR(JSCONF_PID_KI)] = Configuration.PID.KI;
  joPID[FPSTR(JSCONF_PID_KD)] = Configuration.PID.KD;
  joPID[FPSTR(JSCONF_PID_TOLERANCE)] = Configuration.PID.TOL;

  // add programs array if there are any programs in currently loaded configuration
  if(Configuration.nPrograms > 0){
    JsonArray jaPrograms = joConfig.createNestedArray(FPSTR(JSCONF_PROGRAMS));

    // crate object in array for each program
    for(int i = 0; i < Configuration.nPrograms; i++){
      JsonObject joProgram = jaPrograms.createNestedObject();
      TProgram* currProgram = Configuration.Programs[i];

      // set current program name
      joProgram[FPSTR(JSCONF_PROGRAM_NAME)] = currProgram->GetName().c_str(); // prevent copying string

      // add steps array
      JsonArray jaSteps = joProgram.createNestedArray(FPSTR(JSCONF_PROGRAM_STEPS));

      // for each step create nested object and set values
      for(int j = 0; j < currProgram->GetStepsTotal(); j++){
        // create step and set values
        JsonObject joStep = jaSteps.createNestedObject();
        TProgramStep* currStep = currProgram->GetStep(j);

        joStep[FPSTR(JSCONF_PROGRAM_STEP_TSTART)] = currStep->GetTStart();
        joStep[FPSTR(JSCONF_PROGRAM_STEP_TEND)] = currStep->GetTEnd();
        joStep[FPSTR(JSCONF_PROGRAM_STEP_DURATION)] = currStep->GetDuration();
      }
    }
  }

  // end building up configuration
  // it's up to a caller to decide what to do with it
}

/**
 * @brief WebSocket event handler.
 * 
 * @param server server instance
 * @param client client instance
 * @param type event type
 * @param arg event arguments
 * @param data raw data
 * @param len size
 */
void onWSEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  //Handle WebSocket event
  switch(type){
    case WS_EVT_CONNECT:
      // new client connected  -send ping just in case to say we are alive
      client->ping();
      break;
    case WS_EVT_DISCONNECT:
      // client disconnected
    case WS_EVT_PONG:
      // received PONG response to previously sent PING request
    case WS_EVT_ERROR:
      // an error occured
      break; // so far do nothing on the above events
    case WS_EVT_DATA:
    {
      // handle data event - only accept text data (ignore binary data)
      AwsFrameInfo * info = (AwsFrameInfo*)arg;

      // discard any non-text (binary) data
      if(info->opcode != WS_TEXT) break;

      // drop the message if length is 0
      if(info->len == 0) break;

      // reply "Busy" to client if message buffer is currently occupied with other mesage to process
      // i.e. isValid set to true or data length currently in buffer > 0 and is being written by another client
      if(  gi_WsJSONMsg.status == WSJSONMSG_READY ||
          (gi_WsJSONMsg.status == WSJSONMSG_INPROGRESS && gi_WsJSONMsg.client_id != client->id())
        ){
        // send BUSY response
        client->printf(JSON_RESPONSE_TEMPLATE_ERR, "Busy");
        break;
      }

      // at this point received data is allowed to be processed:
      // 1) a complete new message that fits into exactly one incoming buffer with final bit set
      // 2) a complete message that is split into several packets but fits into one complete frame
      // 2) a frame of ongoing message that fits into exactly one incoming buffer
      // 3) a packet of a frame - worst case scenario
      //
      // WebSocket protocol (RFC6455, 5.4) demands that FRAMES are delivered in the order they are sent out
      // however it may seem that packets can be out of order !!!!
      // in any case it should be checked that we can rebuild entire message for JSON parsing within the preallocated buffer
      // of gi_WsJSONMsg set to JSON_BUFFER_MAX_SIZE at compile time
      //
      // OBSERVE:
      // info->num   - is the number of the incoming frame (still can be out of order?) - no way to handle it as there is no way to know lengths of previous frames if out of order
      // info->len   - is the length of the entire frame
      // info->index - data offset within current frame
      // len         - is the length of piece of data (packet) that should reside at a given offset(info->index) of a current frame

      // mark buffer as being in INPROGRESS state
      gi_WsJSONMsg.status = WSJSONMSG_INPROGRESS;
      // set current client id to prevent others from touching the buffer
      gi_WsJSONMsg.client_id = client->id();

      // check that we can fit message into the buffer with respect to already occupied space
      if(gi_WsJSONMsg.datalen + info->len >= JSON_BUFFER_MAX_SIZE){
        // we do not fit - reset the message buffer, discard message
        gi_WsJSONMsg.datalen = 0;
        gi_WsJSONMsg.client_id = 0;
        gi_WsJSONMsg.status = WSJSONMSG_EMPTY;
        // inform client of an error
        client->printf(JSON_RESPONSE_TEMPLATE_ERR, "Message too big");
        break;
      }

      // copy data to message buffer and adjust values for next fragment if expected
      // gi_WsJSONMsg.datalen - is an offset of currently stored data from previous frames
      // info->index - is an offset of current packet data
      // len is the length of current packet buffer
      memcpy( &gi_WsJSONMsg.buffer[gi_WsJSONMsg.datalen + info->index], data, len); // copy data

      // check whether frame and, maybe entire message, is ready
      if( (info->index + len) == info->len ){
        // this was the final packet out of several packets in the frame
        // adjust data length of rebuilt message
        gi_WsJSONMsg.datalen += info->len;

        if(info->final){
          // this was also the final message frame - ready to hand it over to main loop
          gi_WsJSONMsg.buffer[gi_WsJSONMsg.datalen] = 0; // add terminating character for string operations
          gi_WsJSONMsg.status = WSJSONMSG_READY; // from here it will be picked by the main thread
        }
      }
      break;
    } // done handling WS_EVT_DATA
  } // end of switch(type): done handling WebSocket event
} // end of onWSEvent(...)

/**
 * @brief Initialization routine
 * @details Shows spash screen on TFT display and reports current status of initialization.
 *          - Initializes file system.
 *          - Tries to parse oven.json configuration file
 *          - Calibrates touch screen if necessary
 *          - Attempts to connect to WiFi
 *          - Launches web server
 */
void setup() {
  cSplashScreenWindow wSS{gi_Tft};  // splash screen window

  // initialize screen
  gi_Tft.init();
  gi_Tft.setRotation(1);  
  gi_Tft.fillScreen(DT_C_BACKGROUND);

  // prepare and show splash screen
  wSS.pbrProgress.SetProgress(1);
  wSS.lblStatus = F("Starting controller...");
  wSS.Render(false);
  
  pinMode(D1, OUTPUT);    // set relay pin mode to output
  digitalWrite(D1, LOW);  // turn relay off
  
  wSS.pbrProgress.SetProgress(5);

  // 01. Initialize filesystem support
  wSS.lblStatus = F("Init filesystem...");
  wSS.Render(false);

  gi_FSConfig.setAutoFormat(false);
  LittleFS.setConfig(gi_FSConfig);
  if(!LittleFS.begin()){
    LittleFS.format();
    LittleFS.begin();
  }

  wSS.pbrProgress.SetProgress(10);

  // 02. Try to read and parse configuration
  wSS.lblStatus = F("Reading config...");
  wSS.Render(false);

  // check if configuration file exists
  // if it does - read JSON and fill in configuration structure
  if (LittleFS.exists(FPSTR(FILE_CONFIGURATION))) {

    File f = LittleFS.open(FPSTR(FILE_CONFIGURATION), "r");

    if (f) {
      // try deserializing from JSON config file
      
      // try parsing JSON
      //DynamicJsonDocument JDoc{JSON_DOCUMENT_MAX_SIZE};
      //DeserializationError err = deserializeJson(JDoc, f);

      DeserializationError err = deserializeJson(gi_JDoc, f);

      // parese JSON if it was deserialized successfully
      if(!err){
        //JsonObject joConfig = JDoc.as<JsonObject>();
        JsonObject joConfig = gi_JDoc.as<JsonObject>();
        UpdateRunningConfig(joConfig, true);
      }else{
        // an error occured parsing JSON
      }

      // close the file
      f.close();
    }
  }

  wSS.pbrProgress.SetProgress(50);

  // 03. Calibrate touch screen if necessary
  wSS.lblStatus = F("Setting up touchscreen...");
  wSS.Render(false);

  bool calDataOK = ( Configuration.TFT.data.tft[0] != 0 && Configuration.TFT.data.tft[1] != 0 && Configuration.TFT.data.tft[2] != 0);
  if (calDataOK) {
    // calibration data valid
    gi_Tft.setTouch(Configuration.TFT.data.raw);
  } else {
    // data not valid so recalibrate

    // set text color to red
    wSS.lblStatus.SetTextColor(DT_C_RED);
    wSS.lblStatus = F("Touch screen corners as indicated");
    wSS.Render(false);

    gi_Tft.calibrateTouch(Configuration.TFT.data.raw, DT_C_RED, DT_C_BACKGROUND, 20);

    // set text color to green
    wSS.lblStatus.SetTextColor(DT_C_GREEN);
    wSS.lblStatus = F("Calibration complete!");
    wSS.Render(false);
    // reset color to normal
    wSS.lblStatus.SetTextColor(DT_C_LIGHTGREEN);

    // Save configuration to avoid re-calibration on restarts
    File f = LittleFS.open(FPSTR(FILE_CONFIGURATION), "w");

    if (f) {
      //DynamicJsonDocument JDoc{JSON_DOCUMENT_MAX_SIZE};
      //JsonObject joConfig = JDoc.to<JsonObject>();
      JsonObject joConfig = gi_JDoc.to<JsonObject>();
      BuildRunningConfig(joConfig);

      // save config to file
      //serializeJson(JDoc, f);
      serializeJson(gi_JDoc, f);

      // close the file
      f.close();
    }
  }

  wSS.pbrProgress.SetProgress(70);

  // 0.4 Attempt to connect to WiFi (if can not - try to launch self in AP mode ?)
  (wSS.lblStatus = F("Connecting to WiFi ")) += Configuration.WiFi.SSID;
  wSS.Render(false);

  // if WiFi is not configured or not reachable - shall controller launch in AP mode ?
  WiFi.mode(WIFI_STA);
  WiFi.begin(Configuration.WiFi.SSID, Configuration.WiFi.KEY);
  
  for(int i=0; i < 20; i++){
    if (WiFi.status() == WL_CONNECTED) break;

    wSS.pbrProgress.SetProgress( 70 + i);
    wSS.Render(false);
    delay(500);
  }
  
  wSS.pbrProgress.SetProgress(90);

  // 0.5 Start web server
  wSS.lblStatus = F("Starting Web server");
  wSS.Render(false);

  // attach event to web socket instance
  gi_WebSocket.onEvent(onWSEvent);
  gi_WebServer.addHandler(&gi_WebSocket);

  // add hook to /heap path - show free heap for monitoring purposes
  gi_WebServer.on(PATH_WEB_HEAP, HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, FPSTR(FILE_WEB_CT_TXT), F("Free heap: ") + String(ESP.getFreeHeap())); });
  
  // serve files from filesystem with default being index.html
  gi_WebServer.serveStatic(PATH_WEB_ROOT, LittleFS, PATH_WEB_ROOT).setDefaultFile(FILE_WEB_INDEX);
  
  // add response hook on invalid paths HTTP: 404
  gi_WebServer.onNotFound([](AsyncWebServerRequest *request) { request->send(404, FPSTR(FILE_WEB_CT_TXT), F("Nothing found :(")); });

  // finally start the server
  gi_WebServer.begin();

  //wSS.lblStatus.SetText(F("Done"));
  wSS.lblStatus = F("Done");
  wSS.pbrProgress.SetProgress(100);
  wSS.Render(false);
  delay(1000);

  // 0.6 no more initialization screen updates - invalidate and render main window
  wnd.Invalidate();
  wnd.Render(false);
} // end of setup()

#define BUFF_LEN 32
#define STS_ROTATE_EVERY 10
static int stsrtr = 0;

/**
 * @brief Main controller loop function
 * @details Main program logic:
 *          - Polls touch screen sensor and passes touch event to main window for handling
 *          - Polls temperature sensor and updates probe and ambient temperatures
 *          - If a program is currently active: calculates set point and controlling signal value
 *          - Sends out current state updates to connected WebSocket clients
 */
void loop() {

  unsigned long ticks = millis();
  uint16_t x,y;
  char buff[BUFF_LEN];

  // allow handling touch screen events early to allow emergency oven stop as early as possible
  if ( (ticks - State.ticks_TS >= Configuration.TFT.poll) && gi_Tft.getTouch(&x, &y) )
  {
    wnd.HandleEvent(x,y, true);
    State.ticks_TS = ticks; // save current timer value for next loop
  }

  // after handling events - check program start/stop flag
  // start/stop can come from TFT as well as from async WebSocket message on previous loop
  // in any case we need to adopt both screen and running state before potentially
  // running PID controller
  // this part is run on every loop to avoid delays
  if(State.StartStop){
    // start signalled ...
    if(!State.isPgmRunning){
      // ... but program is not running: check if we can start or reset flag otherwise
      if(State.ActiveProgram != nullptr){
        // active program exists - prepare and start it
        gp_PID.Reset( Configuration.PID.KP, Configuration.PID.KI, Configuration.PID.KD, Configuration.PID.poll);
        State.tSP = State.ActiveProgram->Begin();
        State.isPgmRunning = true;

        // update GUI
        wnd.lblTemprTarget.Visible(true);
        wnd.btnStart = FPSTR(BTN_STOP);
        wnd.btnStart.SetBtnColor(DT_C_RED);
      }else{
        // no active program - reset flag to avoid confusion
        State.StartStop = false;
      }
    } // else program is already running and there is no need to do anything
  }else{
    // stop signalled ...
    if(State.isPgmRunning){
      // ... but program is running: stop it
      digitalWrite(D1, LOW); // turn relay switch off
      State.isRelayOn = false;

      // reset active program but leave it selected - active program can't be nullptr - we should not have started
      State.ActiveProgram->Reset();

      State.U = 0.0; // reset controlling signal
      State.isPgmRunning = false;

      // update TFT screen
      wnd.lblTemprTarget.Visible(false);
      wnd.lblStatus = F("Program has ended");
      wnd.btnStart = FPSTR(BTN_START);
      wnd.btnStart.SetBtnColor(DT_C_GREEN);
    } // else no program is running and there is no need to do anything
  }

  // with respect to PID poll interval:
  // - measure temperature
  // - apply control if needed (i.e. Program is running)
  // - update TFT screen elements
  // - send status updates to connected WebSocket clients
  //
  // NB! once PIDControl program has started - polling interval is taken from current instance of PID regardless of any changes
  //     to configuration from Web interface.
  //     If the program is not running (has stopped) - value from running config is used
  //
  unsigned long _PIDPOLL = State.isPgmRunning ? gp_PID.poll : Configuration.PID.poll;
  if ( ticks - State.ticks_TSENSOR >= _PIDPOLL )
  {
    // read temperature and update values on screen
    uint8_t faultCode = gi_TS.readChip();           // read chip updated value and save error for easy access
    State.tAmbient    = gi_TS.getAmbient();         // get updated value of chip ambient temperature
    State.tProbe      = gi_TS.getProbeLinearized(); // get probe temperature as read by chip

    if (faultCode)                                  // Display error code if present
    {
      if (faultCode & 0b001) {
        wnd.lblStatus = F("ERR: Sensor not connected");
      }
      if (faultCode & 0b010) {
        wnd.lblStatus = F("ERR: Sensor s-c to GND(-)");
      }
      if (faultCode & 0b100) {
        wnd.lblStatus = F("ERR: Sensor s-c to VCC(+)");
      }
      wnd.lblTempr = FPSTR(LBL_EMPTY);
    }
    else
    {
      (wnd.lblTempr = String(State.tProbe, 1)) += FPSTR(LBL_DEGC);
    }

    // check if program is in Running state
    if(State.isPgmRunning){

      // Begin() on program should have been called already but still recalculate the Set Point value
      State.tSP = State.ActiveProgram->CalculateSetPoint();

      // make a check for program termination
      if(isnan(State.tSP) || faultCode){
        // meaning an error or reached the end of the program
        digitalWrite(D1, LOW); // turn relay switch off
        State.isRelayOn = false;

        State.StartStop = false;
        wnd.lblStatus = F("Program terminating...");
      }else{
        // do common staff - calculate controlling signal and turn relay on/off accordingly
        State.U = gp_PID.Evaluate(State.tSP, State.tProbe, State.U);
        
        // check whether relay needs to be turned on or off
        if( Configuration.PID.TOL - abs(State.U) < 0.0 ){
          State.isRelayOn = (State.U > 0.0); // beyond tolerance - adjust relay state
        }else{
          State.isRelayOn = false; // within tolerance - turn off relay
        }
        //State.isRelayOn = !State.isRelayOn;
        digitalWrite(D1, (State.isRelayOn ? HIGH : LOW));

        // update labels' values
        (wnd.lblTemprTarget = String(State.tSP,2)) += FPSTR(LBL_DEGC);
        wnd.lblStepNumber = String(State.ActiveProgram->GetStepsCurrent());

        // update program remaining time
        unsigned long t = State.ActiveProgram->GetDurationTotal() - State.ActiveProgram->GetDurationElapsed();
        snprintf(buff, BUFF_LEN, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
        wnd.lblProgramTimeValue = buff;

        // update step remaining time
        t = State.ActiveProgram->GetDurationElapsedStep();
        snprintf(buff, BUFF_LEN, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
        wnd.lblStepTimeValue = buff;

        (wnd.lblStatus = F("Control U = ")) += String(State.U,6);
      }

    } // end if(isPgmRunning)

    // temp? - show ip address in the status bar every 10 seconds?
    //
    if(stsrtr == STS_ROTATE_EVERY) {
      stsrtr = 0;
      (wnd.lblStatus = F("IP address: ")) += WiFi.localIP().toString();
    }else{
      stsrtr++;
    }
    
    // Send update to all possible connected clients sending them the state
    if(gi_WebSocket.count()) {
      //DynamicJsonDocument jDoc(1024); // adjust capacity to something more appropriate for status JSON document
      gi_JDoc.clear();
      
      // add message id
      //jDoc["id"] = "STS";
      gi_JDoc["id"] = "STS";
      
      // add status and populate fields
      //JsonObject joStatus = jDoc.createNestedObject("status");
      JsonObject joStatus = gi_JDoc.createNestedObject("status");
      joStatus["tPB"] = State.tProbe;
      joStatus["tAM"] = State.tAmbient;
      joStatus["tSP"] = State.tSP;
      joStatus["U"] = State.U;
      joStatus["isRunning"] = State.isPgmRunning;
      joStatus["isRelayOn"] = State.isRelayOn;
      joStatus["stsText"] = wnd.lblStatus.c_str(); // should be safe as serialize is called shortly

      // relevant information about active program
      if(State.ActiveProgram != nullptr){
        joStatus["actStep"] = State.ActiveProgram->GetStepsCurrent();
        joStatus["tmElapsed"] = State.ActiveProgram->GetDurationElapsed();

        // add active program
        JsonObject joProgram = joStatus.createNestedObject("actPgm");
        joProgram[FPSTR(JSCONF_PROGRAM_NAME)] = State.ActiveProgram->GetName();

        // add steps array and loop through all steps adding their parameters
        JsonArray jaSteps = joProgram.createNestedArray(FPSTR(JSCONF_PROGRAM_STEPS));
        for(int i = 0; i < State.ActiveProgram->GetStepsTotal(); i++){
          JsonObject joStep = jaSteps.createNestedObject();
          joStep[FPSTR(JSCONF_PROGRAM_STEP_TSTART)] = State.ActiveProgram->GetStep(i)->GetTStart(); // start temperature
          joStep[FPSTR(JSCONF_PROGRAM_STEP_TEND)] = State.ActiveProgram->GetStep(i)->GetTEnd(); // end temperature
          joStep[FPSTR(JSCONF_PROGRAM_STEP_DURATION)] = State.ActiveProgram->GetStep(i)->GetDuration(); // duration in ms
        }
      }else{
        // there is no active program
        joStatus["actPgm"] = nullptr;
        joStatus["actStep"] = 0;
        joStatus["tmElapsed"] = 0;
      }

      // send resulting JSON to clients
      //size_t jdLen = measureJson(jDoc); // length of the resulting condenced JSON excluding 0-terminator
      size_t jdLen = measureJson(gi_JDoc);
      AsyncWebSocketMessageBuffer * buffer = gi_WebSocket.makeBuffer(jdLen); // allocates buffer that includes 0-terminator
      if(buffer){
        //serializeJson(jDoc, buffer->get(), jdLen + 1); // note accounting for 0-terminator in the length of the buffer provided to function
        serializeJson(gi_JDoc, buffer->get(), jdLen + 1);
        gi_WebSocket.textAll(buffer);
      }
    }

    // finally save timer value
    State.ticks_TSENSOR = ticks;
  }

  // WebSocket message processing
  // should happen without any timer checks so that we are making responses available for async handler asap
  // should be last in a chain of actions in the main loop
  if(gi_WsJSONMsg.status == WSJSONMSG_READY){
    // read message in case data available is > 0
    if( gi_WsJSONMsg.datalen > 0 ){
      // create an object - same as data length
      //DynamicJsonDocument jDocInc{JSON_DOCUMENT_MAX_SIZE};
      //DeserializationError err = deserializeJson(jDocInc,gi_WsJSONMsg.buffer);
      DeserializationError err = deserializeJson(gi_JDoc,gi_WsJSONMsg.buffer);

      if(!err){

        //const char* cmd = jDocInc["id"];
        const char* cmd = gi_JDoc["id"];

        // parse JSON command and prepare reply
        if(!strcmp("start",cmd)){
          // "start" command received - toggle start and send reply
          State.StartStop = true;
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Start signal sent");

        }else if(!strcmp("stop",cmd)){
          // "stop" command received - toggle stop and send reply
          State.StartStop = false;
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Stop signal sent");

        }else if(!strcmp("cfgWR",cmd)){
          // configuration submitted from web interface - try building and saving configuration

          // jDocInc should contain msg object with configuration set
          //JsonObject joConfig = jDocInc["msg"];
          JsonObject joConfig = gi_JDoc["msg"];
          if(joConfig){
            // msg object present
            UpdateRunningConfig(joConfig, false);

            // notify client that configuration was modified
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Configuration modified");

          }else{
            // malformed request
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Malformed JSON");
          }

          // done parsing incoming configuration

        }else if(!strcmp("cfgRD",cmd)){
          // configuration requested - build JSON and send back to client

          // create outgoing JSON object
          //DynamicJsonDocument jDocOut{8192}; // ??? appropriate size - especially for configuration file
          gi_JDoc.clear();

          //jDocOut["id"] = "OK";
          gi_JDoc["id"] = "OK";

          // add nested configuration object
          //JsonObject joConfig = jDocOut.createNestedObject("config");
          JsonObject joConfig = gi_JDoc.createNestedObject("config");

          // build running configuration
          BuildRunningConfig(joConfig);

          // send out reply
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          // make sure client is still valid and can be communicated to
          if( wsc != nullptr ){
            //size_t jdLenOut = measureJson(jDocOut);
            size_t jdLenOut = measureJson(gi_JDoc);
            AsyncWebSocketMessageBuffer * buffer = gi_WebSocket.makeBuffer(jdLenOut);
            if(buffer){
              //serializeJson(jDocOut, buffer->get(), jdLenOut + 1);
              serializeJson(gi_JDoc, buffer->get(), jdLenOut + 1);
              wsc->text(buffer);
            }
          }

          // done sending active configuration. jDocOut should be destructed here

        }else if(!strcmp("cfgSV",cmd)){
          // write configuration to flash memory
          // Save configuration to avoid re-calibration on restarts
          File f = LittleFS.open(FPSTR(FILE_CONFIGURATION), "w");

          if (f) {
            //DynamicJsonDocument JDoc{JSON_DOCUMENT_MAX_SIZE};
            //JsonObject joConfig = JDoc.to<JsonObject>();
            JsonObject joConfig = gi_JDoc.to<JsonObject>();
            BuildRunningConfig(joConfig);

            // save config to file
            //serializeJson(JDoc, f);
            serializeJson(gi_JDoc, f);

            // close the file
            f.close();

            // notify client that configuration was saved to flash
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Configuration saved to flash");
          }else{
            // notify client that configuration was modified
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Flash write failed");
          }
        }else if(!strcmp("setPG",cmd)){
          // set program to be run
          // make sure controller is not running program and no attempt to start program was made
          if(!State.isPgmRunning && !State.StartStop){
            // msg element should contain program name - try to find it
            const char* pgmName = gi_JDoc["msg"];
            // ----------------
            if(pgmName != nullptr){
              int i = 0;
              for(i = 0; i < Configuration.nPrograms; i++){
                // compare program name to one supplied in message
                if(!strcmp(pgmName, Configuration.Programs[i]->GetName().c_str())){
                  // found a program
                  // delete active program if it was set
                  if(State.ActiveProgram != nullptr) delete State.ActiveProgram;

                  // create a copy of selected program
                  State.ActiveProgram = new TProgram(*Configuration.Programs[i]); // invoke copy constructor

                  // set selected program details
                  wnd.lblProgramName = State.ActiveProgram->GetName();
                  wnd.lblStepTotal = String(State.ActiveProgram->GetStepsTotal());
                  wnd.lblStepNumber = String(State.ActiveProgram->GetStepsCurrent());

                  // show initial duration of selected program
                  unsigned long t = State.ActiveProgram->GetDurationTotal();
                  snprintf(buff, BUFF_LEN, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
                  wnd.lblProgramTimeValue = buff;
                  // end of adjusting controls
                  AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
                  if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Program selected");

                  break; // break for() loop
                }
                if(i == Configuration.nPrograms){
                  AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
                  if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Program not found");
                }
              }
            }else{
              AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
              if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Empty program");
            }
            // -----------------
          }
        }else{
          // unknown command
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Unknown command");
        }

      }else{
        // unable to deserialize JSON
        AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
        if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Malformed JSON");
      } // end if(!err)

      // jDocInc goes out of scope - should be destructed here
      // end processing incoming message

    } // end if(datalen > 0)

    // so far drop incoming message silently if it was malformed

    gi_WsJSONMsg.datalen = 0;
    gi_WsJSONMsg.client_id = 0;
    gi_WsJSONMsg.status = WSJSONMSG_EMPTY; // mark as available for accomodating new message after all cleanup activities that may take time
  } // done handling incoming message

  // and finally redraw screen if needed
  wnd.Render(false);

} // end of loop()
