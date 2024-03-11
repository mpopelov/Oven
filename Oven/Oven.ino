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

#define PIN_RELAY   36 // pin used for relay control
#define PIN_TSENSOR 21 // pin used for temperature sensor control

#include <Arduino.h>
#include <math.h>
#include <TFT_eSPI.h>             // included for TFT support
#include <LittleFS.h>             // included for file system support

#include <WiFi.h>                 // included for WiFi support
#include <AsyncTCP.h>             // included for async TCP communication
#include <ESPAsyncWebServer.h>    // included for HTTP and WebSocket support

#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>          // included for JSON support

#include <DeskTop.h>              // included for simple desktop class library
#include <TSensor.h>              // included for K-type temperature sensor support
#include <PIDControl.h>           // included for PID-control implementation
#include <TProgram.h>             // included for base classes describing controller programs

#include "JSconf.h"             // controller configuration related contents

/**
 * Fonts to be used for rendering on TFT display
 */
#define FSN1 FreeSans9pt7b      // for status text and small labels
#define FSB1 FreeSansBold9pt7b  // bold font for small text
#define FSN2 FreeSans12pt7b     // for medium sized text
#define FSB2 FreeSansBold12pt7b // for medium sized text in bold
#define FSB3 FreeSansBold18pt7b
#define FSB4 FreeSansBold24pt7b


/**
 * Global string constants
 */
// file names and paths (also for web server)
const char FILE_WEB_INDEX[]      = "index.html";
const char PATH_WEB_ROOT[]       = "/";
const char PATH_WEB_HEAP[]       = "/heap";
const char PATH_WEB_WS[]         = "/ws";
const char FILE_WEB_CT_TXT[]     = "text/plain";

// button text strings
const char BTN_START[]           = "Start";
const char BTN_STOP[]            = "Stop";
const char BTN_PROG[]            = "Prog";
// label text strings
const char LBL_EMPTY[]           = "---";
const char LBL_TEMPREMPTY[]      = "----C";
const char LBL_TIMEREMPTY[]      = "00:00:00";
const char LBL_PROG[]            = "Program: ";
const char LBL_STEP[]            = "Step: ";
const char LBL_OF[]              = " of ";
const char LBL_STEPTARGET[]      = "Step target: ";
const char LBL_STEPTIME[]        = "Step remaining: ";
const char LBL_TIMEREMAINING[]   = "Program remaining: ";
const char LBL_DEGC[]            = " C";

// JSON message / configuration elements and defines
#define JSON_BUFFER_MAX_SIZE 8192   // maximum message/buffer size
#define JSON_DOCUMENT_MAX_SIZE 8192 // maximum size of JSON document allowed

const char JSON_RESPONSE_TEMPLATE_OK[]             = "{\"id\":\"OK\",\"details\":\"%s\"}";
const char JSON_RESPONSE_TEMPLATE_ERR[]            = "{\"id\":\"ERR\",\"details\":\"%s\"}";




/**
 * @brief Global controller state
 * 
 */
struct _sState {

  double tProbe = NAN;                        // current temperature as reported by probe
  double tAmbient = NAN;                      // current ambient temperature
  double tSP = NAN;                           // current set point temperature
  double U = 0.0;                             // current controlling signal value

  TProgram ActiveProgram;                     // current active program

  unsigned long ticks_TS = 0;                 // timestamp of last touchscreen polling
  unsigned long ticks_TSENSOR = 0;            // timestamp of last temperature sensor polling
  unsigned long ticks_PGM = 0;                // time elapsed since the start of current program step
  unsigned long ticks_PGM_Total = 0;          // time elapsed since current program start

  volatile bool StartStop = false;            // a flag to signal main loop to start (true) or stop (false) selected program.
  bool isPgmRunning = false;                  // gloabl flag for signalling controller program state
  bool isRelayOn = false;                     // flag to indicate that the relay is turned on
  uint32_t err_count = 0;

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

TSensor           gi_TS{PIN_TSENSOR};                 // MAX31855 K-type temperature sensor instance
// Enable only one PID control instance
PIDControlBasic   gp_PID{};
//PIDControlSimple  gp_PID{};
//PIDControlIIR     gp_PID{};

AsyncWebServer    gi_WebServer{80};                 // a web server instance
AsyncWebSocket    gi_WebSocket{PATH_WEB_WS}; // web socket for communicating with OvenWEB

JSConf Configuration;

JsonDocument gi_JDoc;



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
      btnUp( gfx, 270,   0,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "UP", DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Up>(this)),
      btnOk( gfx, 270,  51,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, "OK", DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Ok>(this)),
  btnCancel( gfx, 270, 102,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_RED, DT_C_BACKGROUND, &FSB2, "X", DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Cancel>(this)),
    btnDown( gfx, 270, 153,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "DN", DTDelegate::create<cPgmSelectWindow,&cPgmSelectWindow::OnButton_Down>(this)),
   selProgs( gfx,   0,   0, 269, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_LIGHTGREEN, DT_C_BACKGROUND, DT_C_BACKGROUND, DT_C_LIGHTGREEN, &FSN1),
  _Result(false), _callback(callback)
  {
    AddControl(&btnUp);
    AddControl(&btnOk);
    AddControl(&btnCancel);
    AddControl(&btnDown);
    AddControl(&selProgs);
    if(Configuration.nPrograms > 0)
      for(int idx=0; idx < Configuration.nPrograms; idx++) selProgs.AddItem(idx, Configuration.Programs[idx].GetName());
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
              btnProg( gfx, 250,   0,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, BTN_PROG, DTDelegate::create<cMainWindow,&cMainWindow::OnButton_PGM>(this)),
             btnStart( gfx, 250,  51,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, BTN_START, DTDelegate::create<cMainWindow,&cMainWindow::OnButton_STRTSTP>(this)),
  // program details - Y offset 0
           lblProgram( gfx,   0,   0,  80,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_PROG),
       lblProgramName( gfx,  80,   0, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
  // step details + forward and backward butons - Y offset 25
              btnSBck( gfx,   0,  25,  50,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSN1, "<", DTDelegate::create<cMainWindow,&cMainWindow::OnButton_SBck>(this)),
              lblStep( gfx,  50,  25,  50,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_STEP),
        lblStepNumber( gfx, 100,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
            lblStepOf( gfx, 130,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_OF),
         lblStepTotal( gfx, 160,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
              btnSFwd( gfx, 190,  25,  50,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSN1, ">", DTDelegate::create<cMainWindow,&cMainWindow::OnButton_SFwd>(this)),
  // temperature main display - Y offset 50
             lblTempr( gfx,   0,  50, 240,  85, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSB4, LBL_TEMPREMPTY),
       lblTemprTarget( gfx, 130, 135,  90,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TEMPREMPTY),
          lblTemprAmb( gfx,   0, 135,  90,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TEMPREMPTY),
  // step timing values - Y offset 160
          lblStepTime( gfx,   0, 160, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_STEPTIME),
     lblStepTimeValue( gfx, 170, 160,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMPTY),
  // displaying program timer - Y offset 185
       lblProgramTime( gfx,   0, 185, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMAINING),
  lblProgramTimeValue( gfx, 170, 185,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMPTY),
  // status label - Y offset 210
            lblStatus( gfx,   0, 210, 320,  30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, "Initializing...")
  {
    // Pre-allocate some space for labels that will have changing content
    // and their content is bigger than SSO buffer of String class.
    // Static text labels are ok as they are going to remain as is and most often fit into SSO buffer.

    lblProgramName.reserve(32);
    lblStepNumber.reserve(12);
    lblStatus.reserve(64);

    // set alignment on lebels as necessary
    lblTempr.SetTextAlignment(CENTER);
    lblTemprTarget.SetTextAlignment(RIGHT);
    lblTemprAmb.SetTextAlignment(LEFT);

    // Add controls to handling stack
    AddControl(&lblProgram);
    AddControl(&lblProgramName);
    AddControl(&btnSBck);
    AddControl(&lblStep);
    AddControl(&lblStepNumber);
    AddControl(&lblStepOf);
    AddControl(&lblStepTotal);
    AddControl(&btnSFwd);
    AddControl(&lblTemprTarget);
    AddControl(&lblTemprAmb);
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
          // create a copy of selected program
          State.ActiveProgram = Configuration.Programs[_pgmIdx];

          // set selected program details
          lblProgramName = State.ActiveProgram.GetName();
          lblStepTotal = String(State.ActiveProgram.GetStepsTotal());

          UpdateProgramDisplay();
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
      if(State.ActiveProgram.IsValid()){
        // only start things if there is an active program selected
        // raise the flag and signal main loop to start executing
        State.StartStop = true;
      }
    }
  }

  // callback for step forward button
  void OnButton_SFwd()
  {
    // change step only in case there is active program and controller is not running
    if(!State.isPgmRunning && State.ActiveProgram.IsValid()){
      // change step
      State.ActiveProgram.StepForward();
      // update displayed values
      UpdateProgramDisplay();
    }
  }

  // callback for step backward button
  void OnButton_SBck()
  {
    // change step only in case there is active program and controller is not running
    if(!State.isPgmRunning && State.ActiveProgram.IsValid()){
      // change step
      State.ActiveProgram.StepBack();
      // update displayed values
      UpdateProgramDisplay();
    }
  }

  // update display values for program/step reflecting remaining time before/during program execution
  void UpdateProgramDisplay()
  {
    if(State.ActiveProgram.IsValid()){

      // update current step - remember we get back 0-based index.
      // for user we present adjusted value starting from 1 up to the maximum number of steps in the program
      lblStepNumber = String(State.ActiveProgram.GetStepsCurrent() + 1);

      // adjust remaining duration of selected program
      char buff[12];
      unsigned long t = State.ActiveProgram.GetDurationProgram() - State.ActiveProgram.GetDurationProgramElapsed();
      snprintf(buff, 12, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
      lblProgramTimeValue = buff;

      // show remaining duration of current step
      t = State.ActiveProgram.GetDurationStep() - State.ActiveProgram.GetDurationStepElapsed();
      snprintf(buff, 12, "%02lu:%02lu:%02lu", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
      lblStepTimeValue = buff;

      // update target temperature label:
      // if program is running - update with setpoint value
      // if program is not running - update with target temperature of current step
      if(State.isPgmRunning){
        (lblTemprTarget = String(State.tSP,2)) += LBL_DEGC;
      }else{
        lblTemprTarget.SetTextColor(DT_C_LIGHTGREEN);
        (lblTemprTarget = String(State.ActiveProgram.GetTemperatureStepEnd(),2)) += LBL_DEGC;
      }

    }else{
      // no valid program selected - update with initial "empty" values
      lblStepNumber = LBL_EMPTY;
      lblStepTotal = LBL_EMPTY;
      lblProgramTimeValue = LBL_TIMEREMPTY;
      lblStepTimeValue = LBL_TIMEREMPTY;
      lblTemprTarget = LBL_TEMPREMPTY;
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
  DTButton btnProg;   // Program select button
  DTButton btnStart;  // Start/Stop program button
  DTButton btnSFwd;   // step forward button
  DTButton btnSBck;   // step backward button
  
  // labels
  DTLabel lblTempr;             // current measured temperature
  DTLabel lblTemprTarget;       // current target temperature (SetPoint)
  DTLabel lblTemprAmb;          // current ambient temperature
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

hw_timer_t * timerTouch = NULL;
volatile SemaphoreHandle_t semaphoreTimerTouch;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void ARDUINO_ISR_ATTR onTimerTouch(){
  // signal main loop thread that it's time to check touch screen / redraw GUI
  xSemaphoreGiveFromISR(semaphoreTimerTouch, NULL);
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
  wSS.lblStatus.SetTextAlignment(CENTER);
  wSS.lblStatus = "Starting controller...";
  wSS.Render(false);
  
  pinMode(PIN_RELAY, OUTPUT);    // set relay pin mode to output
  digitalWrite(PIN_RELAY, LOW);  // turn relay off
  
  wSS.pbrProgress.SetProgress(5);

  // 01. Initialize filesystem support
  wSS.lblStatus = "Init filesystem...";
  wSS.Render(false);

  #define FORMAT_LITTLEFS_IF_FAILED true
  LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);

  wSS.pbrProgress.SetProgress(10);

  // 02. Try to read and parse configuration
  wSS.lblStatus = "Reading config...";
  wSS.Render(false);

  // check if configuration file exists
  // if it does - read JSON and fill in configuration structure
  if (LittleFS.exists(JSConf::FILE_CONFIGURATION)) {

    File f = LittleFS.open(JSConf::FILE_CONFIGURATION, "r");

    if (f) {
      // try deserializing from JSON config file
      DeserializationError err = deserializeJson(gi_JDoc, f);

      // parese JSON if it was deserialized successfully
      if(!err){
        JsonObject joConfig = gi_JDoc.as<JsonObject>();
        Configuration.UpdateRunningConfig(joConfig, true);
      }else{
        // an error occured parsing JSON
      }

      // close the file
      f.close();
    }
  }

  wSS.pbrProgress.SetProgress(30);

  // 03. Try to read and parse preograms
  wSS.lblStatus = "Reading programs...";
  wSS.Render(false);

  // check if programs file exists
  // if it does - read JSON and fill in programs array
  if (LittleFS.exists(JSConf::FILE_PROGRAMS)) {

    File f = LittleFS.open(JSConf::FILE_PROGRAMS, "r");

    if (f) {
      // try deserializing from JSON programs file
      DeserializationError err = deserializeJson(gi_JDoc, f);

      // parese JSON if it was deserialized successfully
      if(!err){
        JsonArray jaPrograms = gi_JDoc.as<JsonArray>();
        Configuration.UpdateRunningPrograms(jaPrograms, true);
      }else{
        // an error occured parsing JSON
      }

      // close the file
      f.close();
    }
  }

  wSS.pbrProgress.SetProgress(50);

  // 04. Calibrate touch screen if necessary
  wSS.lblStatus = "Setting up touchscreen...";
  wSS.Render(false);

  bool calDataOK = ( Configuration.TFT.data.tft[0] != 0 && Configuration.TFT.data.tft[1] != 0 && Configuration.TFT.data.tft[2] != 0);
  if (calDataOK) {
    // calibration data valid
    gi_Tft.setTouch(Configuration.TFT.data.raw);
  } else {
    // data not valid so recalibrate

    // set text color to red
    wSS.lblStatus.SetTextColor(DT_C_RED);
    wSS.lblStatus = "Touch screen corners as indicated";
    wSS.Render(false);

    gi_Tft.calibrateTouch(Configuration.TFT.data.raw, DT_C_RED, DT_C_BACKGROUND, 20);

    // set text color to green
    wSS.lblStatus.SetTextColor(DT_C_GREEN);
    wSS.lblStatus = "Calibration complete!";
    wSS.Render(false);
    // reset color to normal
    wSS.lblStatus.SetTextColor(DT_C_LIGHTGREEN);

    // Save configuration to avoid re-calibration on restarts
    File f = LittleFS.open(JSConf::FILE_CONFIGURATION, "w");

    if (f) {
      JsonObject joConfig = gi_JDoc.to<JsonObject>();
      Configuration.BuildRunningConfig(joConfig);

      // save config to file
      serializeJson(gi_JDoc, f);

      // close the file
      f.close();
    }
  }

  wSS.Invalidate(); // invalidate entire window to get rid of calibrating artifacts
  wSS.pbrProgress.SetProgress(70);

  // 0.4 Attempt to connect to WiFi (if can not - try to launch self in AP mode ?)
  (wSS.lblStatus = "Connecting to WiFi ") += Configuration.WiFi.SSID;
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
  wSS.lblStatus = "Starting Web server";
  wSS.Render(false);

  // attach event to web socket instance
  gi_WebSocket.onEvent(onWSEvent);
  gi_WebServer.addHandler(&gi_WebSocket);

  // add hook to /heap path - show free heap for monitoring purposes
  gi_WebServer.on(PATH_WEB_HEAP, HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, FILE_WEB_CT_TXT, String("Free heap: ") + String(ESP.getFreeHeap())); });
  
  // serve files from filesystem with default being index.html
  gi_WebServer.serveStatic(PATH_WEB_ROOT, LittleFS, PATH_WEB_ROOT).setDefaultFile(FILE_WEB_INDEX);
  
  // add response hook on invalid paths HTTP: 404
  gi_WebServer.onNotFound([](AsyncWebServerRequest *request) { request->send(404, FILE_WEB_CT_TXT, "Nothing found :("); });

  // finally start the server
  gi_WebServer.begin();

  //wSS.lblStatus.SetText(F("Done"));
  wSS.lblStatus = "Done";
  wSS.pbrProgress.SetProgress(100);
  wSS.Render(false);
  delay(1000);

  // 0.6 no more initialization screen updates - invalidate and render main window
  wnd.Invalidate();
  wnd.Render(false);

  // touchscreen polling on intervals.
  // set up touchscreen handler: timer and semaphore
  semaphoreTimerTouch = xSemaphoreCreateBinary();
  timerTouch = timerBegin(0, 80, true);
  timerAttachInterrupt(timerTouch, &onTimerTouch, true);
  timerAlarmWrite(timerTouch, Configuration.TFT.poll*500, true);
  timerAlarmEnable(timerTouch);

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
  /* if ( (ticks - State.ticks_TS >= Configuration.TFT.poll) && gi_Tft.getTouch(&x, &y) )
  {
    wnd.HandleEvent(x,y, true);
    State.ticks_TS = ticks; // save current timer value for next loop
  } */

  //new implementation with timer interrupt
  // 1. check semaphore to see if it was fired from interrupt
  // 2. if yes - check the touch sensor state and process message in case there is an active touch
  if( (xSemaphoreTake(semaphoreTimerTouch,0) == pdTRUE) && gi_Tft.getTouch(&x,&y) ){
    wnd.HandleEvent(x,y,true);
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
      if(State.ActiveProgram.IsValid()){
        // active program exists - prepare and start it
        gp_PID.Reset( Configuration.PID.KP, Configuration.PID.KI, Configuration.PID.KD, Configuration.PID.poll);
        State.tSP = State.ActiveProgram.Begin();
        State.isPgmRunning = true;

        // update GUI
        wnd.btnStart = BTN_STOP;
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
      digitalWrite(PIN_RELAY, LOW); // turn relay switch off
      State.isRelayOn = false;

      // reset active program but leave it selected - active program can't be nullptr - we should not have started
      State.ActiveProgram.Reset();

      State.U = 0.0; // reset controlling signal
      State.isPgmRunning = false;

      // update TFT screen
      wnd.lblStatus = "Program has ended";
      wnd.btnStart = BTN_START;
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
    //
    // TEMP: there seem to be some errors generated by probe at higher temperatures
    // for a reason unknown so far.
    // To avoid breaking program upon random unsuccessful attempts to read temperature sensor the following
    // patch is introduced:
    // Make at most 3 attempts to read temperature values.
    // If any attempt is successful - proceed normally and log the fact error occured.
    // If all 3 readings result in an error - stop the program as intended initially.

    uint8_t faultCode = 0;  // reset fault code to 0
    for(int tAttempt = 0; tAttempt < 3; tAttempt++){
      // try reading and updateing temperature values
      faultCode = gi_TS.readChip();
      if(faultCode == 0) break; // break the for loop in case of good reading
      delay(5); // TEMP!!!! add a delay between readings assuming error conditions can change and a good reading will follow
    }

    // update values
    // save previous probe value for error handling situation

    double tProbe_previous = State.tProbe;

    State.tAmbient    = gi_TS.getAmbient();         // get updated value of chip ambient temperature
    State.tProbe      = gi_TS.getProbeLinearized(); // get probe temperature as read by chip

    // original implementation
    /* if(faultCode){
      // an error occured
      if (faultCode & 0b001) { wnd.lblStatus = "ERR: Sensor not connected"; }
      if (faultCode & 0b010) { wnd.lblStatus = "ERR: Sensor s-c to GND(-)"; }
      if (faultCode & 0b100) { wnd.lblStatus = "ERR: Sensor s-c to VCC(+)"; }
      wnd.lblTempr = LBL_TEMPREMPTY;
    }else{
      (wnd.lblTempr = String(State.tProbe, 1)) += LBL_DEGC;
    } */

    ///------------------------------- START OF ALTERNATIVE IMPLEMENTATION ---------------------------
    // alternative implementation - try to ignore Short-To-Ground fault at high temperatures
    if(faultCode){
      // an error occured
      State.err_count++; // increase the error counter
      if (faultCode & 0b001) { (wnd.lblStatus = String(State.err_count)) += " ERR: OC"; }  // open cirquit
      if (faultCode & 0b010) { (wnd.lblStatus = String(State.err_count)) += " ERR: SCG"; } // short to GND
      if (faultCode & 0b100) { (wnd.lblStatus = String(State.err_count)) += " ERR: SCV"; } // short to Vcc
      //!!!!!!! prevent switching off by faultcode - think of adding limit on how many errors we can tolerate
      faultCode = 0;
      // now compare the values of previous and last temperature values to see if there is a spike difference
      // make an assumption: in furnace temperature can't change dramatically - the system is too inertial.
      // discard the new value if the difference between measurements it above the threshold
      if( abs(State.tProbe - tProbe_previous) > 5.0) {
        // if it is bigger than 5.0 C - restore previous value
        State.tProbe = tProbe_previous;
      }
    }else{
      // there were no fault - reset error counter
      State.err_count = 0;
    }

    (wnd.lblTempr = String(State.tProbe, 1)) += LBL_DEGC;
    ///------------------------------- END OF ALTERNATIVE IMPLEMENTATION ---------------------------

    // show ambient temperature
    (wnd.lblTemprAmb = String(State.tAmbient, 1)) += LBL_DEGC;

    // check if program is in Running state
    if(State.isPgmRunning){

      // Begin() on program should have been called already but still recalculate the Set Point value
      State.tSP = State.ActiveProgram.CalculateSetPoint();

      // make a check for program termination
      if(isnan(State.tSP) || faultCode){
        // meaning an error or reached the end of the program
        digitalWrite(PIN_RELAY, LOW); // turn relay switch off
        State.isRelayOn = false;

        State.StartStop = false;
        wnd.lblStatus = "Program terminating...";
      }else{
        // do common stuff - calculate controlling signal and turn relay on/off accordingly
        State.U = gp_PID.Evaluate(State.tSP, State.tProbe, State.U);
        
        // check whether relay needs to be turned on or off
        if( Configuration.PID.TOL - abs(State.U) < 0.0 ){
          State.isRelayOn = (State.U > 0.0); // beyond tolerance - adjust relay state
          wnd.lblTemprTarget.SetTextColor(DT_C_RED);
        }else{
          State.isRelayOn = false; // within tolerance - turn off relay
          wnd.lblTemprTarget.SetTextColor(DT_C_LIGHTGREEN);
        }
        digitalWrite(PIN_RELAY, (State.isRelayOn ? HIGH : LOW));

        wnd.UpdateProgramDisplay();

        (wnd.lblStatus = "Control U = ") += String(State.U,6);
      }

    } // end if(isPgmRunning)

    // temp? - show ip address in the status bar every 10 seconds?
    //
    if(stsrtr == STS_ROTATE_EVERY) {
      stsrtr = 0;
      (wnd.lblStatus = "IP address: ") += WiFi.localIP().toString();
    }else{
      stsrtr++;
    }
    
    // Send update to all possible connected clients sending them the state
    if(gi_WebSocket.count()) {
      gi_JDoc.clear();
      
      // add message id
      gi_JDoc["id"] = "STS";
      
      // add status and populate fields
      JsonObject joStatus = gi_JDoc["status"].to<JsonObject>();
      joStatus["tPB"] = State.tProbe;
      joStatus["tAM"] = State.tAmbient;
      joStatus["tSP"] = State.tSP;
      joStatus["U"] = State.U;
      joStatus["isRunning"] = State.isPgmRunning;
      joStatus["isRelayOn"] = State.isRelayOn;
      joStatus["stsText"] = wnd.lblStatus.c_str(); // should be safe as serialize is called shortly

      // relevant information about active program
      if(State.ActiveProgram.IsValid()){
        joStatus["actStep"] = (State.ActiveProgram.GetStepsCurrent() + 1);
        joStatus["tmElapsed"] = State.ActiveProgram.GetDurationProgramElapsed();

        // add active program
        JsonObject joProgram = joStatus["actPgm"].to<JsonObject>();
        joProgram[JSConf::TOKEN_PROGRAM_NAME] = State.ActiveProgram.GetName();

        // add steps array and loop through all steps adding their parameters
        JsonArray jaSteps = joProgram[JSConf::TOKEN_PROGRAM_STEPS].to<JsonArray>();
        for(int i = 0; i < State.ActiveProgram.GetStepsTotal(); i++){
          JsonObject joStep = jaSteps.add<JsonObject>();
          TProgramStep* currStep = State.ActiveProgram.GetStep(i);
          joStep[JSConf::TOKEN_PROGRAM_STEP_TSTART] = currStep->GetTStart(); // start temperature
          joStep[JSConf::TOKEN_PROGRAM_STEP_TEND] = currStep->GetTEnd(); // end temperature
          joStep[JSConf::TOKEN_PROGRAM_STEP_DURATION] = currStep->GetDuration(); // duration in ms
        }
      }else{
        // there is no active program
        joStatus["actPgm"] = nullptr;
        joStatus["actStep"] = 0;
        joStatus["tmElapsed"] = 0;
      }

      // send resulting JSON to clients
      size_t jdLen = measureJson(gi_JDoc);
      AsyncWebSocketMessageBuffer * buffer = gi_WebSocket.makeBuffer(jdLen); // allocates buffer that includes 0-terminator
      if(buffer){
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
      DeserializationError err = deserializeJson(gi_JDoc,gi_WsJSONMsg.buffer);

      if(!err){
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
                if(!strcmp(pgmName, Configuration.Programs[i].GetName())){
                  // found a program - create a copy of selected program
                  State.ActiveProgram = Configuration.Programs[i]; // invoke copy constructor

                  // set selected program details
                  wnd.lblProgramName = State.ActiveProgram.GetName();
                  wnd.lblStepTotal = String(State.ActiveProgram.GetStepsTotal());
                  wnd.UpdateProgramDisplay();

                  // end of adjusting controls
                  AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
                  if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Program selected");

                  break; // break for() loop
                }
              }
              if(i == Configuration.nPrograms){
                AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
                if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Program not found");
              }
            }else{
              AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
              if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Empty program");
            }
            // -----------------
          }

        }else if(!strcmp("cfgRD",cmd)){
          // configuration requested - build JSON and send back to client
          gi_JDoc.clear();
          gi_JDoc["id"] = "OK";
          // add nested configuration object
          JsonObject joConfig = gi_JDoc["config"].to<JsonObject>();
          // build running configuration
          Configuration.BuildRunningConfig(joConfig);

          // send out reply
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          // make sure client is still valid and can be communicated to
          if( wsc != nullptr ){
            size_t jdLenOut = measureJson(gi_JDoc);
            AsyncWebSocketMessageBuffer * buffer = gi_WebSocket.makeBuffer(jdLenOut);
            if(buffer){
              serializeJson(gi_JDoc, buffer->get(), jdLenOut + 1);
              wsc->text(buffer);
            }
          }

        }else if(!strcmp("cfgWR",cmd)){
          // configuration submitted from web interface - try building and saving configuration
          JsonObject joConfig = gi_JDoc["msg"];
          if(joConfig){
            // msg object present
            Configuration.UpdateRunningConfig(joConfig, false);
            // notify client that configuration was modified
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Configuration modified");
          }else{
            // malformed request
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Malformed JSON");
          }

        }else if(!strcmp("cfgSV",cmd)){
          // write configuration to flash memory
          File f = LittleFS.open(JSConf::FILE_CONFIGURATION, "w");

          if (f) {
            JsonObject joConfig = gi_JDoc.to<JsonObject>();
            Configuration.BuildRunningConfig(joConfig);
            // save config to file
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

        }else if(!strcmp("pgmRD",cmd)){
          // programs requested - build JSON and send back to client
          gi_JDoc.clear();
          gi_JDoc["id"] = "OK";
          // add nested configuration object
          JsonArray jaPrograms = gi_JDoc["programs"].to<JsonArray>();
          // build running programs
          Configuration.BuildRunningPrograms(jaPrograms);

          // send out reply
          AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
          // make sure client is still valid and can be communicated to
          if( wsc != nullptr ){
            size_t jdLenOut = measureJson(gi_JDoc);
            AsyncWebSocketMessageBuffer * buffer = gi_WebSocket.makeBuffer(jdLenOut);
            if(buffer){
              serializeJson(gi_JDoc, buffer->get(), jdLenOut + 1);
              wsc->text(buffer);
            }
          }

        }else if(!strcmp("pgmWR",cmd)){
          // programs submitted from web interface - try building and saving them to file
          JsonArray jaPrograms = gi_JDoc["msg"];
          if(jaPrograms){
            // msg object present
            Configuration.UpdateRunningPrograms(jaPrograms, false);
            // notify client that configuration was modified
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Programs modified");
          }else{
            // malformed request
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Malformed JSON");
          }

        }else if(!strcmp("pgmSV",cmd)){
          // write programs to flash memory
          File f = LittleFS.open(JSConf::FILE_PROGRAMS, "w");

          if (f) {
            JsonArray jaPrograms = gi_JDoc.to<JsonArray>();
            Configuration.BuildRunningPrograms(jaPrograms);
            // save config to file
            serializeJson(gi_JDoc, f);
            // close the file
            f.close();
            // notify client that configuration was saved to flash
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_OK, "Programs saved to flash");
          }else{
            // notify client that configuration was modified
            AsyncWebSocketClient* wsc = gi_WebSocket.client(gi_WsJSONMsg.client_id);
            if(wsc != nullptr) wsc->printf(JSON_RESPONSE_TEMPLATE_ERR, "Flash write failed");
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

      // end processing incoming message

    } // end if(datalen > 0)

    // so far drop incoming message silently if it was malformed

    gi_WsJSONMsg.datalen = 0;
    gi_WsJSONMsg.client_id = 0;
    gi_WsJSONMsg.status = WSJSONMSG_EMPTY; // mark as available for accomodating new message after all cleanup activities that may take time
  } // done handling incoming message

  // and finally redraw screen if needed: i.e. if timerGUI has fired.
  wnd.Render(false);

} // end of loop()
