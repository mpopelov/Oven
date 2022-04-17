/*
 * Oven project - a controller for electrical heat oven
*/

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h> // SPI for accessing devices on SPI bus
#include <DeskTop.h> // include for simple desktop class library
#include <LittleFS.h> // include for file system support
#include <TSensor.h> // include for K-type sensor

#include <PIDControl.h>
#include <TProgram.h>


// fonts to be used
#define FSN1 FreeSans9pt7b // for status text and small labels
#define FSB1 FreeSansBold9pt7b // bold font for small text

#define FSN2 FreeSans12pt7b // for medium sized text
#define FSB2 FreeSansBold12pt7b // for medium sized text in bold

#define FSB3 FreeSansBold18pt7b
#define FSB4 FreeSansBold24pt7b

// different tolerances for reactions
#define TS_USER_TOLERANCE   300 // touch screen user action tolerance: holding touchscreen for less than this milliseconds will not trigger additional event to GUI
#define TSENSOR_INTERVAL    1000 // only measure temperature that often


/***********************************************
 * Global string constants
 ***********************************************/
// file names
static const char FILE_TS_CALIBRATION[] PROGMEM = "/TouchCalData";
static const char FILE_HTTP_INDEX[] PROGMEM = "/index.html";
// button and label texts
static const char BTN_START[] PROGMEM         = "Start";
static const char BTN_STOP[] PROGMEM          = "Stop";
static const char BTN_PROG[] PROGMEM          = "Prog";
static const char LBL_EMPTY[] PROGMEM         = "---";
static const char LBL_TEMPRMPTY[] PROGMEM     = "----C";
static const char LBL_TIMEREMPTY[] PROGMEM    = "00:00:00";
static const char LBL_PROG[] PROGMEM          = "Program: ";
static const char LBL_STEP[] PROGMEM          = "Step: ";
static const char LBL_OF[] PROGMEM            = " of ";
static const char LBL_STEPTARGET[] PROGMEM    = "Step target: ";
static const char LBL_STEPTIME[] PROGMEM      = "Step remaining: ";
static const char LBL_TIMEREMAINING[] PROGMEM = "Program remaining: ";


// test program placeholder
TProgram** PGMList = NULL;
TProgram* ActiveProgram = NULL; // current active program
PIDControl* PID = NULL; // pid control instance
double U = 0; // current controlling signal value

int PGMNo = 4;

volatile bool IsPgmRunning = false; // gloabl flag for signalling controller program state
volatile bool PgmHasStarted = false; // check if begin() was properly called


// file system
FS* fileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();

// K-type temperature sensor instance
TSensor TS(D2, false);  ///< Create an instance of MAX31855 sensor




//global instances
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI* gfx = &tft;

unsigned long ticks_TS = 0;
unsigned long ticks_TSENSOR = 0;
unsigned long ticks_PGM = 0;
unsigned long ticks_PGM_Total = 0;




/***********************************************
 * Classes for GUI windows used in sketch
 ***********************************************/

// a class showing window that allows selecting programs from memory
class SWindow : public DTWindow {
  public:
  SWindow(TFT_eSPI* gfx, DTDelegate callback) :
  DTWindow( gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
      btnUp( gfx, 270,   0,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "UP", DTDelegate::create<SWindow,&SWindow::OnButton_Up>(this)),
      btnOk( gfx, 270,  51,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, "OK", DTDelegate::create<SWindow,&SWindow::OnButton_Ok>(this)),
  btnCancel( gfx, 270, 102,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_RED, DT_C_BACKGROUND, &FSB2, "X", DTDelegate::create<SWindow,&SWindow::OnButton_Cancel>(this)),
    btnDown( gfx, 270, 153,  50,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "DN", DTDelegate::create<SWindow,&SWindow::OnButton_Down>(this)),
   selProgs( gfx,   0,   0, 269, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_LIGHTGREEN, DT_C_BACKGROUND, DT_C_BACKGROUND, DT_C_LIGHTGREEN, &FSN1),
  _Result(false), _callback(callback)
  {
    AddControl(&btnUp);
    AddControl(&btnOk);
    AddControl(&btnCancel);
    AddControl(&btnDown);
    AddControl(&selProgs);
    if(PGMList != NULL) for(int idx=0; idx < PGMNo; idx++) selProgs.AddItem(idx, PGMList[idx]->GetName());
  }

  // public callback methods for 2 buttons
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



// Main window class showing temperature, running program and a few buttons to have control over the device
class CWindow : public DTWindow {
  public:
  CWindow(TFT_eSPI* gfx) :
  _RelayOn(false), _SelectOn(false), SWnd(NULL),
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
  // initialize child elements
              btnProg( gfx, 250,   0,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, BTN_PROG, DTDelegate::create<CWindow,&CWindow::OnButton_PGM>(this)),
             btnStart( gfx, 250,  51,  70,  50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, BTN_START, DTDelegate::create<CWindow,&CWindow::OnButton_STRTSTP>(this)),
  // program details - Y offset 0
           lblProgram( gfx,   0,   0,  80,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_PROG),
       lblProgramName( gfx,  80,   0, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
  // step details - Y offset 25
              lblStep( gfx,   0,  25,  50,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_STEP),
        lblStepNumber( gfx,  50,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
            lblStepOf( gfx,  80,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_OF),
         lblStepTotal( gfx, 110,  25,  30,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_EMPTY),
  // temperature main display - Y offset 50
             lblTempr( gfx,   0,  50, 210, 110, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSB4, LBL_TEMPRMPTY),
       lblTemprTarget( gfx, 135, 135,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_RED, &FSN1, LBL_TEMPRMPTY),
  // step timing values - Y offset 160
          lblStepTime( gfx,   0, 160, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_STEPTIME),
     lblStepTimeValue( gfx, 170, 160,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMPTY),
  // displaying program timer - Y offset 185
       lblProgramTime( gfx,   0, 185, 170,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMAINING),
  lblProgramTimeValue( gfx, 170, 185,  75,  25, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, LBL_TIMEREMPTY),
  // status label - Y offset 210
            lblStatus( gfx,   0, 210, 320,  30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, F("Initializing..."))
  {
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
    _SelectOn = false;
    if(SWnd->_Result){
      // new program was selected
      // update active program with the one selected only if controller is not in running mode
      if(!IsPgmRunning){
       uint16_t _pgmIdx = SWnd->selProgs.GetSelected();
       if(_pgmIdx < PGMNo) ActiveProgram = PGMList[_pgmIdx];
       // set selected program details
       lblProgramName.SetText(ActiveProgram->GetName());
       lblStepTotal.SetText(String(ActiveProgram->GetStepsTotal()));
       lblStepNumber.SetText(String(ActiveProgram->GetStepsCurrent()));

       // show initial duration of selected program
       char buff[12];
       unsigned long t = ActiveProgram->GetDurationTotal();
       snprintf(buff, 12, "%02u:%02u:%02u", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
       lblProgramTimeValue.SetText(String(buff));
       // end of adjusting controls
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
    _SelectOn = true;
    SWnd = new SWindow(_gfx, DTDelegate::create<CWindow,&CWindow::OnSelect>(this));
    SWnd->Invalidate();
  }

  // overload HandleEvent function
  virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed)
  {
    if(_SelectOn && SWnd != NULL){
      //pass event to select window
      bool res = SWnd->HandleEvent(x, y, pressed);
      // select might have processed the event and could have signalled we shall destroy it
      if(!_SelectOn){
        // flag was re-set meaning we shall destroy child window
        delete SWnd;
        SWnd = NULL;
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
    if(_SelectOn && SWnd != NULL){
      SWnd->Render(parentCleared);
    }else{
      DTWindow::Render(parentCleared);
    }
  }


  
  // callback for start/stop button
  void OnButton_STRTSTP() {
    if(IsPgmRunning){
      // program running - emergency stop
      digitalWrite(D1, LOW);
      IsPgmRunning = false;
      _RelayOn = false;
      btnStart.SetText(BTN_START);
      btnStart.SetBtnColor(DT_C_GREEN);
    }else{
      // program is not running - start active program and let PID controller decide on relay state.
      if(ActiveProgram != NULL){
        // only start things if there is an active program selected
        // so far just raise the flag and signal program handling code to du all necessary steps to update screen
        // and calculate setpoint and control action
        IsPgmRunning = true;
        btnStart.SetText(BTN_STOP);
        btnStart.SetBtnColor(DT_C_RED);
        // test case - enable relay and set 1st step
        // digitalWrite(D1, HIGH);
      }
      // should there be some message to user about the need to select progam ?
    }
    Invalidate();
  }

  /*
  / Members holding all control elements
  */
  // buttons
  DTButton btnProg;
  DTButton btnStart;
  
  // labels
  DTLabel lblTempr; // current measured temperature
  DTLabel lblTemprTarget; // current target temperature (SetPoint)
  DTLabel lblStatus; // status text label at the bottom of the screen
  DTLabel lblProgram; // Label "Program; "
  DTLabel lblProgramName; // actual program name
  DTLabel lblStep; // Label "Step: "
  DTLabel lblStepNumber; // current step number
  DTLabel lblStepOf; // 
  DTLabel lblStepTotal; // total number of steps

  DTLabel lblStepTime; // total number of steps
  DTLabel lblStepTimeValue; // total number of steps
  DTLabel lblProgramTime; // total number of steps
  DTLabel lblProgramTimeValue; // total number of steps

  bool _RelayOn;
  bool _SelectOn;
  SWindow* SWnd;
};

CWindow* wnd = NULL;


//
// The setup
//
void setup() {

  // set relay pin
  pinMode(D1, OUTPUT); // set relay pin mode to output
  digitalWrite(D1, LOW); // turn relay off

  // debug only
  //Serial.begin(115200);
  //Serial.println("Starting...");

  uint16_t calData[5]; // buffer for touchscreen calibration data
  uint8_t calDataOK = 0; // calibration data reding status

  // initialize file system
  fileSystemConfig.setAutoFormat(false);
  fileSystem->setConfig(fileSystemConfig);
  if(!fileSystem->begin()){
    fileSystem->format();
    fileSystem->begin();
  }

  // check if calibration file exists and size is correct
  if (fileSystem->exists(FPSTR(FILE_TS_CALIBRATION))) {
      File f = fileSystem->open(FPSTR(FILE_TS_CALIBRATION), "r");
      if (f) {
        if (f.readBytes((char *)calData, 10) == 10)
          calDataOK = 1;
        f.close();
    }
  }

  // initialize screen
  gfx->init();
  gfx->setRotation(1);  
  gfx->fillScreen(TFT_BLACK);

  if (calDataOK) {
    // calibration data valid
    gfx->setTouch(calData);
  } else {
    // data not valid so recalibrate
    gfx->setCursor(20, 0);
    gfx->setTextFont(2);
    gfx->setTextSize(1);
    gfx->setTextColor(TFT_WHITE, TFT_BLACK);

    gfx->println("Touch corners as indicated");

    gfx->setTextFont(1);
    gfx->println();

    gfx->calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    gfx->setTextColor(TFT_GREEN, TFT_BLACK);
    gfx->println("Calibration complete!");

    // store data
    File f = fileSystem->open(FPSTR(FILE_TS_CALIBRATION), "w");
    if (f) {
      f.write((const unsigned char *)calData, 10);
      f.close();
    }
  }

 // test programs to add and run on oven:
 // objects created in global scope for tests
/*
 * Prog name  | temp range  | temp range  | temp range  | temp range  | temp range  | temp range  | temp range  | temp range  |
 *            | duration    | duration    | duration    | duration    | duration    | duration    | duration    | duration    |
 * -----------+-------------+-------------+-------------+-------------+-------------+-------------+-------------+-------------|
 * utility R  | 0-100       | 100-100     | 100-200     | 200-200     | 200-595     | 595-595     | 595-950     | 950-950     |
 *            | 35 min      | 35 min      | 35 min      | 35 min      | 2 h 30 min  | 45 min      | 2 h 30 min  | 15 min      |
 *            
 * utility W  | 0-100       | 100-100     | 100-200     | 200-200     | 200-595     | 595-595     | 595-1000    | 1000-1000   |
 *            | 35 min      | 35 min      | 35 min      | 35 min      | 2 h 30 min  | 45 min      | 2 h 30 min  | 15 min      |
 * 
 * glazing    | 0-100       | 100-100     | 100-200     | 200-200     | 200-max     | max         |             |             |
 *            | 35 min      | 35 min      | 35 min      | 35 min      | 3 h 00 min  | 40 min      |             |             | 
 * 
 */

 PGMList = new TProgram*[PGMNo];

 TProgram* p = new TProgram(3, "Testing PID");
 p->AddStep(28 , 100, 1*60000); // step 1
 p->AddStep(100, 100, 1*60000); // step 2
 p->AddStep(100, 40, 1*60000); // step 3
 p->Reset();
 PGMList[0] = p;

 p = new TProgram(8, "Utility Red");
 p->AddStep(28 , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 595, (2*60+30)*60000); // step 5
 p->AddStep(595, 595, 45*60000); // step 6
 p->AddStep(595, 950, (2*60+30)*60000); // step 7
 p->AddStep(950, 950, 15*60000); // step 8
 p->Reset();
 PGMList[1] = p;

 p = new TProgram(8, "Utility White");
 p->AddStep(28 , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 595, (2*60+30)*60000); // step 5
 p->AddStep(595, 595, 45*60000); // step 6
 p->AddStep(595, 1000, (2*60+30)*60000); // step 7
 p->AddStep(1000, 1000, 15*60000); // step 8
 p->Reset();
 PGMList[2] = p;

 p = new TProgram(6, "Glazing");
 p->AddStep(28  , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 1250, (3*60)*60000); // step 5
 p->AddStep(1250, 1250, 40*60000); // step 6
 p->Reset();
 PGMList[3] = p;

 // create main window - all details of setting up elements are in CWindow class ctor
 wnd = new CWindow(gfx);
 wnd->Invalidate();
}




//
// Main loop
//


String strStatus = String("");
String strTempr = String("");

#define BUFF_LEN 32

void loop() {

  unsigned long ticks = millis();
  uint16_t x,y;
  bool pressed;
  char buff[BUFF_LEN];

  // allow handling touch screen events early to allow emergency stop as early as possible
  if ( (ticks - ticks_TS >= TS_USER_TOLERANCE) && gfx->getTouch(&x, &y) )
  {
    wnd->HandleEvent(x,y, true);
    // finally save timer value
    ticks_TS = ticks;
  }

  // now the main functionality
  // measure temperature / apply control if needed / update screen elements
  if ( ticks - ticks_TSENSOR >= TSENSOR_INTERVAL )
  {
    // read temperature and update values on screen
    uint8_t faultCode         = TS.readChip();    // read chip updated value and save error for easy access
    double ambientTemperature = TS.getAmbient();  // get updated value of chip ambient temperature
    double probeTemperature   = TS.getProbeLinearized(); // get probe temperature as read by chip
    double SetPointTemperature = NAN;                       // current set point as returned by program running

    if (faultCode)                                        // Display error code if present
    {
      if (faultCode & B001) {
        wnd->lblStatus.SetText("Fault: Wire not connected");
      }
      if (faultCode & B010) {
        wnd->lblStatus.SetText("Fault: Short-circuited to Ground (negative)");
      }
      if (faultCode & B100) {
        wnd->lblStatus.SetText("Fault: Short-circuited to VCC (positive)");
      }
    }
    else
    {
      //strStatus = String(F("Ambient temperature = ")) + String(ambientTemperature, 1) + " C";
      strTempr = String(probeTemperature, 1) + " C";
    }

    // check if program has to be run (and only if active program is properly set)
    if(IsPgmRunning && ActiveProgram != NULL){
      // check if this is the first time program starts
      if(PgmHasStarted){
        // do the normal routine
        SetPointTemperature = ActiveProgram->CalculateSetPoint();

      }else{
        // this is the initial step in the program
        PID = new PIDControl( 1, 1, 1, TSENSOR_INTERVAL);
        SetPointTemperature = ActiveProgram->Begin();
        wnd->lblTemprTarget.Visible(true);
        PgmHasStarted = true;
      }

      // make a check for program termination
      if(isnan(SetPointTemperature)){
        // meaning an error or reached the end of the program
        IsPgmRunning = false;
        strStatus = "Program terminating...";
      }else{
        // do common staff - calculate controlling signal and turn relay on/off accordingly
        U = PID->Evaluate(SetPointTemperature, probeTemperature, U);
        digitalWrite(D1, (U > 0.0 ? HIGH : LOW));

        // update labels' values
        wnd->lblTemprTarget.SetText(String(SetPointTemperature,2) + " C");
        wnd->lblStepNumber.SetText(String(ActiveProgram->GetStepsCurrent()));

        // update program remaining time
        unsigned long t = ActiveProgram->GetDurationTotal() - ActiveProgram->GetDurationElapsed();
        snprintf(buff, BUFF_LEN, "%02u:%02u:%02u", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
        wnd->lblProgramTimeValue.SetText(buff);

        // update step remaining time
        t = ActiveProgram->GetDurationElapsedStep();
        snprintf(buff, BUFF_LEN, "%02u:%02u:%02u", TPGM_MS_HOURS(t), TPGM_MS_MINUTES(t), TPGM_MS_SECONDS(t));
        wnd->lblStepTimeValue.SetText(buff);

        strStatus = String("Control U = ") + String(U,6);
      }

    }else{

      // check if running flag was reset but program was actually running - take actions to stop everything and reset program
      if(PgmHasStarted){
        // stop activity on heater if any and reset program
        digitalWrite(D1, LOW); // turn relay switch off
        if(ActiveProgram != NULL) ActiveProgram->Reset();
        // reset PID control
        if(PID != NULL){
          delete PID;
          PID = NULL;
        }
        U = 0.0; // reset controlling signal
        PgmHasStarted = false;
        wnd->lblTemprTarget.Visible(false);
        strStatus = "Program has ended";
        // reset GUI button
        wnd->_RelayOn = false;
        wnd->btnStart.SetText(BTN_START);
        wnd->btnStart.SetBtnColor(DT_C_GREEN);
        wnd->Invalidate();
      }
    }

    // update values on the screen
    
    wnd->lblStatus.SetText(strStatus); // update status text at the bottom of the screen
    wnd->lblTempr.SetText(strTempr); // update temperature value

    // finally save timer value
    ticks_TSENSOR = ticks;
  }

  // and finally redraw screen if needed
  wnd->Render(false);
}
