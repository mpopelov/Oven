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
// button text labels
static const char BTN_START[] PROGMEM = "Start";
static const char BTN_STOP[] PROGMEM  = "Stop";
static const char BTN_PROG[] PROGMEM  = "Prog";



// test program placeholder
TProgram** PGMList = NULL;
int PGMNo = 3;





/***********************************************
 * Classes for windows used in sketch
 ***********************************************/
// a class showing window that allows selecting programs from memory
class SWindow : public DTWindow {
  public:
  SWindow(TFT_eSPI* gfx, DTDelegate callback) :
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND),
  _Result(false), _callback(callback)
  {

    // Add Up button
    btnUp = new DTButton( gfx, 270, 0, 50, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "UP", 
    DTDelegate::create<SWindow,&SWindow::OnButton_Up>(this) );
    AddControl(btnUp);
    
    // Add OK button
    btnOk = new DTButton( gfx, 270, 51, 50, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, "OK", 
    DTDelegate::create<SWindow,&SWindow::OnButton_Ok>(this) );
    AddControl(btnOk);

    // Add Cancel
    btnCancel = new DTButton( gfx, 270, 102, 50, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_RED, DT_C_BACKGROUND, &FSB2, "X",
    DTDelegate::create<SWindow,&SWindow::OnButton_Cancel>(this) );
    AddControl(btnCancel);

    // Add Down
    btnDown = new DTButton( gfx, 270, 153, 50, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, "DN",
    DTDelegate::create<SWindow,&SWindow::OnButton_Down>(this) );
    AddControl(btnDown);

    // add select control an fill it with data
    selProgs = new DTSelect( gfx, 0, 0, 269, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND,
                              DT_C_LIGHTGREEN, DT_C_BACKGROUND, // text color normal / selected
                              DT_C_BACKGROUND, DT_C_LIGHTGREEN, // item background normal / selected
                              &FSN1);
    
    // add items to select to show them on the screen
    if(PGMList != NULL) for(int idx=0; idx < PGMNo; idx++) selProgs->AddItem(idx, PGMList[idx]->GetName());

    AddControl(selProgs);
  }

  // public callback methods for 2 buttons
  void OnButton_Up() { selProgs->MovePrev(); };
  void OnButton_Down() { selProgs->MoveNext(); };

  void OnButton_Ok() { _Result = true; _callback(); };
  void OnButton_Cancel() { _Result = false; _callback(); };
  

  // public controls
  DTSelect* selProgs;
  DTButton* btnUp;
  DTButton* btnOk;
  DTButton* btnCancel;
  DTButton* btnDown;

  // a variable to tell th result: ok/cancel
  bool _Result;
  
  // a callback to tell parent we are through
  DTDelegate _callback;
};

 
// Main window class showing temperature, running program and a few buttons to have control over the device
class CWindow : public DTWindow {
  public:
  CWindow(TFT_eSPI* gfx) :
  _RelayOn(false), _SelectOn(false), SWnd(NULL),
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_BACKGROUND) {

    // Add program select button
    btnProg = new DTButton( gfx, 250, 0, 70, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREY, DT_C_BACKGROUND, &FSB2, BTN_PROG, 
    DTDelegate::create<CWindow,&CWindow::OnButton_PGM>(this) );
    AddControl(btnProg);

    // Add start/stop button
    btnStart = new DTButton( gfx, 250, 51, 70, 50, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, DT_C_GREEN, DT_C_BACKGROUND, &FSB2, BTN_START,
    DTDelegate::create<CWindow,&CWindow::OnButton_STRTSTP>(this) );
    AddControl(btnStart);
    
    // add status text label
    lblStatus = new DTLabel( gfx, 0, 210, 320, 30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_TOP | DTLABEL_BRDR_LEFT, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSN1, F("Initializing..."));
    AddControl(lblStatus);
    
    // add temperature display label
    lblTempr = new DTLabel( gfx, 0, 0, 210, 210, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_NONE, DT_C_BACKGROUND, DT_C_RED, DT_C_LIGHTGREEN, &FSB4, F("----C"));
    AddControl(lblTempr);
  }


  // callback for select program window to indicate its result
  void OnSelect()
  {
    // either OK or Cancel was clicked - check the result and trigger child window deletion.
    _SelectOn = false;
    if(SWnd->_Result){
      // new program was selected
      // for test - just set button text to idx instead of PGM text
      btnProg->SetText(String(SWnd->selProgs->GetSelected()));
    }else{
      // just cancelled
    }
  };
  
  // callback for program select button
  void OnButton_PGM()
  {
    // make this window not visible and create child window
    Visible(false);
    _SelectOn = true;
    SWnd = new SWindow(_gfx, DTDelegate::create<CWindow,&CWindow::OnSelect>(this));
    SWnd->Invalidate();
  };

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
  };

  // overload Render function
  virtual void Render(bool parentCleared)
  {
    // properly route rendering call
    if(_SelectOn && SWnd != NULL){
      SWnd->Render(parentCleared);
    }else{
      DTWindow::Render(parentCleared);
    }
  };


  
  // callback for start/stop button
  void OnButton_STRTSTP() {
    if(_RelayOn){
      digitalWrite(D1, LOW);
      btnStart->SetText(BTN_START);
      btnStart->SetBtnColor(DT_C_GREEN);
    }else{
      digitalWrite(D1, HIGH);
      btnStart->SetText(BTN_STOP);
      btnStart->SetBtnColor(DT_C_RED);
    }
    _RelayOn = !_RelayOn;
    Invalidate();
  };

  // public labels with temperature and status text
  DTLabel* lblTempr;
  DTLabel* lblStatus;
  DTButton* btnProg;
  DTButton* btnStart;

  bool _RelayOn;
  bool _SelectOn;
  SWindow* SWnd;
};












// file system
FS* fileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();

// K-type temperature sensor instance
TSensor TS(D2, false);  ///< Create an instance of MAX31855 sensor













//global instances
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI* gfx = &tft;
CWindow* wnd = NULL;

unsigned long ticks_TS = 0;
unsigned long ticks_TSENSOR = 0;






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

 TProgram* p = new TProgram(8, "Utility Red");
 p->AddStep(0  , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 595, (2*60+30)*60000); // step 5
 p->AddStep(595, 595, 45*60000); // step 6
 p->AddStep(595, 950, (2*60+30)*60000); // step 7
 p->AddStep(950, 950, 15*60000); // step 8
 PGMList[0] = p;

 p = new TProgram(8, "Utility White");
 p->AddStep(0  , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 595, (2*60+30)*60000); // step 5
 p->AddStep(595, 595, 45*60000); // step 6
 p->AddStep(595, 1000, (2*60+30)*60000); // step 7
 p->AddStep(1000, 1000, 15*60000); // step 8
 PGMList[1] = p;

 p = new TProgram(6, "Glazing");
 p->AddStep(0  , 100, 35*60000); // step 1
 p->AddStep(100, 100, 35*60000); // step 2
 p->AddStep(100, 200, 35*60000); // step 3
 p->AddStep(200, 200, 35*60000); // step 4
 p->AddStep(200, 1250, (3*60)*60000); // step 5
 p->AddStep(1250, 1250, 40*60000); // step 6
 PGMList[2] = p;

 // create main window - all details of setting up elements are in CWindow class ctor
 wnd = new CWindow(gfx);
 wnd->Invalidate();
}




//
// Main loop
//


String strStatus = String("");
String strTempr = String("");


void loop() {

  unsigned long ticks = millis();
  uint16_t x,y;
  bool pressed;

  if ( (ticks - ticks_TSENSOR >= TSENSOR_INTERVAL) )
  {
    // read temperature and update values on screen
    uint8_t faultCode         = TS.readChip();    // read chip updated value and save error for easy access
    double ambientTemperature = TS.getAmbient();  // get updated value of chip ambient temperature
    double probeTemperature   = TS.getProbeLinearized(); // get probe temperature as read by chip
    if (faultCode)                                        // Display error code if present
    {
      if (faultCode & B001) {
        wnd->lblStatus->SetText("Fault: Wire not connected");
      }
      if (faultCode & B010) {
        wnd->lblStatus->SetText("Fault: Short-circuited to Ground (negative)");
      }
      if (faultCode & B100) {
        wnd->lblStatus->SetText("Fault: Short-circuited to VCC (positive)");
      }
    }
    else
    {
      strStatus = String(F("Ambient temperature = ")) + String(ambientTemperature, 1) + " C";
      strTempr = String(probeTemperature, 1) + " C";
      wnd->lblStatus->SetText(strStatus);
      wnd->lblTempr->SetText(strTempr);
    }
    
    // finally save timer value
    ticks_TSENSOR = ticks;
  }

  if ( (ticks - ticks_TS >= TS_USER_TOLERANCE) && gfx->getTouch(&x, &y) )
  {
    wnd->HandleEvent(x,y, true);
    // finally save timer value
    ticks_TS = ticks;
  }
  
  // and redraw screen if needed
  wnd->Render(false);
}
