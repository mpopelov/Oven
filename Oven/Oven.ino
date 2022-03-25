/*
 * Oven project - a controller for electrical heat oven
*/

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h> // SPI for accessing devices on SPI bus
#include <DeskTop.h> // include for simple desktop class library
#include <LittleFS.h> // include for file system support


// fonts to be used
#define FF21 FreeSansBold9pt7b
#define FF22 FreeSansBold12pt7b
#define FF23 FreeSansBold18pt7b
#define FF24 FreeSansBold24pt7b

// touch screen user action tolerance:
// holding touchscreen for less than this milliseconds will not trigger additional event to GUI
#define TS_USER_TOLERANCE 500


// global identifiers
// file names
static const char FILE_TS_CALIBRATION[] PROGMEM = "/TouchCalData";
static const char FILE_HTTP_INDEX[] PROGMEM = "/index.html";

// file system
FS* fileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();



class CWindow : public DTWindow {
  public:
  CWindow(TFT_eSPI* gfx) :
  DTWindow(gfx, NULL, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_BLACK) {
    DTControl* cntrl;

    // add navy colorizing button
    cntrl = new DTButton( gfx, this, 270, 0, 50, 40, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_CYAN, TFT_BLACK, &FF23, F("..."), 
    DTDelegate::create<CWindow,&CWindow::OnButton1>(this) );
    AddControl(cntrl);

    // add black colorizing button
    cntrl = new DTButton( gfx, this, 270, 41, 50, 40, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_GREEN, TFT_BLACK, &FF23, F("go"),
    DTDelegate::create<CWindow,&CWindow::OnButton2>(this) );
    AddControl(cntrl);
    
    // add status text label
    cntrl = new DTLabel( gfx, this, 0, 210, 320, 30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_TOP | DTLABEL_BRDR_LEFT, TFT_BLACK, TFT_RED, TFT_WHITE, &FF21, F("Initializing..."));
    AddControl(cntrl);
    
    // add temperature display label
    cntrl = new DTLabel( gfx, this, 0, 0, 210, 210, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_NONE, TFT_BLACK, TFT_GREEN, TFT_WHITE, &FF24, F("1250 `C"));
    AddControl(cntrl);
  };
  
  void OnButton1() { _bkg_color = TFT_NAVY; Invalidate(); };
  void OnButton2() { _bkg_color = TFT_BLACK; Invalidate(); };
};








//global instances
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI* gfx = &tft;
CWindow* wnd;

unsigned long ticks_total = 0;








//
// The setup
//
void setup() {

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

  // create main window - all details of setting up elements are in CWindow class ctor
  wnd = new CWindow(gfx);
}




//
// Main loop
//

void loop() {

  unsigned long ticks = millis();
  uint16_t x,y;
  bool pressed;
  
  if ( gfx->getTouch(&x, &y) && (ticks - ticks_total >= TS_USER_TOLERANCE) )
  {
    ticks_total = ticks;
    wnd->HandleEvent(x,y, true);
  }
  
  wnd->Render();
  
  delay(250);

}
