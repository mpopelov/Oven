/*
 * Oven project - a controller for electrical heat oven
*/

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h> // SPI for accessing devices on SPI bus
#include <DeskTop.h> // include for simple desktop class library
#include <LittleFS.h> // include for file system support

// global identifiers
// file names
static const char FILE_TS_CALIBRATION[] PROGMEM = "/TouchCalData";
static const char FILE_HTTP_INDEX[] PROGMEM = "/index.html";

// file system
FS* fileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();


// fonts to be used
#define FF21 FreeSansBold9pt7b
#define FF22 FreeSansBold12pt7b
#define FF23 FreeSansBold18pt7b
#define FF24 FreeSansBold24pt7b















//global instances
TFT_eSPI* gfx;
DTWindow* wnd;




//
// The setup
//
void setup() {

  // initialize file system
  fileSystemConfig.setAutoFormat(false);
  fileSystem->setConfig(fileSystemConfig);
  if(!fileSystem->begin()){
    fileSystem->format();
    fileSystem->begin();
  }

  // calibrate touch screen

  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check if calibration file exists and size is correct
  if (fileSystem->exists(FPSTR(FILE_TS_CALIBRATION))) {
      File f = fileSystem->open(FPSTR(FILE_TS_CALIBRATION), "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
    }
  }

  // initialize screen
  gfx = new TFT_eSPI();
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
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }

  uint32_t flags = DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED;

  // create a single element on the screen so far : button
  DTControl* cntrl;
  wnd = new DTWindow(gfx, 0, 0, 320, 240, flags, TFT_GREEN);
  
  // add button control
  cntrl = new DTButton( gfx, 270, 0, 50, 40, flags, TFT_CYAN, TFT_BLACK, &FF23, F("...") );
  wnd->AddControl(cntrl);

  // add button control
  cntrl = new DTButton( gfx, 270, 41, 50, 40, flags, TFT_GREEN, TFT_BLACK, &FF23, F("go") );
  wnd->AddControl(cntrl);

  // add status text label
  cntrl = new DTLabel( gfx, 0, 210, 320, 30, flags | DTLABEL_BRDR_TOP | DTLABEL_BRDR_LEFT, TFT_BLACK, TFT_RED, TFT_WHITE, &FF21, F("Initializing..."));
  wnd->AddControl(cntrl);

  // add temperature display label
  cntrl = new DTLabel( gfx, 0, 0, 210, 210, flags | DTLABEL_BRDR_NONE, TFT_BLACK, TFT_GREEN, TFT_WHITE, &FF24, F("1250 `C"));
  wnd->AddControl(cntrl);
}




//
// Main loop
//
void loop() {
  uint16_t x,y;
  bool pressed;

  if (gfx->getTouch(&x, &y))
  {
    wnd->HandleEvent(x,y, true);
  }else{
    wnd->HandleEvent(x,y, false);
  }
  
  wnd->Render();
  
  delay(250);

}
