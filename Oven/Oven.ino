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

// K-type temperature sensor instance
TSensor TS(D2, false);  ///< Create an instance of MAX31855 sensor



class CWindow : public DTWindow {
  public:
  CWindow(TFT_eSPI* gfx) :
  DTWindow(gfx, 0, 0, 320, 240, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_BLACK) {
    DTControl* cntrl;

    // add navy colorizing button
    cntrl = new DTButton( gfx, 270, 0, 50, 40, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_CYAN, TFT_BLACK, &FF23, F("..."), 
    DTDelegate::create<CWindow,&CWindow::OnButton1>(this) );
    AddControl(cntrl);

    // add black colorizing button
    cntrl = new DTButton( gfx, 270, 41, 50, 40, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, TFT_GREEN, TFT_BLACK, &FF23, F("go"),
    DTDelegate::create<CWindow,&CWindow::OnButton2>(this) );
    AddControl(cntrl);
    
    // add status text label
    lblStatus = new DTLabel( gfx, 0, 210, 320, 30, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_TOP | DTLABEL_BRDR_LEFT, TFT_BLACK, TFT_RED, TFT_WHITE, &FF21, F("Initializing..."));
    AddControl(lblStatus);
    
    // add temperature display label
    lblTempr = new DTLabel( gfx, 0, 0, 210, 210, DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED | DTLABEL_BRDR_NONE, TFT_BLACK, TFT_GREEN, TFT_WHITE, &FF24, F("1250 `C"));
    AddControl(lblTempr);
  }

  // public callback methods for 2 buttons
  void OnButton1() { _bkg_color = TFT_NAVY; Invalidate(); };
  void OnButton2() { _bkg_color = TFT_BLACK; Invalidate(); };

  // public labels with temperature and status text
  DTLabel* lblTempr;
  DTLabel* lblStatus;
};








//global instances
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI* gfx = &tft;
CWindow* wnd;

unsigned long ticks_total = 0;



 // test program placeholder
TProgram PGM_1 = TProgram(8, "Utility Red");






//
// The setup
//
void setup() {

  // debug only
  Serial.begin(115200);
  Serial.println("Starting...");

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
  wnd->Invalidate();

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
  PGM_1.AddStep(0, 100, 35*60*1000); // step 1
  PGM_1.AddStep(1000, 100, 35*60*1000); // step 2
  PGM_1.AddStep(100, 200, 35*60*1000); // step 3
  PGM_1.AddStep(200, 200, 35*60*1000); // step 4
  PGM_1.AddStep(200, 595, 2*60*60*1000+30*60*1000); // step 5
  PGM_1.AddStep(595, 595, 45*60*1000); // step 6
  PGM_1.AddStep(595, 950, 2*60*60*1000+30*60*1000); // step 7
  PGM_1.AddStep(950, 950, 15*60*1000); // step 8
 
  

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
  
  if ( gfx->getTouch(&x, &y) && (ticks - ticks_total >= TS_USER_TOLERANCE) )
  {
    ticks_total = ticks;
    wnd->HandleEvent(x,y, true);
  }

  uint8_t faultCode          = TS.readChip();    // read chip updated value and save error for easy access
  double ambientTemperature = TS.getAmbient();  // get updated value of chip ambient temperature
  double probeTemperature   = TS.getProbe();    // get probe temperature as read by chip
  
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
    strStatus = "Ambient temperature = " + String(ambientTemperature, 1) + " C";
    strTempr = String(probeTemperature, 1) + " C";
    wnd->lblStatus->SetText(strStatus);
    wnd->lblTempr->SetText(strTempr);
  }

  Serial.print("Sensor temperature direct:     ");
  Serial.println(TS.getProbe(),6);

  Serial.print("Sensor temperature linearized: ");
  Serial.println(TS.getProbeLinearized(),6);
  
  Serial.print("Sensor raw value: ");
  // make separate parts of the raw value visible
  uint32_t val = TS.getRaw();
  //               Sppppppppppppp0eSaaaaaaaaaaa0eee
  uint32_t mask = 0b10000000000000000000000000000000;
  
  Serial.print("Raw bitvalue: ");
  for(int i=0; i<32; i++){
    Serial.print( val & mask ? "1" : "0");
    mask >>= 1;
  }

  Serial.println(" ");

  // restore mask
  mask = 0b10000000000000000000000000000000;
  
  Serial.println("-- decode start --");
  Serial.println("Probe temperature data:");
  // probe value sign
  Serial.print( val & mask ? "- " : "+ ");
  mask >>= 1;

  // probe binary value
  for(int i = 0; i<13; i++){
    Serial.print( val & mask ? "1" : "0");
    mask >>= 1;
  }
  uint32_t vu = val >> 18;
  Serial.println(" (" + String(vu) + ")");

  // probe 0 and error bits
  Serial.print("Zero bit: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;

  Serial.print("Error bit: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;

  // ambient temperature
  Serial.println("Ambient temperature data:");
  Serial.println( val & mask ? "- " : "+ ");
  mask >>= 1;

  // ambient binary value
  for(int i = 0; i<11; i++){
    Serial.print( val & mask ? "1" : "0");
    mask >>= 1;
  }
  vu = (val >> 4) & 0b111111111111;
  Serial.println(" (" + String(vu) + ")");

  Serial.println("Error bits:");
  Serial.print("Zero bit: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;
  Serial.print("Short to VCC: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;
  Serial.print("Short to GND: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;
  Serial.print("Open circuit: ");
  Serial.println( val & mask ? "1" : "0");
  mask >>= 1;
  
  Serial.println("-- decode end ----");

  delay(500);
  wnd->Render(false);
}
