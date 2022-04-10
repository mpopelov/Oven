/*
 Simple desktop class library to handle user interaction for muffle oven controller
 using TFT_eSPI library to handle ILI9341 display (320*240) and touch screen
*/


#ifndef DeskTop_h
#define DeskTop_h

#include "DTControl.h"
#include "DTButton.h"
#include "DTLabel.h"
#include "DTSelect.h"
#include "DTWindow.h"

// define interface colors
#define DT_C_BACKGROUND 0x016A  // RGB(5,44,77)
#define DT_C_GREEN      0x15D2  // RGB(18,182,146)
#define DT_C_LIGHTGREEN 0x9E78  // RGB(150,205,190)
#define DT_C_RED        0xC96B  // RGB(200,45,85)
#define DT_C_GREY       0x4B30  // RGB(70,100,125)

#endif