/**
 * @file DTLabel.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief Label control class
 * @details Basic label class that appears as a line of text, optionally surrounded by borders
 * @version 0.1
 * @date 2022-03-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DTLabel_h
#define DTLabel_h

#include "DTControl.h"

// flags to set borders around label - respect DTCONTROL_FLAGS_XXX
#define DTLABEL_BRDR_TOP    0x00000100 // top border visible
#define DTLABEL_BRDR_BOTTOM 0x00000200 // bottom border visible
#define DTLABEL_BRDR_LEFT   0x00000400 // left border visible
#define DTLABEL_BRDR_RIGHT  0x00000800 // right border visible
#define DTLABEL_BRDR_ALL    0x00000F00 // all borders visible (shortcut for triggering all borders separately)
#define DTLABEL_BRDR_NONE   0x00000000 // no borders visible

typedef enum {LEFT, CENTER, RIGHT} DTTextAlignment;

class DTLabel : public DTControl, public String
{
    public:

    DTLabel(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t flags,
            uint16_t bkgc, uint16_t brdc, uint16_t lblc, const GFXfont* f, const char* lbl)
    : DTControl(gfx, x, y, w, h, flags), String(lbl),
    _bkg_color(bkgc), _brd_color(brdc), _lbl_color(lblc), _alignment(LEFT), _font(f)
    {}

    DTLabel(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t flags,
            uint16_t bkgc, uint16_t brdc, uint16_t lblc, const GFXfont* f, const __FlashStringHelper* lbl)
    : DTControl(gfx, x, y, w, h, flags), String(lbl),
    _bkg_color(bkgc), _brd_color(brdc), _lbl_color(lblc), _alignment(LEFT), _font(f)
    {}

     template <class T>
     String& operator=(T arg) { Invalidate(); return String::operator=(arg); }

     template <class T>
     String& operator+=(T arg) { Invalidate(); return String::operator+=(arg); }

     // set text color
     void SetTextColor(uint16_t c);
     // set background color
     void SetBackColor(uint16_t c);
 
     void SetTextAlignment(DTTextAlignment a) { _alignment = a; }
     
     virtual void Render(bool parentCleared); // redraw the label

    protected:
     uint16_t           _bkg_color; // background color
     uint16_t           _brd_color; // border color
     uint16_t           _lbl_color; // label text color
     DTTextAlignment    _alignment; // label text alignment
     const GFXfont*     _font; // font to use
};

#endif