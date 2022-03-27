/*
 Basic label class
*/

#ifndef DTLabel_h
#define DTLabel_h

#include "DTControl.h"

// flags to set borders around label - respect DTCONTROL_FLAGS_XXX
#define DTLABEL_BRDR_TOP    0x00000100
#define DTLABEL_BRDR_BOTTOM 0x00000200
#define DTLABEL_BRDR_LEFT   0x00000400
#define DTLABEL_BRDR_RIGHT  0x00000800
#define DTLABEL_BRDR_ALL    0x00000F00
#define DTLABEL_BRDR_NONE   0x00000000
#define DTLABEL_FPSTR       0x00001000

class DTLabel : public DTControl
{
    public:
     DTLabel(TFT_eSPI* gfx,
            uint16_t x,
            uint16_t y,
            uint16_t w,
            uint16_t h,
            uint16_t flags,
            uint16_t bkgc,
            uint16_t brdc,
            uint16_t lblc,
            const GFXfont* f,
            const char* lbl)
     : DTControl(gfx, x, y, w, h, flags), _bkg_color(bkgc), _brd_color(brdc), _lbl_color(lblc), _font(f), _lbl_text(lbl)
     {
        _flags &= ~DTLABEL_FPSTR;
     }

     DTLabel(TFT_eSPI* gfx,
              uint16_t x,
              uint16_t y,
              uint16_t w,
              uint16_t h,
              uint16_t flags,
              uint16_t bkgc,
              uint16_t brdc,
              uint16_t lblc,
              const GFXfont* f,
              const __FlashStringHelper* rom)
     : DTControl(gfx, x, y, w, h, flags), _bkg_color(bkgc), _brd_color(brdc), _lbl_color(lblc), _font(f), _lbl_rom(rom)
     {
        _flags |= DTLABEL_FPSTR;
     }

     // change label text
     virtual void SetText(const char* t);
     
     // virtual bool HandleEvent(...) is not provided - inheriting from base class as we are not supposed to react to clicks;
     virtual void Render(); // redraw the label

    protected:
     uint16_t _bkg_color; // background color
     uint16_t _brd_color; // border color
     uint16_t _lbl_color; // label text color
     const GFXfont* _font; // font to use
     const char* _lbl_text; // text reference
     const __FlashStringHelper* _lbl_rom; // text in ROM
};

#endif