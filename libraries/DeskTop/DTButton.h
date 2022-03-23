/*
 Basic button class
*/

#ifndef DTButton_h
#define DTButton_h

#include "DTControl.h"

#define DTBUTTON_FPSTR       0x00001000

class DTButton : public DTControl
{
    public:
     DTButton(TFT_eSPI* gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags, uint16_t btnc, uint16_t txtc, const GFXfont* f, const char* txt)
     : DTControl(gfx, x, y, w, h, flags)
     {
        _btn_color = btnc;
        _txt_color = txtc;
        _font = f;

        _lbl_text = txt;
        _flags &= ~DTBUTTON_FPSTR;
     }

     DTButton(TFT_eSPI* gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags, uint16_t btnc, uint16_t txtc, const GFXfont* f, const __FlashStringHelper* rom)
     : DTControl(gfx, x, y, w, h, flags)
     {
        _btn_color = btnc;
        _txt_color = txtc;
        _font = f;
        
        _lbl_rom = rom;
        _flags |= DTBUTTON_FPSTR;
     }
     
     virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);
     virtual void Render();

    protected:
     uint16_t _btn_color; // button color
     uint16_t _txt_color; // label textcolor
     const GFXfont* _font; //
     const __FlashStringHelper* _lbl_rom; // text in ROM
     const char* _lbl_text; // button label limited to 11 characters
};



#endif