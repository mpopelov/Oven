/**
 * @file DTButton.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief Basic button class
 * @version 0.1
 * @date 2022-03-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DTButton_h
#define DTButton_h

#include "DTControl.h"

#define DTBUTTON_FPSTR       0x00001000

/**
 * @brief Basic button class
 * 
 */
class DTButton : public DTControl
{
    public:
    /**
     * @brief Construct a new DTButton object
     * 
     * @param gfx graphical context to use for drawing on the screen
     * @param owner owner control or NULL for the root one
     * @param x top left corner X
     * @param y top left corner Y
     * @param w width
     * @param h height
     * @param flags flags that modify behavior
     * @param btnc button color
     * @param txtc label text color
     * @param f fot to use for label text
     * @param txt text string to show as label
     * @param callback pointer to callback function or NULL if there is none
     */
     DTButton(TFT_eSPI* gfx,
             DTControl* owner,
               uint16_t x,
               uint16_t y,
               uint16_t w,
               uint16_t h,
               uint32_t flags,
               uint16_t btnc,
               uint16_t txtc,
               const GFXfont* f,
               const char* txt,
               DTDelegate callback)
     : DTControl(gfx, owner, x, y, w, h, flags)
     {
        _btn_color = btnc;
        _txt_color = txtc;
        _font = f;

        _lbl_text = txt;
        _flags &= ~DTBUTTON_FPSTR;
        _callback = callback;
     }

    /**
     * @brief Construct a new DTButton object
     * 
     * @param gfx graphical context to use for drawing on the screen
     * @param owner owner control or NULL for the root one
     * @param x top left corner X
     * @param y top left corner Y
     * @param w width
     * @param h height
     * @param flags flags that modify behavior
     * @param btnc button color
     * @param txtc label text color
     * @param f fot to use for label text
     * @param rom text string in PROGMEM to show as label
     */
     DTButton(TFT_eSPI* gfx,
             DTControl* owner,
               uint16_t x,
               uint16_t y,
               uint16_t w,
               uint16_t h,
               uint32_t flags,
               uint16_t btnc,
               uint16_t txtc,
               const GFXfont* f,
               const __FlashStringHelper* rom,
               DTDelegate callback)
     : DTControl(gfx, owner, x, y, w, h, flags)
     {
        _btn_color = btnc;
        _txt_color = txtc;
        _font = f;
        
        _lbl_rom = rom;
        _flags |= DTBUTTON_FPSTR;
        _callback = callback;
     }
     
     virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);
     virtual void Render();

    protected:
     uint16_t _btn_color; // button color
     uint16_t _txt_color; // label textcolor
     const GFXfont* _font; //
     const __FlashStringHelper* _lbl_rom; // text in ROM
     const char* _lbl_text; // button label limited to 11 characters
     DTDelegate _callback; // pointer? to callback function
};



#endif