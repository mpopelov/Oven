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

/**
 * @brief Basic button class
 * 
 */
class DTButton : public DTControl, public String
{
    public:
    /**
     * @brief Construct a new DTButton object
     * 
     * @param gfx graphical context to use for drawing on the screen
     * @param x top left corner X
     * @param y top left corner Y
     * @param w width
     * @param h height
     * @param flags flags that modify behavior
     * @param btnc button color
     * @param txtc label text color
     * @param f fot to use for label text
     * @param txt text string to show as label
     * @param callback pointer to callback function or nullptr if there is none
     */
    DTButton(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags,
             uint16_t btnc, uint16_t txtc, const GFXfont* f, const char* txt, DTDelegate callback)
    : DTControl(gfx, x, y, w, h, flags), String(txt),
    _btn_color(btnc), _txt_color(txtc), _font(f), _callback(callback)
    {}

    DTButton(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags,
             uint16_t btnc, uint16_t txtc, const GFXfont* f, const __FlashStringHelper* txt, DTDelegate callback)
    : DTControl(gfx, x, y, w, h, flags), String(txt),
    _btn_color(btnc), _txt_color(txtc), _font(f), _callback(callback)
    {}

    virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);
    virtual void Render(bool parentCleared);
    template <class T>
    String& operator=(T arg){ Invalidate(); return String::operator=(arg); }

    void SetBtnColor(uint16_t c);

    protected:
    uint16_t       _btn_color; // button color
    uint16_t       _txt_color; // label textcolor
    const GFXfont* _font; //
    DTDelegate     _callback; // callback function delegate
};

#endif
