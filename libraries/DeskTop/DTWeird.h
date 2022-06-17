#ifndef DTWeird_h
#define DTWeird_h

#include <Arduino.h>
#include "DTControl.h"

class DTWeird : public DTControl, public String {
    public:
    DTWeird(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t flags, const char* str)
    : DTControl(gfx, x, y, w, h, flags), String(str)
    {}
    DTWeird(TFT_eSPI& gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t flags, const __FlashStringHelper* str)
    : DTControl(gfx, x, y, w, h, flags), String(str)
    {}

    virtual void Render(bool parentCleared);
};

#endif