/**
 * @file DTProgressBar.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief Progress Bar control class
 * @details Basic progress bar class that shows horizontal line surrounded by border with line length set proportionally to given percentage
 * @version 0.1
 * @date 2022-06-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DTProgressBar_h
#define DTProgressBar_h

#include "DTControl.h"

// flags to toggle borders around progress bar - respect DTCONTROL_FLAGS_XXX
#define DTPROGRESSBAR_BRDR_ON   0x00000100  // border visible
#define DTPROGRESSBAR_BRDR_OFF  0x00000000  // border invisible

// default minimum settings of progressbar control
#define DTPROGRESSBAR_RADIUS    3           // default border corner rounding radius
#define DTPROGRESSBAR_HEIGHT_MIN    2*DTPROGRESSBAR_RADIUS + 1       // minimum total height of the control
#define DTPROGRESSBAR_WIDTH_MIN     2*DTPROGRESSBAR_RADIUS + 1       // minimum total width of the control
#define DTPROGRESSBAR_PADV       2 // vertical padding - add 1px empty space between progress bar and border
#define DTPROGRESSBAR_PADH       DTPROGRESSBAR_RADIUS // horizontal padding

class DTProgressBar : public DTControl
{
    public:
     DTProgressBar(TFT_eSPI& gfx,
            uint16_t x,
            uint16_t y,
            uint16_t w,
            uint16_t h,
            uint16_t flags,
            uint16_t bkgc,
            uint16_t brdc,
            uint16_t pbrc)
     : DTControl(gfx, x, y, (w < DTPROGRESSBAR_WIDTH_MIN ? DTPROGRESSBAR_WIDTH_MIN : w), (h < DTPROGRESSBAR_HEIGHT_MIN ? DTPROGRESSBAR_HEIGHT_MIN : h), flags),
     _progress(0), _bkg_color(bkgc), _brd_color(brdc), _pbr_color(pbrc)
     {}

     void SetProgress(uint16_t progress);
     virtual void Render(bool parentCleared); // redraw the label

    protected:
     uint16_t       _bkg_color; // background color
     uint16_t       _brd_color; // border color
     uint16_t       _pbr_color; // label text color
     uint16_t       _progress;  // current progress to show
};

#endif
