#include "DTProgressBar.h"

void DTProgressBar::SetProgress(uint16_t progress)
{
    _progress = progress > 100 ? 100 : progress;
    Invalidate();
}

void DTProgressBar::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // some basic drawing goes here: just clean the surface with back color if parent control was not invalidated
        if(!parentCleared) _gfx.fillRect(_x, _y, _w, _h, _bkg_color);

        // redraw border if flagged on
        if(_flags & DTPROGRESSBAR_BRDR_ON) _gfx.drawRoundRect(_x, _y, _w, _h, DTPROGRESSBAR_RADIUS, _brd_color);

        // draw progress bar itself within rounded recatngular border if present
        // respect offsets and progress percentage
        _gfx.fillRect(_x + DTPROGRESSBAR_PADH,
                       _y + DTPROGRESSBAR_PADV,
                       (_w - 2 * DTPROGRESSBAR_PADH) * _progress / 100,
                       _h - 2 * DTPROGRESSBAR_PADV,
                       _pbr_color);

        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}