/**
 * @file DTSelect.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief Simple select control used to pick up items out of the list
 * @version 0.1
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DTSelect_h
#define DTSelect_h

#include "DTControl.h"
#include "DTLabel.h"

/**
 * @brief A simple DTSelect control to be able to pick items from a list
 * 
 */
class DTSelect : public DTControl
{
    public:
     DTSelect(TFT_eSPI* gfx,
               uint16_t x,
               uint16_t y,
               uint16_t w,
               uint16_t h,
               uint32_t flags,
               uint16_t bkgc,
               uint16_t item_txt_cn,
               uint16_t item_txt_cs,
               uint16_t item_bkg_cn,
               uint16_t item_bkg_cs,
               const GFXfont* f)
      : DTControl(gfx, x, y, w, h, flags)
      {
          // TODO: identify how many item lines can be drawn on the control
          // based on the size of the font used.
          // Also identify how big should the size of labels be.
      }


      virtual ~DTSelect()
      {
          // TODO: clean up by destroying the controls
      }

     // overriding standard DTControl functions
     virtual void Render(); // redraw the entire control
     virtual void Invalidate(); // invalidate impacted controls

     // manipulating items
     virtual void AddItem(); // adds item to the list
     virtual uint16_t GetSelected(); // get an index of currently selected item
     virtual void MoveNext(); // move selection to the next item
     virtual void MovePrev(); // move selection to previous item

    protected:
     const GFXfont* _font; // font to print item text
     uint16_t       _bkgc; // background color of the entire select control
     uint16_t       _i_txt_cn; // text color of the normal list item
     uint16_t       _i_txt_cs; // text color of the selected list item
     uint16_t       _i_bkg_cn; // background color of the normal list item
     uint16_t       _i_bkg_cs; // background color of the selected list item

     // items related
     DTSelectItem*  _items;
     uint16_t       _item_max;
     uint16_t       _current_idx;
};



class DTSelectItem
{
    public:
    protected:
     uint16_t _idx; // an index of the item
     String text; // string to display on the label
     DTSelectItem* next; // pointer to the next item
     DTSelectItem* prev; // pointer to previous item
};




#endif