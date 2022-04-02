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
      : DTControl(gfx, x, y, w, h, flags),
      _bkgc(bkgc), _i_txt_cn(item_txt_cn), _i_txt_cs(item_txt_cs), _i_bkg_cn(item_bkg_cn), _i_bkg_cs(item_bkg_cs),
      _items(NULL), _lbls(NULL), _current_item(NULL)
      {
          // TODO: identify how many item lines can be drawn on the control
          // based on the size of the font used.
          // Also identify how big should the size of labels be.

          // set font and its size in order to correctly identify the height of the item label
          gfx->setFreeFont(f);
          gfx->setTextSize(1);
          int16_t sel_lbl_height = 2 + gfx->fontHeight(); // get the height of the font and add 2 pixels for padding around the text

          _lbl_max = _h / sel_lbl_height; // this many items can be displayed given height provided
          
          // allocate space and create required controls
          _lbls = new DTLabel*[_lbl_max];
          for(int i=0; i<_lbl_max; i++)
          {
              _lbls[i] = new DTLabel( gfx, // gfx context
                                        x, // all labels are aligned left on control x coord
                     y + i*sel_lbl_height, // labels are sitting one under another without any gap
                                        w, // labels have width of the entire select control
                           sel_lbl_height, // labels have height calculated based on used font height
                         DTLABEL_BRDR_ALL, // set flags to DTLABEL_BRDR_ALL for debug to see actual control placement
                              item_bkg_cn, // initially set to normal background color
                                  TFT_RED, // set border colot to red - should not be visible in PROD
                              item_txt_cn, // normal text color
                                        f, // font
                                        ""); // empty line as text
          }
      }


      virtual ~DTSelect()
      {
          // TODO: clean up by destroying the controls
          for(int i=0; i<_lbl_max; i++) delete _lbls[i]; // delete all labels
          delete _lbls; // delete array itself
      }

     // overriding standard DTControl functions
     virtual void Render();                           // redraw the entire control
     virtual void Invalidate(bool parentInvalidated); // invalidate impacted controls

     // manipulating items
     virtual void AddItem(uint16_t idx, const String& txt);         // adds item to the list
     virtual uint16_t GetSelected(); // get an index of currently selected item
     virtual void MoveNext();        // move selection to the next item
     virtual void MovePrev();        // move selection to previous item

    protected:
     const GFXfont* _font;     // font to print item text
     uint16_t       _bkgc;     // background color of the entire select control
     uint16_t       _i_txt_cn; // text color of the normal list item
     uint16_t       _i_txt_cs; // text color of the selected list item
     uint16_t       _i_bkg_cn; // background color of the normal list item
     uint16_t       _i_bkg_cs; // background color of the selected list item

     // items related
     DTSelectItem*  _items;
     DTSelectItem*  _current_item;
     uint16_t       _lbl_max; // max number of items to display
     DTLabel**      _lbls; // array of labels displaying text
};



class DTSelectItem
{
    public:
     uint16_t idx;      // an index of the item
     String text;        // string to display on the label
     DTSelectItem* next; // pointer to the next item
     DTSelectItem* prev; // pointer to previous item
};




#endif