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
 * @brief flags to control DTSelect behavior
 * 
 */
#define DTSELECT_FLAGS_INITIALRENDER 0x00000100 // set by constructor to let control know it should initially rebuild all the labels


struct DTSelectItem
{
     uint16_t idx;       // an index of the item
     String text;        // string to display on the label
     DTSelectItem* next; // pointer to the next item
     DTSelectItem* prev; // pointer to previous item
};



/**
 * @brief A simple DTSelect control to be able to pick items from a list
 * 
 */
class DTSelect : public DTControl
{
    public:
     DTSelect(TFT_eSPI& gfx,
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
      _items_first(nullptr), _items_last(nullptr), _items_current(nullptr), _lbls(nullptr)
      {
          // set font and its size in order to correctly identify the height of the item label
          gfx.setFreeFont(f);
          gfx.setTextSize(1);
          int16_t sel_lbl_height = 2 + gfx.fontHeight(); // get the height of the font and add 2 pixels for padding around the text

          _lbl_max = _h / sel_lbl_height; // this many items can be displayed given height provided
          if(_lbl_max == 0) return; // nothing to show really
          
          // allocate space and create required controls
          _lbls = new DTLabel*[_lbl_max];
          for(int i=0; i<_lbl_max; i++)
          {
              _lbls[i] = new DTLabel( gfx, // gfx context
                                        x, // all labels are aligned left on control x coord
                     y + i*sel_lbl_height, // labels are sitting one under another without any gap
                                        w, // labels have width of the entire select control
                           sel_lbl_height, // labels have height calculated based on used font height
                         DTCONTROL_FLAGS_VISIBLE | DTCONTROL_FLAGS_INVALIDATED, // set flags to DTLABEL_BRDR_ALL for debug to see actual control placement
                              item_bkg_cn, // initially set to normal background color
                                  TFT_RED, // set border colot to red - should not be visible in PROD
                              item_txt_cn, // normal text color
                                        f, // font
                                        ""); // empty line as text
          }

          _flags |= DTSELECT_FLAGS_INITIALRENDER; // let us know we have to run initial render code
      }

      virtual ~DTSelect()
      {
          // deallocate memory used for labels
          for(int i=0; i<_lbl_max; i++) delete _lbls[i]; // delete all labels
          if(_lbls != nullptr) delete _lbls; // delete array itself

          // deallocate memory used by items
          while(_items_first != nullptr){
              DTSelectItem* itm = _items_first->next;
              delete _items_first;
              _items_first = itm;
          }
      }

     // overriding standard DTControl functions
     virtual void Render(bool parentCleared); // redraw the entire control
     virtual void Invalidate();               // invalidate impacted controls

     // manipulating items
     void AddItem(uint16_t idx, const String& txt); // adds item to the list
     uint16_t GetSelected();                        // get an index of currently selected item
     void MoveNext();                               // move selection to the next item
     void MovePrev();                               // move selection to previous item

    protected:
     const GFXfont* _font;     // font to print item text
     uint16_t       _bkgc;     // background color of the entire select control
     uint16_t       _i_txt_cn; // text color of the normal list item
     uint16_t       _i_txt_cs; // text color of the selected list item
     uint16_t       _i_bkg_cn; // background color of the normal list item
     uint16_t       _i_bkg_cs; // background color of the selected list item

     // items related
     DTSelectItem*  _items_first;   // pointer to the first item in the list
     DTSelectItem*  _items_last;    // pointer to the last item in the list
     DTSelectItem*  _items_current; // currently selected item pointer
     uint16_t       _lbl_max;       // max number of items to display
     uint16_t       _lbl_cur_idx;   // index of the label that currently shows the selection
     DTLabel**      _lbls;          // array of labels displaying text
};

#endif