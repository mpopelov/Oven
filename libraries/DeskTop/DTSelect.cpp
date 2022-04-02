/**
 * @file DTSelect.cpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief Simple select list implementation
 * @version 0.1
 * @date 2022-04-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "DTSelect.h"

/**
 * @brief adds items to a list to pick from. The control preserves an order in which these items are added so they are displayed in the same order
 * 
 * @param idx caller defined index of an item - this will be returned back upon calling GetSelected()
 * @param txt 
 */
void DTSelect::AddItem(uint16_t idx, const String& txt)
{
    DTSelectItem* itm = new DTSelectItem();
    itm->text = txt;
    itm->idx = idx;
    itm->next = NULL;
    itm->prev = _items_last;

    if(_items_first == NULL){
        _items_first = itm; // case first item is NULL - we are adding the very first item
        _items_current = itm; // make the very first item to appear as current
    }else{
        // we simply have to add item to the end of the list
        _items_last->next = itm;
    }
    _items_last = itm;
}

/**
 * @brief returns value associated with earlier added item or 0xFFFF in case there were no items added to the list
 * 
 * @return uint16_t - index value associated with currently selected item.
 */
uint16_t DTSelect::GetSelected()
{
    if(_items_current == NULL) return 0xFFFF;
    return _items_current->idx;
}

/**
 * @brief Moves internal pointer to the next list item and invalidates the control appropriately
 * 
 */
void DTSelect::MoveNext()
{
    // makes sence to move only in case of available list items
    if(_items_current == NULL) return;

    // only move if there is next element in the list
    if(_items_current->next != NULL){
        _items_current = _items_current->next;
    }
}

/**
 * @brief Moves internal pointer to previous item and invalidates the control appropriately
 * 
 */
void DTSelect::MovePrev()
{
    // only move if there is previous element in the list
    if(_items_current->prev != NULL){
        _items_current = _items_current->prev;
    }
}

/**
 * @brief renders the control
 * 
 */
void DTSelect::Render()
{
    // first check that rendering is at all possible
    if(_lbl_max == 0 || _items_current == NULL) return; // if not - return with no impact

    // verify the first attempt to render - labels should be populated with data respectively
    if(_flags & DTSELECT_FLAGS_INITIALRENDER){
        // populate labels with data starting with current item - normally should be the same as the very first item on the list
        DTSelectItem* itm = _items_current;
        for(int i=0; i<_lbl_max; i++){
            _lbls[i]->SetText(itm->text);

            // set text and background color
            if(itm == _items_current){
                _lbl_cur_idx = i;
                _lbls[i]->SetBackColor(_i_bkg_cs);
                _lbls[i]->SetTextColor(_i_txt_cs);
            }else{
                _lbls[i]->SetBackColor(_i_bkg_cn);
                _lbls[i]->SetTextColor(_i_txt_cn);
            }
            //move pointer
            itm = itm->next;
        }
        // clean up the first render flag after the first run
        _flags &= ~DTSELECT_FLAGS_INITIALRENDER;
    }

    // skip control rendering if it's hidden but do not reset invalidation flag as the control
    // might need to be rendered once visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // first step is to clear the surface.
        // in case parent control has also been invalidated we may skip this step to avoid costly drawing on TFT via SPI bus and avoid screen flickering.
        if(!(_flags & DTCONTROL_FLAGS_PARENT_INVALIDATED)) _gfx->fillRect(_x, _y, _w, _h, _bkgc);

        // redraw all the labels as necessary
        for(int i=0; i < _lbl_max; i++){
            _lbls[i]->Render();
        }

        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATIONRST;
    }
    // nothing to do at this point
}


void DTSelect::Invalidate(bool parentInvalidated)
{
    _flags |= DTCONTROL_FLAGS_INVALIDATED;
    if(parentInvalidated) _flags |= DTCONTROL_FLAGS_PARENT_INVALIDATED;

    for(int i=0; i < _lbl_max; i++){
        _lbls[i]->Invalidate(true);
    }
}