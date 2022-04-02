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
    itm->prev = _items;
    if(_items != NULL){
        _items->next = itm;
    }else{
        _current_item = itm; // set the first added item to appear as selected
    }
    _items = itm;
}

/**
 * @brief returns value associated with earlier added item or 0xFFFF in case there were no items added to the list
 * 
 * @return uint16_t - index value associated with currently selected item.
 */
uint16_t DTSelect::GetSelected()
{
    if(_current_item == NULL) return 0xFFFF;
    return _current_item->idx;
}

/**
 * @brief Moves internal pointer to the next list item and invalidates the control appropriately
 * 
 */
void DTSelect::MoveNext()
{
    // only move if there is next element in the list
    if(_current_item->next != NULL){
        _current_item = _current_item->next;
    }
}

/**
 * @brief Moves internal pointer to previous item and invalidates the control appropriately
 * 
 */
void DTSelect::MovePrev()
{
    // only move if there is previous element in the list
    if(_current_item->prev != NULL){
        _current_item = _current_item->prev;
    }
}

/**
 * @brief renders the control
 * 
 */
void DTSelect::Render()
{
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