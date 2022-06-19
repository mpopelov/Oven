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
    itm->next = nullptr;
    itm->prev = _items_last;

    if(_items_first == nullptr){
        _items_first = itm; // case first item is nullptr - we are adding the very first item
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
    if(_items_current == nullptr) return 0xFFFF; // indicate we have no real value
    return _items_current->idx;
}

/**
 * @brief Moves internal pointer to the next list item and invalidates the control appropriately
 * 
 */
void DTSelect::MoveNext()
{
    // makes sence to move only in case of available list items
    if(_items_current == nullptr) return;

    // only move if there is next element in the list
    if(_items_current->next != nullptr){
        // move the pointer
        _items_current = _items_current->next;
        // now check if selected label was displayed at the very bottom of the element
        if(_lbl_cur_idx == (_lbl_max-1) ){
            // we are at the end of the visible list - the entire contents of displayed area needs to be changed - including text
            // 1) starting with the last visible label and all way top
            DTSelectItem* itm = _items_current;
            for(int i=_lbl_cur_idx; i>=0 && itm != nullptr; i--){
                *(_lbls[i]) = itm->text;
                _lbls[i]->SetBackColor(i==_lbl_cur_idx ? _i_bkg_cs : _i_bkg_cn);
                _lbls[i]->SetTextColor(i==_lbl_cur_idx ? _i_txt_cs : _i_txt_cn);
                itm = itm->prev;
            }

            // 2) invalidate entire select control to speed up rendering
            Invalidate();
        }else{
            // we can have it easy: jsut update the 2 labels - no need to update the rest
            // 1) adjust current one to no longer show selection
            _lbls[_lbl_cur_idx]->SetBackColor(_i_bkg_cn);
            _lbls[_lbl_cur_idx]->SetTextColor(_i_txt_cn);
            // 2) advance pointer
            _lbl_cur_idx++;
            // 3) now adjust label to show selection
            _lbls[_lbl_cur_idx]->SetBackColor(_i_bkg_cs);
            _lbls[_lbl_cur_idx]->SetTextColor(_i_txt_cs);
            // labels are invalidated by setting colors, no need to invalidate the entire select control
        }
    }
}

/**
 * @brief Moves internal pointer to previous item and invalidates the control appropriately
 * 
 */
void DTSelect::MovePrev()
{
    // makes sence to move only in case of available list items
    if(_items_current == nullptr) return;

    // only move if there is previous element in the list
    if(_items_current->prev != nullptr){
        // move the pointer
        _items_current = _items_current->prev;
        // now check if selected label was displayed at the very top of the element
        if(_lbl_cur_idx == 0 ){
            // we are at the top of the visible list - the entire contents of displayed area needs to be changed - including text
            // 1) starting with the first visible label and all way bottom
            DTSelectItem* itm = _items_current;
            for(int i=0; i<_lbl_max && itm != nullptr; i++){
                *(_lbls[i]) = itm->text;
                _lbls[i]->SetBackColor(i==_lbl_cur_idx ? _i_bkg_cs : _i_bkg_cn);
                _lbls[i]->SetTextColor(i==_lbl_cur_idx ? _i_txt_cs : _i_txt_cn);
                itm = itm->next;
            }

            // 2) invalidate entire select control to speed up rendering
            Invalidate();
        }else{
            // we can have it easy: jsut update the 2 labels - no need to update the rest
            // 1) adjust current one to no longer show selection
            _lbls[_lbl_cur_idx]->SetBackColor(_i_bkg_cn);
            _lbls[_lbl_cur_idx]->SetTextColor(_i_txt_cn);
            // 2) advance pointer
            _lbl_cur_idx--;
            // 3) now adjust label to show selection
            _lbls[_lbl_cur_idx]->SetBackColor(_i_bkg_cs);
            _lbls[_lbl_cur_idx]->SetTextColor(_i_txt_cs);
            // labels are invalidated by setting colors, no need to invalidate the entire select control
        }
    }
}

/**
 * @brief renders the control
 * 
 */
void DTSelect::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as the control
    // might need to be rendered once visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // check that rendering is at all possible
    if(_lbl_max == 0 || _items_current == nullptr) return; // if not - return with no impact

    // verify the first attempt to render - labels should be populated with data respectively
    if(_flags & DTSELECT_FLAGS_INITIALRENDER){
        // populate labels with data starting with current item - normally should be the same as the very first item on the list
        DTSelectItem* itm = _items_current;
        for(int i=0; i<_lbl_max && itm != nullptr; i++, itm = itm->next){
            *(_lbls[i]) = itm->text;

            // set text and background color
            if(itm == _items_current){
                _lbl_cur_idx = i;
                _lbls[i]->SetBackColor(_i_bkg_cs);
                _lbls[i]->SetTextColor(_i_txt_cs);
            }else{
                _lbls[i]->SetBackColor(_i_bkg_cn);
                _lbls[i]->SetTextColor(_i_txt_cn);
            }
        }
        // clean up the first render flag after the first run
        _flags &= ~DTSELECT_FLAGS_INITIALRENDER;
    }

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // first step is to clear the surface.
        // in case parent control has also been invalidated we may skip this step to avoid costly drawing on TFT via SPI bus
        // and avoid screen flickering.
        // also adjust parentCleared value to let children know we have done so alredy
        if(!parentCleared){
            _gfx.fillRect(_x, _y, _w, _h, _bkgc);
            parentCleared = true;
        }

        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }

    // let child controls render - they might have been invalidated
    for(int i=0; i < _lbl_max; i++){
        _lbls[i]->Render(false); // parentCleared flag should have been adjusted by code above
    }
}


void DTSelect::Invalidate()
{
    _flags |= DTCONTROL_FLAGS_INVALIDATED;

    for(int i=0; i < _lbl_max; i++){
        _lbls[i]->Invalidate();
    }
}