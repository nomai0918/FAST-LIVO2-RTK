//
//  PartialWindowMatrix.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/8/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__PartialWindowMatrix__
#define __FastDTW_x__PartialWindowMatrix__

#include "Foundation.h"
#include <vector>
#include <limits>
#include "CostMatrix.h"
#include "SearchWindow.h"
#include "FDAssert.h"

using namespace std;
FD_NS_START
template <typename ValueType>
class PartialWindowMatrix : public CostMatrix<ValueType>
{
    vector<ValueType> _lastCol;
    vector<ValueType> _currCol;
    JInt _currColIndex;
    JInt _minLastRow;
    JInt _minCurrRow;
    const SearchWindow *_window;
public:
    PartialWindowMatrix(const SearchWindow* searchWindow):_window(searchWindow),_lastCol(),_currCol()
    {
        if (searchWindow->maxI() > 0) {
            _currCol.resize(searchWindow->maxJForI(1) - searchWindow->minJForI(1) + 1);
            _currColIndex = 1;
            _minLastRow = searchWindow->minJForI(_currColIndex - 1);
        }
        else
        {
            _currColIndex = 0;
            _minLastRow = 0;
        }
        _minCurrRow = searchWindow->minJForI(_currColIndex);
        _lastCol.resize(searchWindow->maxJForI(0) - searchWindow->minJForI(0) + 1);
    }
    
    
    void put(JInt col, JInt row, ValueType value)
    {
        FDASSERT(row>=_window->minJForI(col)&&row<=_window->maxJForI(col), "CostMatrix is filled in a cell (col=%d, row=%d) that is not in the search window",col, row);
        if (col == _currColIndex) {
            _currCol[row - _minCurrRow] = value;
        }
        else if(col == _currColIndex - 1)
        {
            _lastCol[row - _minLastRow] = value;
        }
        else if(col == _currColIndex + 1)
        {
            _lastCol = _currCol;
            _minLastRow = _minCurrRow;
            _currColIndex ++;
            _currCol.clear();
            _currCol.resize(_window->maxJForI(col) - _window->minJForI(col) + 1);
            _minCurrRow = _window->minJForI(col);
            _currCol[row - _minCurrRow] = value;
        }
    }
    
    ValueType get(JInt col, JInt row) const
    {
        if(row>=_window->minJForI(col) && row<=_window->maxJForI(col))
        {
            if (col == _currColIndex) {
                return _currCol[row - _minCurrRow];
            }
            else if (col == _currColIndex - 1)
            {
                return _lastCol[row - _minLastRow];
            }
        }
        return numeric_limits<ValueType>::max();
    }
    
    JInt size() const
    {
        return _lastCol.size() + _currCol.size();
    }
    
    JInt windowSize() const
    {
        return _window->size();
    }
};
FD_NS_END
#endif /* defined(__FastDTW_x__PartialWindowMatrix__) */
