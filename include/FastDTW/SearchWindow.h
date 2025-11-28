//
//  SearchWindow.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/4/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__SearchWindow__
#define __FastDTW_x__SearchWindow__

#include "Foundation.h"
#include <vector>
#include <algorithm>
#include "ColMajorCell.h"
#include "FDAssert.h"
#include "FDMath.h"
FD_NS_START
class SearchWindow;

class SearchWindowIterator
{
    JInt _currentI;
    JInt _currentJ;
    const SearchWindow* _window;
    JBool _hasMoreElements;
    JInt _expectedModCount;
    
protected:
    SearchWindowIterator(const SearchWindow* w);
public:
    ~SearchWindowIterator();
    JBool hasNext() const;
    ColMajorCell next();
    
    friend class SearchWindow;
};

using namespace std;
class SearchWindow
{
    vector<JInt> _minValues;
    vector<JInt> _maxValues;
    JInt _maxJ;
    JInt _size;
    JInt _modCount;
    
    void expandSearchWindow(JInt radius);
public:
    SearchWindow(JInt tsIsize, JInt tsJsize);
    
    virtual ~SearchWindow();
    
    JBool isInWindow(JInt i,JInt j) const;
    
    JInt minI() const;
    
    JInt maxI() const;
    
    JInt minJForI(JInt i) const;

    JInt maxJForI(JInt i) const;
    
    JInt minJ() const;
    
    JInt maxJ() const;
    
    JInt size() const;
    
    JInt getModCount() const;
    
    SearchWindowIterator iterator() const;
    
protected:
    void expandWindow(JInt radius);
    
    void markVisited(JInt col, JInt row);
    
};

SearchWindowIterator::SearchWindowIterator(const SearchWindow* w):_window(w),_hasMoreElements(w->size()>0),_currentI(w->minI()),_currentJ(w->minJ()),_expectedModCount(w->getModCount())
{
    
}

SearchWindowIterator::~SearchWindowIterator()
{
    
}

JBool SearchWindowIterator::hasNext() const
{
    return _hasMoreElements;
}

ColMajorCell SearchWindowIterator::next()
{
    FDASSERT0(_window->getModCount() == _expectedModCount, "ConcurrentModificationException");
    FDASSERT0(_hasMoreElements, "NoSuchElementException");
    ColMajorCell cell(_currentI,_currentJ);
    if(++_currentJ > _window->maxJForI(_currentI))
    {
        if (++_currentI<=_window->maxI()) {
            _currentJ = _window->minJForI(_currentI);
        }
        else
        {
            _hasMoreElements = false;
        }
    }
    return cell;
}

SearchWindow::SearchWindow(JInt tsIsize, JInt tsJsize):_minValues(tsIsize, -1),_maxValues(tsIsize),_maxJ(tsJsize -1),_size(0),_modCount(0)
{
//    fill(_minValues.begin(), _minValues.end(), -1);
//    fill(_maxValues.begin(), _maxValues.end(), 0);
}

SearchWindow::~SearchWindow()
{
    
}

JBool SearchWindow::isInWindow(JInt i, JInt j) const
{
    return (i>=minI())&&(i<=maxI())&&(_minValues[i]<=j)&&(_maxValues[i]>=j);
}

JInt SearchWindow::minI() const
{
    return 0;
}

JInt SearchWindow::maxI() const
{
    return _minValues.size() - 1;
}

JInt SearchWindow::minJ() const
{
    return 0;
}

JInt SearchWindow::maxJ() const
{
    return _maxJ;
}

JInt SearchWindow::minJForI(JInt i) const
{
    return _minValues[i];
}

JInt SearchWindow::maxJForI(JInt i) const
{
    return _maxValues[i];
}

JInt SearchWindow::size() const
{
    return _size;
}

JInt SearchWindow::getModCount() const
{
    return _modCount;
}

SearchWindowIterator SearchWindow::iterator() const
{
    return SearchWindowIterator(this);
}

void SearchWindow::expandSearchWindow(JInt radius)
{
    if (radius >0) {
        // Add all cells in the current Window to an array, iterating through the window and expanding the window
        //    at the same time is not possible because the window can't be changed during iteration through the cells.
        vector<ColMajorCell> windowCells;
        windowCells.reserve(size());
        SearchWindowIterator it = iterator();
        for (; it.hasNext();) {
            windowCells.push_back(it.next());
        }
        for (JInt cell = 0; cell<windowCells.size(); ++cell) {
            ColMajorCell currentCell = windowCells[cell];
            if (currentCell.getCol()!=minI() && currentCell.getRow()!=maxJ()) {// move to upper left if possible
                // Either extend full search radius or some fraction until edges of matrix are met.
                JInt targetCol = currentCell.getCol() - radius;
                JInt targetRow = currentCell.getRow() + radius;
                if (targetCol>=minI()&&targetRow<=maxJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = fd_max(minI() - targetCol, targetRow - maxJ());
                    markVisited(targetCol+cellsPastEdge, targetRow - cellsPastEdge);
                }
            }
            
            if (currentCell.getRow() != maxJ()) {// move up if possible
                JInt targetCol = currentCell.getCol();
                JInt targetRow = currentCell.getRow()+radius;
                if (targetRow <= maxJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = targetRow - maxJ();
                    markVisited(targetCol, targetRow - cellsPastEdge);
                }
            }
            
            if (currentCell.getCol()!=maxI()&&currentCell.getRow()!=maxJ()) {// move to upper-right if possible
                JInt targetCol = currentCell.getCol() + radius;
                JInt targetRow = currentCell.getRow() + radius;
                if (targetCol<=maxI()&&targetRow<=maxJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = fd_max(targetCol - maxI() , targetRow - maxJ());
                    markVisited(targetCol-cellsPastEdge, targetRow-cellsPastEdge);
                }
            }
            if (currentCell.getCol()!=minI()) {// move left if possible
                JInt targetCol = currentCell.getCol() - radius;
                JInt targetRow = currentCell.getRow();
                if (targetCol >= minI()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = minI() - targetCol;
                    markVisited(targetCol+cellsPastEdge, targetRow);
                }
            }
            
            if (currentCell.getCol()!= maxI()) {// move right if possible
                JInt targetCol = currentCell.getCol() + radius;
                JInt targetRow = currentCell.getRow();
                if (targetCol<=maxI()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = targetCol - maxI();
                    markVisited(targetCol - cellsPastEdge, targetRow);
                }
            }
            
            if (currentCell.getCol()!=minI() && currentCell.getRow()!=minJ()) { // move to lower-left if possible
                JInt targetCol = currentCell.getCol() - radius;
                JInt targetRow = currentCell.getRow() - radius;
                if (targetCol>=minI() && targetRow>=minJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = fd_max(minI() - targetCol, minJ() - targetRow);
                    markVisited(targetCol+cellsPastEdge,targetRow+cellsPastEdge);
                }
            }
            
            if (currentCell.getRow()!=minJ()) {// move down if possible
                JInt targetCol = currentCell.getCol();
                JInt targetRow = currentCell.getRow() - radius;
                if (targetRow>=minJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = minJ() - targetRow;
                    markVisited(targetCol, targetRow+cellsPastEdge);
                }
            }
            
            if (currentCell.getCol()!=maxI() && currentCell.getRow() != minJ()) {// move to lower-right if possible
                JInt targetCol = currentCell.getCol() + radius;
                JInt targetRow = currentCell.getRow() - radius;
                if (targetCol<=maxI() && targetRow>=minJ()) {
                    markVisited(targetCol, targetRow);
                }
                else
                {
                    JInt cellsPastEdge = fd_max(targetCol-maxI(), minJ() - targetRow);
                    markVisited(targetCol-cellsPastEdge, targetRow+cellsPastEdge);
                }
            }
        }
    }
    
}

void SearchWindow::expandWindow(JInt radius)
{
    if (radius>0) {
        expandSearchWindow(1);
        expandSearchWindow(radius - 1);
    }
}

void SearchWindow::markVisited(JInt col, JInt row)
{
    if (_minValues[col] == -1) {
        _minValues[col] = row;
        _maxValues[col] = row;
        _size++;
        _modCount++;
    }
    else if(_minValues[col]>row)
    {
        _size+=_minValues[col] - row;
        _minValues[col] = row;
        _modCount++;
    }
    else if(_maxValues[col]<row)
    {
        _size+=row - _maxValues[col];
        _maxValues[col] = row;
        _modCount++;
    }
}

FD_NS_END
#endif /* defined(__FastDTW_x__SearchWindow__) */
