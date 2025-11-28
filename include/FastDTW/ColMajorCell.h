//
//  ColMajorCell.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/4/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__ColMajorCell__
#define __FastDTW_x__ColMajorCell__

#include "Foundation.h"

FD_NS_START

class ColMajorCell
{
    JInt _col;
    JInt _row;
    
public:
    ColMajorCell();
    
    ColMajorCell(JInt col, JInt row);
    
    ColMajorCell(const ColMajorCell& cell);
    
    JInt getCol() const;
    
    JInt getRow() const;
    
    bool operator== (ColMajorCell const& cell) const;
    
    bool operator< (ColMajorCell const& cell) const;
};

ColMajorCell::ColMajorCell():_col(0),_row(0)
{
}

ColMajorCell::ColMajorCell(JInt col, JInt row):_col(col),_row(row)
{
    
}

ColMajorCell::ColMajorCell(const ColMajorCell& cell):_col(cell._col),_row(cell._row)
{
}

JInt ColMajorCell::getCol() const
{
    return _col;
}

JInt ColMajorCell::getRow() const
{
    return _row;
}

bool ColMajorCell::operator== (ColMajorCell const& cell) const
{
    return _col == cell.getCol()&&_row == cell.getRow();
}

bool ColMajorCell::operator< (ColMajorCell const& cell) const
{
    return (getCol()*1024 + getRow()) < (cell.getCol()*1024 + cell.getRow());
}


FD_NS_END
#endif /* defined(__FastDTW_x__ColMajorCell__) */
