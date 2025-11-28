//
//  FullWindow.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/5/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__FullWindow__
#define __FastDTW_x__FullWindow__

#include "Foundation.h"
#include "SearchWindow.h"
#include "TimeSeries.h"
FD_NS_START

class FullWindow : public SearchWindow
{
public:
    template <typename ValueType,  JInt nDimension>
    FullWindow(const TimeSeries<ValueType, nDimension>& tsI, const TimeSeries<ValueType, nDimension>& tsJ):SearchWindow(tsI.size(),tsJ.size())
    {
        for (JInt i = 0; i<tsI.size(); ++i) {
            markVisited(i, minJ());
            markVisited(i, maxJ());
        }
    }
};

FD_NS_END
#endif /* defined(__FastDTW_x__FullWindow__) */
