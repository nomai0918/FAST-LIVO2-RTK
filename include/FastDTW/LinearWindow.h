//
//  LinearWindow.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/5/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__LinearWindow__
#define __FastDTW_x__LinearWindow__

#include "Foundation.h"
#include "SearchWindow.h"
#include "TimeSeries.h"
#include <cmath>
#include "FDMath.h"

FD_NS_START
class LinearWindow : public SearchWindow
{
public:
    template <typename ValueType,JInt nDimension>
    LinearWindow(const TimeSeries<ValueType,nDimension>& tsI, const TimeSeries<ValueType,nDimension>& tsJ, JInt searchRadius):SearchWindow(tsI.size(),tsJ.size())
    {
        JDouble ijRatio = tsI.size()/(JDouble)tsJ.size();
        JBool isIlargest = tsI.size() >= tsJ.size();
        for (JInt i = 0; i<tsI.size(); ++i) {
            if (isIlargest) {
                JInt j = fd_min((JInt)round(i/ijRatio) , tsJ.size() - 1);
                markVisited(i, j);
            }
            else
            {
                JInt maxJ = (JInt)round((i+1)/ijRatio) - 1;
                JInt minJ = (JInt)round(i/ijRatio);
                markVisited(i, minJ);
                markVisited(i, maxJ);
            }
        }
    }
};

FD_NS_END
#endif /* defined(__FastDTW_x__LinearWindow__) */
