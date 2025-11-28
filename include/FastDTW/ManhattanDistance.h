//
//  ManhattanDistance.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/2/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__ManhattanDistance__
#define __FastDTW_x__ManhattanDistance__

#include "Foundation.h"
#include <vector>
#include "FDAssert.h"
#include "TimeSeriesPoint.h"

FD_NS_START
class ManhattanDistance
{
public:
    ManhattanDistance()
    {
        
    }
    
    template <typename ValueType,JInt nDimension>
    ValueType calcDistance(const MeasurementVector<ValueType, nDimension>& v1, const MeasurementVector<ValueType, nDimension>& v2) const
    {
        FDASSERT0(v1.size()==v2.size(),"ERROR:  cannot calculate the distance between vectors of different sizes.");
        ValueType diffSum = 0;
        size_t size = v1.size();
        for (size_t i = 0; i<size; ++i)
        {
            diffSum += abs(v1[i] - v2[i]);
        }
        return diffSum;
    }
    
    template <typename ValueType>
    ValueType calcDistance(const std::vector<ValueType>& v1, const std::vector<ValueType>& v2) const
    {
        FDASSERT0(v1.size()==v2.size(),"ERROR:  cannot calculate the distance between vectors of different sizes.");
        ValueType diffSum = 0;
        size_t size = v1.size();
        for (size_t i = 0; i<size; ++i)
        {
            diffSum += abs(v1[i] - v2[i]);
        }
        return diffSum;
    }
};
FD_NS_END
#endif /* defined(__FastDTW_x__ManhattanDistance__) */
