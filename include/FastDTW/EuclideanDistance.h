//
//  EuclideanDistance.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/2/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__EuclideanDistance__
#define __FastDTW_x__EuclideanDistance__

#include "FDAssert.h"
#include "FDMath.h"
#include <cmath>
#include "TimeSeriesPoint.h"

FD_NS_START
class EuclideanDistance
{
public:
    EuclideanDistance()
    {
        
    }
    
    template <typename ValueType,JInt nDimension>
    ValueType calcDistance(const MeasurementVector<ValueType, nDimension>& v1, const MeasurementVector<ValueType, nDimension>& v2) const
    {
        FDASSERT0(v1.size()==v2.size(),"ERROR:  cannot calculate the distance between vectors of different sizes.");
        double sqSum = 0.0;
        size_t size = v1.size();
        for (JInt i = 0; i<size; ++i) {
            sqSum+= pow((double)(v1[i]-v2[i]), 2.0);
        }
        return (ValueType)sqrt(sqSum);
    }
    
    template <typename ValueType>
    ValueType calcDistance(const std::vector<ValueType>& v1, const std::vector<ValueType>& v2) const
    {
        FDASSERT0(v1.size()==v2.size(),"ERROR:  cannot calculate the distance between vectors of different sizes.");
        double sqSum = 0.0;
        size_t size = v1.size();
        for (size_t i = 0; i<size; ++i) {
            sqSum+= pow((double)(v1[i]-v2[i]), 2.0);
        }
        return (ValueType)sqrt(sqSum);
    }
};
FD_NS_END

#endif /* defined(__FastDTW_x__EuclideanDistance__) */
