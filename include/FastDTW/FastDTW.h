//
//  FastDTW.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/9/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__FastDTW__
#define __FastDTW_x__FastDTW__

#include "Foundation.h"
#include "DTW.h"
#include "PAA.h"
#include "ExpandedResWindow.h"

//#include "TimeWarpInfo.h"
//#include "TimeSeries.h"

FD_NS_START

namespace FAST {

    const JInt DEFAULT_SEARCH_RADIUS = 1;
    
    extern const JInt DEFAULT_SEARCH_RADIUS;
    
    template <typename ValueType,JInt nDimension, typename DistanceFunction>
    TimeWarpInfo<ValueType> getWarpInfoBetween(TimeSeries<ValueType,nDimension> const& tsI, TimeSeries<ValueType,nDimension> const& tsJ, JInt searchRadius, DistanceFunction const& distFn)
    {
        if (searchRadius < 0) {
            searchRadius = 0;
        }
        JInt minTSsize = searchRadius + 2;
        if (tsI.size() <= minTSsize || tsJ.size()<=minTSsize) {
            return STRI::getWarpInfoBetween(tsI, tsJ, distFn);
        }
        else
        {
            JDouble resolutionFactor = 2.0;
            PAA<ValueType,nDimension> shrunkI(tsI,(JInt)(tsI.size()/resolutionFactor));
            PAA<ValueType,nDimension> shrunkJ(tsJ,(JInt)(tsJ.size()/resolutionFactor));
            // Determine the search window that constrains the area of the cost matrix that will be evaluated based on
            //    the warp path found at the previous resolution (smaller time series).
            TimeWarpInfo<ValueType> warpInfo = getWarpInfoBetween(shrunkI, shrunkJ, searchRadius, distFn);
            ExpandedResWindow window(tsI, tsJ, shrunkI, shrunkJ,
                                     *(warpInfo.getPath()),
                                     searchRadius);
            return STRI::getWarpInfoBetween(tsI, tsJ, window, distFn);
        }
        
    }
    
    template <typename ValueType,JInt nDimension, typename DistanceFunction>
    inline ValueType getWarpDistBetween(TimeSeries<ValueType,nDimension> const& tsI,TimeSeries<ValueType,nDimension> const& tsJ, DistanceFunction const& distFn)
    {
        return getWarpInfoBetween(tsI, tsJ, DEFAULT_SEARCH_RADIUS, distFn).getDistance();
    }
    
    template <typename ValueType,JInt nDimension, typename DistanceFunction>
    inline TimeWarpInfo<ValueType> getWarpInfoBetween(TimeSeries<ValueType,nDimension> const& tsI,TimeSeries<ValueType,nDimension> const& tsJ,DistanceFunction const& distFn)
    {
        return getWarpInfoBetween(tsI, tsJ, DEFAULT_SEARCH_RADIUS, distFn);
    }
    
    template <typename ValueType,JInt nDimension, typename DistanceFunction>
    inline ValueType getWarpDistBetween(TimeSeries<ValueType,nDimension> const& tsI,TimeSeries<ValueType,nDimension> const& tsJ,
                                        JInt searchRadius,DistanceFunction const& distFn)
    {
        return getWarpInfoBetween(tsI, tsJ, searchRadius, distFn).getDistance();
    }
    
    
    
}
FD_NS_END
#endif /* defined(__FastDTW_x__FastDTW__) */
