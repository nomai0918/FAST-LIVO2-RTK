//
//  TimeWarpInfo.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/5/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__TimeWarpInfo__
#define __FastDTW_x__TimeWarpInfo__

#include "Foundation.h"
#include "WarpPath.h"
FD_NS_START
template <typename ValueType>
class TimeWarpInfo
{
    ValueType _distance;
    WarpPath _path;
    
public:
    TimeWarpInfo(ValueType dist, const WarpPath& wp):_distance(dist), _path(wp)
    {
    }
    
    ValueType getDistance() const
    {
        return _distance;
    }
    
    const WarpPath* getPath() const //Unsafe pointer,For reducing the copy cost
    {
        return &_path;
    }
};

FD_NS_END
#endif /* defined(__FastDTW_x__TimeWarpInfo__) */
