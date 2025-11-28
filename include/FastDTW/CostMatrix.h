//
//  CostMatrix.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/4/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__CostMatrix__
#define __FastDTW_x__CostMatrix__

#include "Foundation.h"
FD_NS_START

template <typename ValueType>
class CostMatrix {
    
public:
    virtual void put(JInt col,JInt row,ValueType value) = 0;
    
    virtual ValueType get(JInt col, JInt row) const = 0;
    
    virtual JInt size() const = 0;
};

FD_NS_END
#endif /* defined(__FastDTW_x__CostMatrix__) */
