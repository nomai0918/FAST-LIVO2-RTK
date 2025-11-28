//
//  Assert.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/2/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef FastDTW_x_Assert_h
#define FastDTW_x_Assert_h
#include <cassert>
#include <stdio.h>

#define FDASSERT0(__assertion__,__msg__) \
do {if(!(__assertion__)){printf(__msg__);assert(false);}}while(0)

#define FDASSERT(__assertion__,__msg__,...) \
do {if(!(__assertion__)){printf(__msg__,__VA_ARGS__);assert(false);}}while(0)

#endif
