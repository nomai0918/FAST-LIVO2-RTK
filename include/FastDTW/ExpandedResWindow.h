//
//  ExpandedResWindow.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/9/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__ExpandedResWindow__
#define __FastDTW_x__ExpandedResWindow__

#include "Foundation.h"
#include "TimeSeries.h"
#include "SearchWindow.h"
#include "PAA.h"
#include "WarpPath.h"
#include <limits>

FD_NS_START
class ExpandedResWindow : public SearchWindow {
    
    
public:
    template <typename ValueType,JInt nDimension>
    ExpandedResWindow(TimeSeries<ValueType,nDimension> const& tsI,TimeSeries<ValueType,nDimension> const& tsJ,
                      PAA<ValueType,nDimension> const& shrunkI,PAA<ValueType,nDimension> const& shrunkJ,WarpPath const& shrunkWarpPath, JInt searchRadius):SearchWindow(tsI.size(),tsJ.size())
    {
        // Variables to keep track of the current location of the higher resolution projected path.
        JInt currentI = shrunkWarpPath.minI();
        JInt currentJ = shrunkWarpPath.minJ();
        
        // Variables to keep track of the last part of the low-resolution warp path that was evaluated
        //    (to determine direction).
        JInt lastWarpedI = numeric_limits<JInt>::max();
        JInt lastWarpedJ = numeric_limits<JInt>::max();
        
        // For each part of the low-resolution warp path, project that path to the higher resolution by filling in the
        //    path's corresponding cells at the higher resolution.
        for (JInt w=0; w<shrunkWarpPath.size(); w++)
        {
            ColMajorCell currentCell = shrunkWarpPath.get(w);
            JInt warpedI = currentCell.getCol();
            JInt warpedJ = currentCell.getRow();
            
            JInt blockISize = shrunkI.aggregatePtSize(warpedI);
            JInt blockJSize = shrunkJ.aggregatePtSize(warpedJ);
            
            // If the path moved up or diagonally, then the next cell's values on the J axis will be larger.
            if (warpedJ > lastWarpedJ)
                currentJ += shrunkJ.aggregatePtSize(lastWarpedJ);
            
            // If the path moved up or diagonally, then the next cell's values on the J axis will be larger.
            if (warpedI > lastWarpedI)
                currentI += shrunkI.aggregatePtSize(lastWarpedI);
            
            // If a diagonal move was performed, add 2 cells to the edges of the 2 blocks in the projected path to create
            //    a continuous path (path with even width...avoid a path of boxes connected only at their corners).
            //                        |_|_|x|x|     then mark      |_|_|x|x|
            //    ex: projected path: |_|_|x|x|  --2 more cells->  |_|X|x|x|
            //                        |x|x|_|_|        (X's)       |x|x|X|_|
            //                        |x|x|_|_|                    |x|x|_|_|
            if ((warpedJ>lastWarpedJ) && (warpedI>lastWarpedI))
            {
                SearchWindow::markVisited(currentI-1, currentJ);
                SearchWindow::markVisited(currentI, currentJ-1);
            }  // end if
            
            // Fill in the cells that are created by a projection from the cell in the low-resolution warp path to a
            //    higher resolution.
            for (int x=0; x<blockISize; x++)
            {
                SearchWindow::markVisited(currentI+x, currentJ);
                SearchWindow::markVisited(currentI+x, currentJ+blockJSize-1);
            }  // end for loop
            
            // Record the last position in the warp path so the direction of the path can be determined when the next
            //    position of the path is evaluated.
            lastWarpedI = warpedI;
            lastWarpedJ = warpedJ;
        }  // end for loop
        
        
        // Expand the size of the projected warp path by the specified width.
        SearchWindow::expandWindow(searchRadius);
    }
};

FD_NS_END

#endif /* defined(__FastDTW_x__ExpandedResWindow__) */
