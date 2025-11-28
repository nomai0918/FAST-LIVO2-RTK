//
//  DTW.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/6/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__DTW__
#define __FastDTW_x__DTW__

#include "Foundation.h"
#include "WarpPath.h"
#include "TimeSeries.h"
#include "ColMajorCell.h"
#include "FDMath.h"
#include "TimeWarpInfo.h"
#include "SearchWindow.h"
#include "PartialWindowMatrix.h"
#include "MemoryResidentMatrix.h"
#include <vector>
#include <limits>
FD_NS_START

namespace STRI {
    using namespace std;
    
    template <typename ValueType, JInt nDimension, typename DistanceFunction>
    ValueType calcWarpCost(const WarpPath& path,const TimeSeries<ValueType,nDimension>& tsI, const TimeSeries<ValueType,nDimension>& tsJ, const DistanceFunction& distFn)
    {
        ValueType totalCost = 0.0;
        for (JInt p =0; p<path.size(); ++p) {
            ColMajorCell currWarp = path.get(p);
            totalCost += distFn.calcDistance(*tsI.getMeasurementVector(currWarp.getCol()), *tsJ.getMeasurementVector(currWarp.getRow()));
        }
        return totalCost;
    }
    
    // Dynamic Time Warping where the warp path is not needed, an alternate implementation can be used that does not
    //    require the entire cost matrix to be filled and only needs 2 columns to be stored at any one time.
    template <typename ValueType, JInt nDimension, typename DistanceFunction>
    ValueType getWarpDistBetween(TimeSeries<ValueType, nDimension> const& tsI, TimeSeries<ValueType,nDimension> const& tsJ, DistanceFunction const& distFn)
    {
        // The space complexity is 2*tsJ.size().  Dynamic time warping is symmetric so switching the two time series
        //    parameters does not effect the final warp cost but can reduce the space complexity by allowing tsJ to
        //    be set as the shorter time series and only requiring 2 columns of size |tsJ| rather than 2 larger columns of
        //    size |tsI|.
        if (tsI.size() < tsJ.size()) {
            return getWarpDistBetween(tsJ, tsI, distFn);
        }
        vector<ValueType> lastColumn(tsJ.size());
        vector<ValueType> currColumn(tsJ.size());
        JInt maxI = tsI.size() - 1;
        JInt maxJ = tsJ.size() - 1;
        // Calculate the values for the first column, from the bottom up.
        currColumn[0] = distFn.calcDistance(*tsI.getMeasurementVector(0), *tsJ.getMeasurementVector(0));
        for (JInt j = 1; j<maxJ; ++j) {
            currColumn[j] = currColumn[j-1] + distFn.calcDistance(*tsI.getMeasurementVector(0), *tsJ.getMeasurementVector(j));
        }
        vector<ValueType>* lastCol = &lastColumn;
        vector<ValueType>* currCol = &currColumn;
        for (JInt i = 1; i<maxI; ++i) {
            // Swap the references between the two arrays.
            vector<ValueType>* temp = lastCol;
            lastCol = currCol;
            currCol = temp;
            // Calculate the value for the bottom row of the current column
            //    (i,0) = LocalCost(i,0) + GlobalCost(i-1,0)
            (*currCol)[0] = (*lastCol)[0] + distFn.calcDistance(*tsI.getMeasurementVector(i), *tsJ.getMeasurementVector(0));
            
            for (int j=1; j<=maxJ; j++)  // j = rows
            {
                // (i,j) = LocalCost(i,j) + minGlobalCost{(i-1,j),(i-1,j-1),(i,j-1)}
                ValueType minGlobalCost = fd_min(lastCol->at(j), fd_min(lastCol->at(j-1), currCol->at(j-1)));
                (*currCol)[j] = minGlobalCost + distFn.calcDistance(*tsI.getMeasurementVector(i), *tsJ.getMeasurementVector(j));
            }  // end for loop
        }
        return currCol->at(maxJ);
    }
    
    template <typename  ValueType, JInt nDimension, typename DistanceFunction>
    const TimeWarpInfo<ValueType> getWarpInfoBetween(TimeSeries<ValueType, nDimension> const& tsI, TimeSeries<ValueType, nDimension> const& tsJ, DistanceFunction const& distFn)
    {
        //     COST MATRIX:
        //   5|_|_|_|_|_|_|E| E = min Global Cost
        //   4|_|_|_|_|_|_|_| S = Start point
        //   3|_|_|_|_|_|_|_| each cell = min global cost to get to that point
        // j 2|_|_|_|_|_|_|_|
        //   1|_|_|_|_|_|_|_|
        //   0|S|_|_|_|_|_|_|
        //     0 1 2 3 4 5 6
        //            i
        //   access is M(i,j)... column-row
        vector<vector<ValueType> > costMatrix;
        costMatrix.reserve(tsI.size());
        JInt jSize = tsJ.size();
        for (JInt i = 0; i<tsI.size(); ++i) {
            costMatrix.push_back(vector<ValueType>(jSize));
        }
        JInt maxI = tsI.size() - 1;
        JInt maxJ = tsJ.size() - 1;
        costMatrix[0][0] = distFn.calcDistance(*tsI.getMeasurementVector(0),
                                               *tsJ.getMeasurementVector(0));
        for (int j=1; j<=maxJ; j++)
            costMatrix[0][j] = costMatrix[0][j-1] + distFn.calcDistance(*tsI.getMeasurementVector(0),
                                                                        *tsJ.getMeasurementVector(j));
        for (int i=1; i<=maxI; i++)   // i = columns
        {
            // Calculate the value for the bottom row of the current column
            //    (i,0) = LocalCost(i,0) + GlobalCost(i-1,0)
            costMatrix[i][0] = costMatrix[i-1][0] + distFn.calcDistance(*tsI.getMeasurementVector(i),
                                                                        *tsJ.getMeasurementVector(0));
            
            for (int j=1; j<=maxJ; j++)  // j = rows
            {
                // (i,j) = LocalCost(i,j) + minGlobalCost{(i-1,j),(i-1,j-1),(i,j-1)}
                ValueType minGlobalCost = fd_min(costMatrix[i-1][j],
                                              fd_min(costMatrix[i-1][j-1],
                                                  costMatrix[i][j-1]));
                costMatrix[i][j] = minGlobalCost + distFn.calcDistance(*tsI.getMeasurementVector(i),
                                                                       *tsJ.getMeasurementVector(j));
            }
        }
        ValueType minimumCost = costMatrix[maxI][maxJ];
        WarpPath minCostPath(maxI + maxJ - 1);
        JInt i = maxI;
        JInt j = maxJ;
        minCostPath.addFirst(i,j);
        while (i>0 || j>0) {
            ValueType diagCost;
            ValueType leftCost;
            ValueType downCost;
            if ((i>0) && (j>0))
            {
                diagCost = costMatrix[i-1][j-1];
            }
            else
            {
                diagCost = numeric_limits<ValueType>::max();
            }
            if (i > 0)
            {
                leftCost = costMatrix[i-1][j];
            }
            else
            {
                leftCost = numeric_limits<ValueType>::max();
            }
            if (j > 0)
            {
                downCost = costMatrix[i][j-1];
            }
            else
            {
                downCost = numeric_limits<ValueType>::max();
            }
            
            // Determine which direction to move in.  Prefer moving diagonally and
            //    moving towards the i==j axis of the matrix if there are ties.
            if ((diagCost<=leftCost) && (diagCost<=downCost))
            {
                i--;
                j--;
            }
            else if ((leftCost<diagCost) && (leftCost<downCost))
            {
                i--;
            }
            else if ((downCost<diagCost) && (downCost<leftCost))
            {
                j--;
            }
            else if (i <= j)  // leftCost==rightCost > diagCost
            {
                j--;
            }
            else   // leftCost==rightCost > diagCost
            {
                i--;
            }
            minCostPath.addFirst(i, j);
        }
        //Let Return Value Optimization do its job.
        return TimeWarpInfo<ValueType>(minimumCost, minCostPath);
    }
    
    template <typename  ValueType, JInt nDimension, typename DistanceFunction>
    ValueType getWarpDistBetween(TimeSeries<ValueType,nDimension> const& tsI,TimeSeries<ValueType,nDimension> const& tsJ,SearchWindow const& window, DistanceFunction const& distFn)
    {
        //     COST MATRIX:
        //   5|_|_|_|_|_|_|E| E = min Global Cost
        //   4|_|_|_|_|_|_|_| S = Start point
        //   3|_|_|_|_|_|_|_| each cell = min global cost to get to that point
        // j 2|_|_|_|_|_|_|_|
        //   1|_|_|_|_|_|_|_|
        //   0|S|_|_|_|_|_|_|
        //     0 1 2 3 4 5 6
        //            i
        //   access is M(i,j)... column-row
        PartialWindowMatrix<ValueType> costMatrix(window);
        JInt maxI = tsI.size()-1;
        JInt maxJ = tsI.size() -1;
        // Get an iterator that traverses the window cells in the order that the cost matrix is filled.
        //    (first to last row (1..maxI), bottom to top (1..MaxJ)
        SearchWindowIterator matrixIterator = window.iterator();
        while (matrixIterator.hasNext()) {
            ColMajorCell currentCell = matrixIterator.next();
            JInt i = currentCell.getCol();
            JInt j = currentCell.getRow();
            if (i == 0 && j==0) { // bottom left cell (first row AND first column)
                costMatrix.put(i,j,distFn.calcDistance(*tsI.getMeasurementVector(0),*tsJ.getMeasurementVector(0)));
            }
            else if (i == 0)             // first column
            {
                costMatrix.put(i, j, distFn.calcDistance(*tsI.getMeasurementVector(0), *tsJ.getMeasurementVector(j)) +
                               costMatrix.get(i, j-1));
            }
            else if (j == 0)             // first row
            {
                costMatrix.put(i, j, distFn.calcDistance(*tsI.getMeasurementVector(i), *tsJ.getMeasurementVector(0)) +
                               costMatrix.get(i-1, j));
            }
            else                         // not first column or first row
            {
                ValueType minGlobalCost = fd_min(costMatrix.get(i-1, j),
                                              fd_min(costMatrix.get(i-1, j-1),
                                                  costMatrix.get(i, j-1)));
                costMatrix.put(i, j, minGlobalCost + distFn.calcDistance(*tsI.getMeasurementVector(i),
                                                                         *tsJ.getMeasurementVector(j)));
            }
        }
        return costMatrix.get(maxI,maxJ);
    }
    
    template <typename ValueType, JInt nDimension, typename DistanceFunction>
    TimeWarpInfo<ValueType> getWarpInfoBetween(TimeSeries<ValueType,nDimension> const& tsI, TimeSeries<ValueType,nDimension> const& tsJ,SearchWindow const& window, DistanceFunction const& distFn)
    {
        //     COST MATRIX:
        //   5|_|_|_|_|_|_|E| E = min Global Cost
        //   4|_|_|_|_|_|_|_| S = Start point
        //   3|_|_|_|_|_|_|_| each cell = min global cost to get to that point
        // j 2|_|_|_|_|_|_|_|
        //   1|_|_|_|_|_|_|_|
        //   0|S|_|_|_|_|_|_|
        //     0 1 2 3 4 5 6
        //            i
        //   access is M(i,j)... column-row
        MemoryResidentMatrix<ValueType> costMatrix(&window);
        JInt maxI = tsI.size()-1;
        JInt maxJ = tsJ.size()-1;
        
        // Get an iterator that traverses the window cells in the order that the cost matrix is filled.
        //    (first to last row (1..maxI), bottom to top (1..MaxJ)
        SearchWindowIterator matrixIterator = window.iterator();
        
        while (matrixIterator.hasNext())
        {
            ColMajorCell currentCell = matrixIterator.next();  // current cell being filled
            JInt i = currentCell.getCol();
            JInt j = currentCell.getRow();
            
            if ( (i==0) && (j==0) )      // bottom left cell (first row AND first column)
                costMatrix.put(i, j, distFn.calcDistance(*tsI.getMeasurementVector(0), *tsJ.getMeasurementVector(0)));
            else if (i == 0)             // first column
            {
                costMatrix.put(i, j, distFn.calcDistance(*tsI.getMeasurementVector(0), *tsJ.getMeasurementVector(j)) +
                               costMatrix.get(i, j-1));
            }
            else if (j == 0)             // first row
            {
                costMatrix.put(i, j, distFn.calcDistance(*tsI.getMeasurementVector(i), *tsJ.getMeasurementVector(0)) +
                               costMatrix.get(i-1, j));
            }
            else                         // not first column or first row
            {
                ValueType minGlobalCost = fd_min(costMatrix.get(i-1, j),
                                              fd_min(costMatrix.get(i-1, j-1),
                                                  costMatrix.get(i, j-1)));
                costMatrix.put(i, j, minGlobalCost + distFn.calcDistance(*tsI.getMeasurementVector(i),
                                                                         *tsJ.getMeasurementVector(j)));
            }
        }
        
        // Minimum Cost is at (maxI, maxJ)
        ValueType minimumCost = costMatrix.get(maxI, maxJ);
        
        WarpPath minCostPath(maxI+maxJ-1);
        JInt i = maxI;
        JInt j = maxJ;
        minCostPath.addFirst(i, j);
        while ((i>0) || (j>0))
        {
            // Find the costs of moving in all three possible directions (left,
            //    down, and diagonal (down and left at the same time).
            ValueType diagCost;
            ValueType leftCost;
            ValueType downCost;
            
            if ((i>0) && (j>0))
                diagCost = costMatrix.get(i-1, j-1);
            else
                diagCost = numeric_limits<ValueType>::max();
            
            if (i > 0)
                leftCost = costMatrix.get(i-1, j);
            else
                leftCost = numeric_limits<ValueType>::max();
            
            if (j > 0)
                downCost = costMatrix.get(i, j-1);
            else
                downCost = numeric_limits<ValueType>::max();
            
            // Determine which direction to move in.  Prefer moving diagonally and
            //    moving towards the i==j axis of the matrix if there are ties.
            if ((diagCost<=leftCost) && (diagCost<=downCost))
            {
                i--;
                j--;
            }
            else if ((leftCost<diagCost) && (leftCost<downCost))
                i--;
            else if ((downCost<diagCost) && (downCost<leftCost))
                j--;
            else if (i <= j)  // leftCost==rightCost > diagCost
                j--;
            else   // leftCost==rightCost > diagCost
                i--;
            
            // Add the current step to the warp path.
            minCostPath.addFirst(i, j);
        }
        
        return TimeWarpInfo<ValueType>(minimumCost, minCostPath);
    }
}

FD_NS_END
#endif /* defined(__FastDTW_x__DTW__) */
