//
//  TimeSeriesPoint.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/3/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__TimeSeriesPoint__
#define __FastDTW_x__TimeSeriesPoint__
#include "Foundation.h"
#include "FDAssert.h"
#include <iostream>
#include <algorithm>
#include <vector>
FD_NS_START
using namespace std;

//Fixed dimension TimeSeriesPoint template
template <typename ValueType, JInt nDimension>
class MeasurementVector
{
    vector<ValueType> value;
    
public:
    MeasurementVector():value(nDimension)
    {
    }
    
    MeasurementVector(const ValueType* meas):value(nDimension)
    {
        copy(meas, meas+nDimension, value.begin());
    }
    
    void setDynamicMeasurements(const ValueType* meas, JInt nDim)
    {
        FDASSERT0(false, "Invalid method for dynamic TimeSeriesPoint only(set dimension template parameter to 0 to use dynamic TimeSeriesPoint).");
    }
    
    JInt size() const
    {
        return value.size();
    }
    
    ValueType operator[](JInt index) const
    {
        return value[index];
    }
    
    ValueType& operator[](JInt index)
    {
        return value[index];
    }
    
    bool operator==(const MeasurementVector<ValueType,nDimension>& mv) const
    {
        return  value == mv.value;
    }
    
    bool operator<(const MeasurementVector<ValueType, nDimension>& mv) const
    {
        return value < mv.value;
    }
    
    void print(ostream& stream) const
    {
        stream<<"p(";
        for (JInt i = 0; i<value.size(); ++i) {
            stream << value[i] << ",";
        }
        stream <<")";
    }
};

//1 dimension TimeSeriesPoint specification
template <typename ValueType>
class MeasurementVector<ValueType, 1> {
    
    ValueType value;
    
public:
    MeasurementVector():value(0)
    {
    }
    
    MeasurementVector(const ValueType* meas):value(*meas)
    {
    }
    
    void setDynamicMeasurements(const ValueType* meas, JInt nDim)
    {
        FDASSERT0(false, "Invalid method for dynamic TimeSeriesPoint only(set dimension template parameter to 0 to use dynamic TimeSeriesPoint).");
    }
    
    JInt size() const
    {
        return 1;
    }
    
    ValueType operator[](JInt index) const
    {
        return value;
    }

    
    ValueType& operator[](JInt index)
    {
        return value;
    }
    
    bool operator==(const MeasurementVector<ValueType,1>& mv) const
    {
        return  value == mv.value;
    }
    
    bool operator<(const MeasurementVector<ValueType, 1>& mv) const
    {
        return value < mv.value;
    }
    
    void print(ostream& stream) const
    {
        stream<<"p("<<value<<")";
    }
};


//Dynamic dimension TimeSeriesPoint specification
template <typename ValueType>
class MeasurementVector<ValueType, 0> {
    
    vector<ValueType> value;
    
public:
    MeasurementVector():value()
    {
    }
    
    MeasurementVector(const ValueType* meas):value()
    {
    }
    
    void setDynamicMeasurements(const ValueType* meas, JInt nDim)
    {
        value.resize(nDim);
        for (int i = 0; i<nDim; ++i) {
            value[i] = meas[i];
        }
    }
    
    JInt size() const
    {
        return value.size();
    }
    
    ValueType operator[](JInt index) const
    {
        return value[index];
    }
    
    ValueType& operator[](JInt index)
    {
        return value[index];
    }
    
    bool operator==(const MeasurementVector<ValueType,1>& mv) const
    {
        return  value == mv.value;
    }
    
    bool operator<(const MeasurementVector<ValueType, 1>& mv) const
    {
        return value < mv.value;
    }
    
    void print(ostream& stream) const
    {
        stream<<"p(";
        for (JInt i = 0; i<value.size(); ++i) {
            stream << value[i] << ",";
        }
        stream <<")";
    }
};

template <typename ValueType,JInt nDimension>
class TimeSeriesPoint {
    MeasurementVector<ValueType, nDimension> _measurements;
    
public:
    TimeSeriesPoint(const ValueType* meas):_measurements(meas)
    {
    }
    
    ValueType get(JInt dimension) const
    {
        return _measurements[dimension];
    }
    
    void setDynamicMeasurements(const ValueType* meas, JInt nDim)
    {
        _measurements.setDynamicMeasurements(meas,nDim);
    }
    
    void set(JInt dimension,ValueType newValue)
    {
        _measurements[dimension] = newValue;
    }
    
    JInt size() const
    {
        return _measurements.size();
    }
    
    const MeasurementVector<ValueType, nDimension>* toArray() const
    {
        return &_measurements;
    }
    
    bool operator==(const TimeSeriesPoint& p) const
    {
        return _measurements == p._measurements;
    }
    
    bool operator<(TimeSeriesPoint& p)
    {
        return _measurements < p._measurements;
    }
    
    ~TimeSeriesPoint()
    {
    }
    
    void print(ostream& stream) const
    {
        _measurements.print(stream);
    }
};

FD_NS_END
#endif /* defined(__FastDTW_x__TimeSeriesPoint__) */
