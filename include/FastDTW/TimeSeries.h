//
//  TimeSeries.h
//  FastDTW-x
//
//  Created by Melo Yao on 12/3/13.
//  Copyright (c) 2013 melo.yao. All rights reserved.
//

#ifndef __FastDTW_x__TimeSeries__
#define __FastDTW_x__TimeSeries__

#include "Foundation.h"
#include <vector>
#include <string>
#include "TimeSeriesPoint.h"
#include "FDAssert.h"
#include <iostream>
#include <cstdio>
#include <sstream>

FD_NS_START
using namespace std;
template <typename ValueType, JInt nDimension>
class TimeSeries
{
private:
    static const JInt ZERO_ARRAY[0];
    static const JBool DEFAULT_IS_TIME_1ST_COL = true;
    static const JChar DEFAULT_DELIMITER = ',';
    static const JBool DEFAULT_IS_LABELED = true;
   
   
protected:
    vector<string> _labels;
    vector<JDouble> _timeReadings;
    vector<TimeSeriesPoint<ValueType,nDimension> > _tsArray;
    
    void setMaxCapacity(JInt capacity)
    {
        _timeReadings.reserve(capacity);
        _tsArray.reserve(capacity);
    }
public:

    
    TimeSeries():_labels(),_timeReadings(),_tsArray()
    {
        _labels.push_back(string("time"));
        stringstream ss;

        for (JInt i = 0; i<nDimension; ++i) {
			ss.clear();
            ss << i;
			_labels.push_back(ss.str());
        }
    }
    
    TimeSeries(TimeSeries& timeseries):_labels(timeseries._labels),_timeReadings(timeseries._timeReadings),_tsArray(timeseries._tsArray)
    {
        
    }
        
    //ignored file io interfaces
    
    void clear()
    {
        _timeReadings.clear();
        _tsArray.clear();
    }
    
    JInt size() const
    {
        return _timeReadings.size();
    }
    
    JInt numOfPts() const
    {
        return this->size();
    }
    
    JInt numOfDimensions() const
    {
        return _labels.size() - 1;//potential bug when use default constructor
    }
    
    JDouble getTimeAtNthPoint(JInt n) const
    {
        return _timeReadings[n];
    }
    
    string getLabel(JInt n) const
    {
        return _labels[n];
    }

//    JInt getLabelsArr(string* strArr,JInt maxNum)
//    {
//        JInt len = maxNum>_labels.size()?_labels.size():maxNum;
//        memcpy(strArr, _labels.data(), len*sizeof(string));
//        return len;
//    }
    
    void setLabels(const vector<string>& lbs)
    {
        _labels = lbs;
    }
    
    void setLabels(const string* strArr,JInt num)
    {
        _labels.clear();
        _labels.reserve(num);
        copy(strArr, strArr+num, _labels.begin());
    }
    
    const vector<string>* getLabels() const
    {
        return &_labels;
    }
    
    ValueType getMeasurement(JInt pointIndex, string& valueLabel) const
    {
        JInt idx = find(_labels.begin(), _labels.end(), valueLabel) - _labels.begin();
        FDASSERT(idx>0, "ERROR:  the label %s was not one of labels",valueLabel.data());
        return _tsArray[pointIndex].get(idx - 1);
    }
    
    ValueType getMeasurement(JInt pointIndex, JInt valueIndex) const
    {
        return _tsArray[pointIndex].get(valueIndex);
    }
    
    const MeasurementVector<ValueType, nDimension>* getMeasurementVector(JInt pointIndex) const
    {
        return _tsArray[pointIndex].toArray();
    }
    
    void setMeasurement(JInt pointIndex,JInt valueIndex,ValueType value)
    {
        _tsArray[pointIndex].set(valueIndex,value);
    }
    
    void addFirst(JDouble time, TimeSeriesPoint<ValueType,nDimension> const& values)
    {
        FDASSERT(values.size()+1 == _labels.size(), "ERROR:  The TimeSeriesPoint contains the wrong number of values. expected:%ld,found:%ld",_labels.size()-1,values.size());
        FDASSERT0(time<_timeReadings[0], "ERROR:  The point being inserted into the beginning of the time series does not have the correct time sequence.");
        _timeReadings.insert(_timeReadings.begin(), time);
        _tsArray.insert(_tsArray.begin(),values);
    }
    
    void addLast(JDouble time, TimeSeriesPoint<ValueType,nDimension> const& values)
    {
        FDASSERT(values.size()+1 == _labels.size(), "ERROR:  The TimeSeriesPoint contains the wrong number of values. expected:%ld,found:%ld",_labels.size()-1,values.size());
        FDASSERT0(_timeReadings.size()==0 || time>_timeReadings[_timeReadings.size() - 1], "ERROR:  The point being inserted into the beginning of the time series does not have the correct time sequence.");
        _timeReadings.push_back(time);
        _tsArray.push_back(values);
    }
    
    virtual void print(ostream& stream) const
    {
        stream<<"time readings ["<<_timeReadings.size()<<"]:";
        for (JInt i = 0; i<_timeReadings.size(); ++i) {
            stream<<_timeReadings[i] << ",";
        }
        stream<<"\n";
        stream<<"time series ["<<_tsArray.size()<<"]:";
        for(JInt i = 0;i<_tsArray.size();++i)
        {
            _tsArray[i].print(stream);
            stream << ",";
        }
        stream<<"\n";
    }
};

FD_NS_END
#endif /* defined(__FastDTW_x__TimeSeries__) */
