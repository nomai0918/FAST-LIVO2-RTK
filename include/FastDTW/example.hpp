#include "DTW.h"
#include "FastDTW.h"
#include "EuclideanDistance.h"
#include <iostream>

using namespace fastdtw;


double *sample1;
double *sample2;



void DTW(std::vector<double> &time_Series_1, std::vector<double> &time_Series_2,std::vector<std::pair<int,int>> &imu_lidar_index)
{
    TimeSeries<double,1> tsI;
    for (int i = 0; i<time_Series_1.size(); ++i) {
        tsI.addLast(i, TimeSeriesPoint<double,1>(sample1+i));
    }

    TimeSeries<double,1> tsJ;
    for (int i = 0;i<time_Series_2.size(); ++i)
    {
        tsJ.addLast(i, TimeSeriesPoint<double,1>(sample2+i));
    }

    TimeWarpInfo<double> info =  STRI::getWarpInfoBetween(tsI,tsJ,EuclideanDistance());
    // printf("Warp Distance by DTW:%lf\n",info.getDistance());
    info.getPath()->print(imu_lidar_index);
}

void DTW(std::vector<double> &time_Series_1, std::vector<double> &time_Series_2, double &radius)
{
    TimeSeries<double,1> tsI;
    for (int i = 0; i<time_Series_1.size(); ++i) {
        tsI.addLast(i, TimeSeriesPoint<double,1>(sample1+i));
    }

    TimeSeries<double,1> tsJ;
    for (int i = 0;i<time_Series_2.size(); ++i)
    {
        tsJ.addLast(i, TimeSeriesPoint<double,1>(sample2+i));
    }

    TimeWarpInfo<double> info =  STRI::getWarpInfoBetween(tsI,tsJ,EuclideanDistance());
    radius = info.getDistance();
    // printf("Warp Distance by DTW:%lf\n",info.getDistance());
    // info.getPath()->print(std::cout);

}

void FastDTW(std::vector<double> &time_Series_1, std::vector<double> &time_Series_2, double &radius)
{
    TimeSeries<double,1> tsI;
    for (int i = 0; i<time_Series_1.size(); ++i) {
        tsI.addLast(i, TimeSeriesPoint<double,1>(sample1+i));
    }

    TimeSeries<double,1> tsJ;
    for (int i = 0;i<time_Series_2.size(); ++i)
    {
        tsJ.addLast(i, TimeSeriesPoint<double,1>(sample2+i));
    }

    TimeWarpInfo<double> info =  FAST::getWarpInfoBetween(tsI,tsJ,EuclideanDistance());
    radius = info.getDistance();
    printf("Warp Distance by FASTDTW:%lf\n",info.getDistance());
    // info.getPath()->print(std::cout);
}

