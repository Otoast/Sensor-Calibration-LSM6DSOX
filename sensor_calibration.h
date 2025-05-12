#ifndef SENSOR_CALIB
#define SENSOR_CALIB

#include "registers.h"
#include "sensor_reading.h"

#include <sstream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

#include <Eigen/Dense>
    

class FooBarCalibration {
private:
    int channel;

    static constexpr int MAX_DATA_SIZE = 40;
    static constexpr int MAX_SAMPLE_SIZE = 200;
    static constexpr double GRAVITY = 1.0;
    static constexpr double GRAVITY_TOL = 0.2;
    static constexpr double ZERO_TOL = 0.2;
    static constexpr double STDEV_THRESHOLD = 0.02;
    
    std::mutex queueMutex;
    std::atomic<bool> stopDataCollection;
    std::queue<LSM6DSOX_Data> recentData;
    
    struct XYZData {double x, y, z;};
    struct CalibratedAxis {std::string axisName; bool isCalibrated; int axisRow, axisCol;};
    
    struct {
        double raw_x, raw_y, raw_z,
        raw_x_sq, raw_y_sq, raw_z_sq;
    } 
    rawData;

    struct {
        LSM6DSOX_Data neg_x, neg_y, neg_z;
        LSM6DSOX_Data pos_x, pos_y, pos_z;
    }
    dataSamples;
    

    inline double _computeStDev(double sum_values, double sum_squares, int num_values) {
        return sqrt((sum_squares - (pow(sum_values, 2) / num_values)) / (num_values - 1));
    }
    inline double _computeRollingMean(double sum_values, int num_values){
        return sum_values / num_values;
    }

    void _addData(LSM6DSOX_Data d, bool addOrSubtract);

    void _collectData();

    // 1 for X | 2 for Y | 3 for Z | 4 for None. Negative sign means negative axis
    int _findValidAxis();

    void _getAxisInfo(int axis, CalibratedAxis* ai, bool isAxisCalibrated[3][2], std::string calibratedAxisName[3][2]);

    bool _collectDataSamples(CalibratedAxis* ai, bool isAxisCalibrated[3][2], std::string calibratedAxisName[3][2]);



public:
    FooBarCalibration(int channel);

    CalibrationData calibrationLoop ();

    XYZData getRollingMean();

    XYZData getRollingStDev();
};


CalibrationData parseStringData(std::string s);

std::vector<CalibrationData> sensorCalibration();


#endif