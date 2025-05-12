#include "registers.h"
#include "sensor_reading.h"
#include "sensor_calibration.h"


void FooBarCalibration::_addData(LSM6DSOX_Data d, bool addOrSubtract) {
    int m = (!addOrSubtract) ? -1 : 1;

    rawData.raw_x += m * d.AccelData.x;
    rawData.raw_y += m * d.AccelData.y;
    rawData.raw_z += m * d.AccelData.z;
    rawData.raw_x_sq += m * pow(d.AccelData.x, 2);
    rawData.raw_y_sq += m * pow(d.AccelData.y, 2);
    rawData.raw_z_sq += m * pow(d.AccelData.z, 2);
}

void FooBarCalibration::_collectData() {
    communicateWithLSM6DOX(channel);
    SensorStatusBits ssb = get_LSM6DSOX_status();
    LSM6DSOX_Data newData;

    if (!ssb.accelAvail || !ssb.gyroAvail) return;
    
    queueMutex.lock();
    LSM6DSOX_read_xyz(newData);
    recentData.push(newData);
    if (recentData.size() > MAX_DATA_SIZE) {
        _addData(recentData.front(), false);
        recentData.pop();
    }
    _addData(newData, true);
    queueMutex.unlock();
}

// 1 for X | 2 for Y | 3 for Z | 4 for None. Negative sign means negative axis
int FooBarCalibration::_findValidAxis() {
    XYZData rollingMean = getRollingMean();
    XYZData rollingStDev = getRollingStDev();
    double mean_data[] = {rollingMean.x, rollingMean.y, rollingMean.z};
    double std_data[] = {rollingStDev.x, rollingStDev.y, rollingStDev.z};

    for (int i = 0; i < 3; ++i) {
        bool gravity_axis = abs(abs(mean_data[i]) - GRAVITY) < GRAVITY_TOL && std_data[i] < STDEV_THRESHOLD;
    
        bool others_zero = true;
        for (int j = 0; j < 3; ++j) {
            if (j == i) continue;
            others_zero = (abs(mean_data[j]) < ZERO_TOL || std_data[j] < STDEV_THRESHOLD);
            if (!others_zero) break;
        }
        
        if (gravity_axis && others_zero) 
            return  (mean_data[i] > 0) ? (i + 1) :-(i + 1);  // Return the index of the axis aligned with gravity
    }    
    return 4;
}


void FooBarCalibration::_getAxisInfo(int axis, CalibratedAxis* ai, bool isAxisCalibrated[3][2], std::string calibratedAxisName[3][2]) {
    if (axis == 4) {
        ai->isCalibrated = true;
        ai-> axisName = "None";
        ai->axisRow = -1;
        ai->axisCol = -1;
        return;
    }
    
    int index  = abs(axis) - 1;
    const bool* ax = isAxisCalibrated[index];
    const std::string* ax_str = calibratedAxisName[index];
    int i = (axis < 0) ? 0 : 1;
    ai->isCalibrated = ax[i];
    ai->axisName = ax_str[i];
    ai->axisRow = index;
    ai->axisCol = i;
}

bool FooBarCalibration::_collectDataSamples(CalibratedAxis* ai, bool isAxisCalibrated[3][2], std::string calibratedAxisName[3][2]) {
    usleep(100000); // Wait a bit to make sure person is steady
    std::string currentAxis = ai->axisName;
    LSM6DSOX_Data summedAxisData;

    LSM6DSOX_Data* dataSamplesRefrences[3][2] = {
        &dataSamples.neg_x, &dataSamples.pos_x, &dataSamples.neg_y,
        &dataSamples.pos_y, &dataSamples.neg_z, &dataSamples.pos_z
    };

    int sample_count = 0;
    while (currentAxis == ai->axisName && sample_count < MAX_SAMPLE_SIZE) {
        queueMutex.lock();
        summedAxisData += recentData.back();
        queueMutex.unlock();
        usleep(rand() % 20000);
        _getAxisInfo(_findValidAxis(), ai, isAxisCalibrated, calibratedAxisName);
        sample_count += 1;
    }
    
    if (sample_count < MAX_SAMPLE_SIZE && currentAxis != ai->axisName)
        return false;
    
    summedAxisData /= MAX_SAMPLE_SIZE;

    ai->isCalibrated = true;
    isAxisCalibrated[ai->axisRow][ai->axisCol] = true;
    *dataSamplesRefrences[ai->axisRow][ai->axisCol] = summedAxisData;
    return true;
}

FooBarCalibration::FooBarCalibration(int channel) : 
    channel(channel),
    queueMutex(), stopDataCollection(false), recentData(),
    rawData({}), dataSamples({}) {
    
    auto data_collection_behavior = [&] () {
        _collectData(); // To prevent division by zero errors
        while (!stopDataCollection) _collectData();
        stopDataCollection = false;
        return;
    };
    std::thread collection_thread (data_collection_behavior);
    collection_thread.detach();
}


CalibrationData  FooBarCalibration::calibrationLoop() {
    std::cout << "Please calibrate the (+-) XYZ axis:" << std::endl;
    std::cout << "PRO TIP: When calbirating the +Z Axis, ideally have the sensor laid flat on a surface. This is also used to calibrate the gyro." << std::endl;
    // 3 axis, 2 directions +-
    int amt_calibrated = 0;
    std::string calibratedAxisName[3][2] = {{"-X Axis", "+X Axis"}, {"-Y Axis", "+Y Axis"}, {"-Z Axis", "+Z Axis"}};
    bool isAxisCalibrated[3][2] = {false};
    struct CalibratedAxis axisInfo = {"None", true, 0, 0};

    while (amt_calibrated != 6) {
        while (axisInfo.isCalibrated || axisInfo.axisName == std::string("None")) {
            usleep(100000);
            _getAxisInfo(_findValidAxis(), &axisInfo, isAxisCalibrated, calibratedAxisName);
        }            
        std::cout << axisInfo.axisName << " detected... Hold the accelerometer steady at that axis." << std::endl;
        std::string current_axis = axisInfo.axisName;
        bool sucessful = _collectDataSamples(&axisInfo, isAxisCalibrated, calibratedAxisName);
        
        if (sucessful) std::cout << current_axis << " data samples sucessfully collected. Try a different axis! (" << ++amt_calibrated << " / 6 Done)" << std::endl;
        else std::cout << "\rFailed to get samples for the axis: " << current_axis << ". Did you move the axis? Please try again.\n" << std::flush;            

        if (sucessful && (current_axis == std::string("+Z Axis"))) std::cout << "Gyro calibrated. Nothing needs to be done further for the gyro." << std::endl;

    }

    std::cout << "All 6 data axis collected. Calculating offset values..." << std::endl;
    stopDataCollection = true;

    CalibrationData cd;
    cd.channelNum = channel;

    cd.offsetsMatrix(0, 0) = (1 / 6.0) * (
        dataSamples.pos_x.AccelData.x + dataSamples.neg_x.AccelData.x + dataSamples.pos_y .AccelData.x +
        dataSamples.neg_y.AccelData.x + dataSamples.pos_z.AccelData.x + dataSamples.neg_z.AccelData.x
    );
    cd.offsetsMatrix(1, 0) = (1 / 6.0) * (
        dataSamples.pos_x.AccelData.y + dataSamples.neg_x.AccelData.y + dataSamples.pos_y.AccelData.y +
        dataSamples.neg_y.AccelData.y + dataSamples.pos_z.AccelData.y + dataSamples.neg_z.AccelData.y
    );
    cd.offsetsMatrix(2, 0) = (1 / 6.0) * (
        dataSamples.pos_x.AccelData.z + dataSamples.neg_x.AccelData.z + dataSamples.pos_y .AccelData.z +
        dataSamples.neg_y.AccelData.z + dataSamples.pos_z.AccelData.z + dataSamples.neg_z.AccelData.z
    );

    cd.omegaOffsetsMatrix << dataSamples.pos_z.GyroData.omega_x, dataSamples.pos_z.GyroData.omega_y, dataSamples.pos_z.GyroData.omega_z;
    

    cd.calibrationMatrix(0, 0) = ((dataSamples.pos_x.AccelData.x - cd.offsetsMatrix(0, 0)) + (-dataSamples.neg_x.AccelData.x + cd.offsetsMatrix(0, 0))) / 2;
    cd.calibrationMatrix(0, 1) = ((dataSamples.pos_y.AccelData.x - cd.offsetsMatrix(0, 0)) + (-dataSamples.neg_y.AccelData.x + cd.offsetsMatrix(0, 0))) / 2;
    cd.calibrationMatrix(0, 2) = ((dataSamples.pos_z.AccelData.x - cd.offsetsMatrix(0, 0)) + (-dataSamples.neg_z.AccelData.x + cd.offsetsMatrix(0, 0))) / 2;
    cd.calibrationMatrix(1, 0) = ((dataSamples.pos_x.AccelData.y - cd.offsetsMatrix(1, 0)) + (-dataSamples.neg_x.AccelData.y + cd.offsetsMatrix(1, 0))) / 2;
    cd.calibrationMatrix(1, 1) = ((dataSamples.pos_y.AccelData.y - cd.offsetsMatrix(1, 0)) + (-dataSamples.neg_y.AccelData.y + cd.offsetsMatrix(1, 0))) / 2;
    cd.calibrationMatrix(1, 2) = ((dataSamples.pos_z.AccelData.y - cd.offsetsMatrix(1, 0)) + (-dataSamples.neg_z.AccelData.y + cd.offsetsMatrix(1, 0))) / 2;
    cd.calibrationMatrix(2, 0) = ((dataSamples.pos_x.AccelData.z - cd.offsetsMatrix(2, 0)) + (-dataSamples.neg_x.AccelData.z + cd.offsetsMatrix(2, 0))) / 2;
    cd.calibrationMatrix(2, 1) = ((dataSamples.pos_y.AccelData.z - cd.offsetsMatrix(2, 0)) + (-dataSamples.neg_y.AccelData.z + cd.offsetsMatrix(2, 0))) / 2;
    cd.calibrationMatrix(2, 2) = ((dataSamples.pos_z.AccelData.z - cd.offsetsMatrix(2, 0)) + (-dataSamples.neg_z.AccelData.z + cd.offsetsMatrix(2, 0))) / 2;
    
    
    cd.calibrationMatrix = cd.calibrationMatrix.inverse().eval();

    while (stopDataCollection) usleep(100000); // Wait for the thread to signal its finished

    return cd;
}


FooBarCalibration::XYZData  FooBarCalibration::getRollingMean() {
    return XYZData {
        _computeRollingMean(rawData.raw_x, recentData.size()),
        _computeRollingMean(rawData.raw_y, recentData.size()),
        _computeRollingMean(rawData.raw_z, recentData.size()),
    };
}

FooBarCalibration::XYZData  FooBarCalibration::getRollingStDev() {
    return XYZData {
        _computeStDev(rawData.raw_x, rawData.raw_x_sq, recentData.size()),
        _computeStDev(rawData.raw_y, rawData.raw_y_sq, recentData.size()),
        _computeStDev(rawData.raw_z, rawData.raw_z_sq, recentData.size()),
    };
} 



CalibrationData parseStringData(std::string s) {
    std::vector<double> dataBuf;
    std::stringstream dataStream(s);
    std::string buf;
    
    while (getline(dataStream, buf, '\t')) {
        if (dataBuf.size() > 16) {
            std::cerr << "Overflow in collecting calibration data. Too many values per line?" << std::endl;
            exit(1);
        }
        dataBuf.push_back(stof(buf));
    }
    if (dataBuf.size() < 16) {
        std::cerr << "Not enough values for assumed sensor channel: " << (int)dataBuf.at(0) << ". Please fix this entry or remove it." << std::endl;
        exit(1);
    }
    CalibrationData cd;

    cd.channelNum = (int)dataBuf.at(0);

    cd.offsetsMatrix << dataBuf.at(1), dataBuf.at(2), dataBuf.at(3);

    cd.omegaOffsetsMatrix << dataBuf.at(4), dataBuf.at(5), dataBuf.at(6);
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cd.calibrationMatrix(i, j) = dataBuf[7 + (i * 3 + j)];
        }
    }
    
    return cd;
    
}



std::vector<CalibrationData> sensorCalibration() {
    // Assumes there is only 1 I2C and MAX_I2C_CHANNELS channels on the board. Change if not true
    int sensorChannels[MAX_I2C_CHANNELS] = {};
    int numSensors = findSensorChannels(sensorChannels, MAX_I2C_CHANNELS);
    for (int i = 0; i < numSensors; ++i) {
        int channel = sensorChannels[i];
        communicateWithLSM6DOX(channel);
        _setUpSensor();
    }
    std::cout << "Sensors found and set up for calibration." << std::endl;

    std::fstream calibrationFile ("LSM6DSOX_CALIBRATION", std::fstream::in | std::fstream::out | std::fstream::app);
    
    if (!calibrationFile.is_open()) {
        std::cerr << "Issue opening or creating the calibration file!" << std::endl;
        exit(1);
    }

    std::cout << "To recalibrate any preset data, please remove the channel data from the LSM6DSOX_CALIBRATION file. Make sure to end file with a newline." << std::endl;
    
    std::string sBuffer;
    std::vector<CalibrationData> allData;

    while (std::getline(calibrationFile, sBuffer)) {
        CalibrationData cd = parseStringData(sBuffer);
        for (int i = 0; i < numSensors; ++i) {
            if (sensorChannels[i] == cd.channelNum) {
                sensorChannels[i] = -1;
                std::cout << "Found preset calibration for channel: " << std::to_string(cd.channelNum) << ". Will not recalibrate." << std::endl;
            }
        }        
        allData.push_back(cd);
    }
    
    calibrationFile.clear(); // Reset bits to write to it again
    
    for (int i = 0; i < numSensors; ++i) {
        if (sensorChannels[i] > -1) {
            std::cout << "LSM6DSOX at channel: " << std::to_string(sensorChannels[i]) << " not calibrated. Beginning process now." << std::endl;

            FooBarCalibration calibrationHelper (sensorChannels[i]);

            {
                char temp;
                std::cout << "Press any key + Enter to continue:\t" << std::flush;
                std::cin >> temp;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            CalibrationData cd = calibrationHelper.calibrationLoop();
            std::cout << "Calibrated Sensor: " << sensorChannels[i] << ". Adding to calibration file." << std::endl;
            std::cout << "Data collected: " << cd.exportData() << std::endl;
            calibrationFile << cd.exportData() << std::endl;
            allData.push_back(cd); 
        }
    }
    return allData;
}

int sensorCalibration_main() {
    if (!bcm2835_init() | !bcm2835_i2c_begin()) {
        std::cout << "Issue starting bcim and/or i2c." << std::endl;
        exit(1);
    }
    bcm2835_i2c_set_baudrate(400000);
    sensorCalibration(); 

    std::fstream calibrationFile ("LSM6DSOX_CALIBRATION");
    std::string sBuffer;
    std::vector<CalibrationData> allData;

    CalibrationData cd;
    while (std::getline(calibrationFile, sBuffer)) {
        cd = parseStringData(sBuffer);
        if (cd.channelNum == 4) break;
    }


    std::cout << "got calibration for channel 4. Will test now." << std::endl;
            
    for (int i = 0; i < 100; ++i) {
        communicateWithLSM6DOX(4);
        SensorStatusBits ssb = get_LSM6DSOX_status();
        LSM6DSOX_Data newData;

        if (!ssb.accelAvail || !ssb.gyroAvail) usleep(10000);
        
        LSM6DSOX_read_xyz(newData);

        Eigen::MatrixXd rawData (3, 1);
        
        rawData << newData.AccelData.x, newData.AccelData.y, newData.AccelData.z;

        std::cout << "This is the raw data:" << std::endl;
        std::cout << rawData << std::endl;
        std::cout << "This is the calibration:" << std::endl;
        std::cout << cd.calibrationMatrix * (rawData - cd.offsetsMatrix) << std::endl;

    }

    return 0;
}