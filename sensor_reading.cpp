
#include "registers.h"
#include "sensor_reading.h"
#include "sensor_calibration.h"

void _setUpSensor() {
    bcm2835_i2c_setSlaveAddress(LSM6DSOX_ADDRESS);

    auto writeSettings = [] (char command, char value) {
        char buf[2] = {command, value};
        bcm2835_i2c_write(buf, 2);
    };

    // Reset software -- start at clean slate
    writeSettings((char)LSM6DSOX_Commands::CTRL3_C, (char)0b1);
    // Speed up data sampling, enable low-pass filter for crtl_8 (Accel)
    writeSettings((char)LSM6DSOX_Commands::CTRL1_XL, (char)0b10101110);
    // Speed up data sampling (Gyro)
    writeSettings((char)LSM6DSOX_Commands::CTRL2_G, (char)0b10101000);
    // Data blocking
    writeSettings((char)LSM6DSOX_Commands::CTRL3_C, (char)0b01000000);
    // Set the filter-level for low-pass filtering (first 3 bits)
    writeSettings((char)LSM6DSOX_Commands::CTRL8_XL, (char)0b11100000);
    // Enable Timestamp
    writeSettings((char)LSM6DSOX_Commands::CTRL10_C, (char)0b00100000);

}

void _parseRegister(char outRegister, char* readBuf) {
    char writeBuf = outRegister;
    bcm2835_i2c_write(&writeBuf, 1);
    bcm2835_i2c_read(readBuf, 1);
}

void read_timestamp(uint32_t& timestamp_buf) {
    bcm2835_i2c_setSlaveAddress(LSM6DSOX_ADDRESS);

    char timeBuf[4];

    _parseRegister((char)LSM6DSOX_Commands::TIMESTAMP0, (char *)&timeBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::TIMESTAMP1, (char *)&timeBuf[1]);
    _parseRegister((char)LSM6DSOX_Commands::TIMESTAMP2, (char *)&timeBuf[2]);
    _parseRegister((char)LSM6DSOX_Commands::TIMESTAMP3, (char *)&timeBuf[3]);

    timestamp_buf = (uint32_t)timeBuf[0] | (uint32_t)timeBuf[1] << 8 | (uint32_t)timeBuf[2] << 16 | (uint32_t)timeBuf[3] << 24;
}


void _readAccel(LSM6DSOX_Data &d) {
    uint8_t readBuf[2];

    // X-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTX_L_ACCELEROMETER, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTX_H_ACCELEROMETER, (char *)&readBuf[1]);
    d.AccelData.x = LSM6DSOX_ACC_SENSITIVITY_FS_8G * MILLI_G_TO_G *
        (readBuf[0] | (int16_t)(readBuf[1] << 8));

    // Y-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTY_L_ACCELEROMETER, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTY_H_ACCELEROMETER, (char *)&readBuf[1]);
    d.AccelData.y = LSM6DSOX_ACC_SENSITIVITY_FS_8G * MILLI_G_TO_G *  
        (readBuf[0] | (int16_t)(readBuf[1] << 8));

    // Z-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTZ_L_ACCELEROMETER, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTZ_H_ACCELEROMETER, (char *)&readBuf[1]);
    d.AccelData.z = LSM6DSOX_ACC_SENSITIVITY_FS_8G * MILLI_G_TO_G *  
        (readBuf[0] | (int16_t)(readBuf[1] << 8));
    
}


void _readGyro(LSM6DSOX_Data &d) {
    char readBuf[2];


    // X-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTX_L_GYROSCOPE, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTX_H_GYROSCOPE, (char *)&readBuf[1]);
    d.GyroData.omega_x = .001f * LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS * DEGREES_TO_RADIANS * 
        (readBuf[0] | (int16_t)(readBuf[1] << 8));

    // Y-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTY_L_GYROSCOPE, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTY_H_GYROSCOPE, (char *)&readBuf[1]);
    d.GyroData.omega_y = .001f * LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS * DEGREES_TO_RADIANS * 
        (readBuf[0] | (int16_t)(readBuf[1] << 8));

    // Z-Axis low and high bits
    _parseRegister((char)LSM6DSOX_Commands::OUTZ_L_GYROSCOPE, (char *)&readBuf[0]);
    _parseRegister((char)LSM6DSOX_Commands::OUTZ_H_GYROSCOPE, (char *)&readBuf[1]);
    d.GyroData.omega_z = .001f * LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS * DEGREES_TO_RADIANS * 
        (readBuf[0] | (int16_t)(readBuf[1] << 8));

}

void LSM6DSOX_read_xyz(LSM6DSOX_Data &accelData) {
    bcm2835_i2c_setSlaveAddress(LSM6DSOX_ADDRESS);
    _readAccel(accelData);
    _readGyro(accelData);
}

SensorStatusBits get_LSM6DSOX_status() {
    char writeBuf = (char)LSM6DSOX_Commands::STATUS_REG;
    bcm2835_i2c_write(&writeBuf, 1);

    uint8_t readBuf;
    bcm2835_i2c_read((char *)&readBuf, 1);

    SensorStatusBits ssb;
    ssb.accelAvail = (readBuf & 0b1) != 0;
    ssb.gyroAvail = (readBuf & 0b10) != 0;
    ssb.tempAvail = (readBuf & 0b100) != 0;

    return ssb;

}


std::ofstream setUpOutputFile() {
    time_t currTime = time(nullptr);
    tm *g = gmtime(&currTime);
    char buf[20];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H_%M_%S", g);

    std::string filename = std::string(buf) + " UTC.csv";

    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to create file" << filename << std::endl;
        exit(1);
    }
    return outFile;
}

void LSM6DSOX_calibrationCorrection(LSM6DSOX_Data& data, CalibrationData& cd) {
    Eigen::MatrixXd accelMatrix (3, 1);
    accelMatrix << data.AccelData.x, data.AccelData.y, data.AccelData.z;
    accelMatrix = cd.calibrationMatrix * (accelMatrix - cd.offsetsMatrix);
    
    Eigen::MatrixXd gyroMatrix (3, 1);
    gyroMatrix << data.GyroData.omega_x, data.GyroData.omega_y, data.GyroData.omega_z;
    gyroMatrix = gyroMatrix - cd.omegaOffsetsMatrix;

    data.AccelData.x = accelMatrix(0, 0);
    data.AccelData.y = accelMatrix(1, 0);
    data.AccelData.z = accelMatrix(2, 0);
  
    data.GyroData.omega_x = gyroMatrix(0, 0);
    data.GyroData.omega_y = gyroMatrix(1, 0);
    data.GyroData.omega_z = gyroMatrix(2, 0);
}

void sensor_reading() {
    communicateWithButton(BUTTON_CHANNEL);
    setLEDBrightness(40);   

    int lsm6dsox_channels[MAX_I2C_CHANNELS]; 

    int amtChannels = findSensorChannels(lsm6dsox_channels, MAX_I2C_CHANNELS);
    
    // Settings for data collection
    for (int i = 0; i < amtChannels; ++i) {
        int channel = lsm6dsox_channels[i];
        communicateWithLSM6DOX(channel);
        _setUpSensor();
    }

    // Calibration data for each sensor parsed. This is for easy mapping of sensor to calibration data
    CalibrationData sensorCalibrations[MAX_I2C_CHANNELS];
    {
        std::vector<CalibrationData> calibData = sensorCalibration();
        for (int i = 0; i < (int)calibData.size(); ++i) {
            CalibrationData cd = calibData.at(i);
            sensorCalibrations[cd.channelNum] = cd;
        }    
    }

    std::ofstream outputFile = setUpOutputFile();
    LSM6DSOX_Data myData;
    SensorStatusBits ssb;

    communicateWithButton(BUTTON_CHANNEL);
    setLEDBrightness(255);

    bool buttonClicked = false;
    while (!buttonClicked) {
        long long timestamp = time(nullptr);
        for (int i = 0; i < amtChannels; ++i) {
            int channel = lsm6dsox_channels[i];
            communicateWithLSM6DOX(channel);
            ssb = get_LSM6DSOX_status();
            
            // Busy waiting, but ideally should never need to be on this section for long, and this is the only process running
            while(!(ssb.accelAvail && ssb.gyroAvail)) ssb = get_LSM6DSOX_status();

            LSM6DSOX_read_xyz(myData);
            LSM6DSOX_calibrationCorrection(myData, sensorCalibrations[channel]);
            outputFile << timestamp << "," << channel << "," << myData << "\n";
        }
        communicateWithButton(BUTTON_CHANNEL);
        buttonClicked = getButtonStatus().hasBeenClicked;
        bcm2835_delay(10);
    }

    outputFile.close();
    communicateWithButton(BUTTON_CHANNEL);
    setLEDBrightness(0);
}
