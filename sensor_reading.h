#ifndef SENSORS_FUNC  
#define SENSORS_FUNC  

#include "registers.h"
#include <ctime>
#include <fstream>

#define MILLI_G_TO_ACCEL 0.00980665f
#define MILLI_G_TO_G 1 / 1000.0f
#define DEGREES_TO_RADIANS 3.141592653589793f / 180

#define LSM6DSOX_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSOX_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSOX_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSOX_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSOX_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSOX_GYRO_SENSITIVITY_FS_2000DPS  70.000f

/// @brief Helper function to set up parameter options for each accelerometer. Refer to datasheet to know what each bit represents.
void _setUpSensor();

void _parseRegister(char outRegister, char* readBuf);

/// @brief Gets the timestamp from an accelerometer and writes the value in the timestamp buffer.
/// @param timestamp_buf Buffer int for timestamp
void read_timestamp(uint32_t& timestamp_buf);

void _readAccel(LSM6DSOX_Data &d);

void _readGyro(LSM6DSOX_Data &d);

void LSM6DSOX_read_xyz(LSM6DSOX_Data &accelData);

/// @brief Mainly used to observe if accelerometer, gyrosocope, and temperature sensors available.
/// @return SensorStatusBits struct providing the availability of these data points
SensorStatusBits get_LSM6DSOX_status();


std::ofstream setUpOutputFile();


void LSM6DSOX_calibrationCorrection(LSM6DSOX_Data& data, CalibrationData& cd);

void sensor_reading();

#endif