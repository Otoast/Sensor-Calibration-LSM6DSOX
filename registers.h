#ifndef REGISTER_FUNC  
#define REGISTER_FUNC 

#include <bcm2835.h>
#include <iostream>
#include <unistd.h>


constexpr uint8_t I2C_DEVICE_ID = 0x70;   // Default address of SparkFun I2C Mux
constexpr uint8_t BUTTON_ADDRESS = 0x6F;  // Qwiic Button address
constexpr uint8_t BUTTON_CHANNEL = 1; // Channel the button is connected to on the I2C Mux

constexpr uint8_t LSM6DSOX_ADDRESS = 0x6A; // LSM6DSOX accelerometer address


/// @brief Refer to Qwiic Button Register Map to gain more information.
enum class ButtonCommands {
    ID = 1, FIRMWARE_LSB = 2, FIRMWARE_MSB = 2, BUTTON_STATUS = 3, INPUT_CONFIG = 4, BUTTON_DEBOUNCE_TIME = 5,
    PRESSED_QUEUE_STATUS = 7, PRESSED_QUEUE_FRONT = 8, PRESSED_QUEUE_BACK = 12, CLICKED_QUEUE_STATUS = 16,
    CLICKED_QUEUE_FRONT = 17, CLICKED_QUEUE_BACK = 21, LED_BRIGHTNESS = 25, LED_PULSE_GRANULARITY = 26,
    LED_PULSE_CYCLE_TIME = 27, LED_PULSE_OFF_TIME = 29, I2C_ADDRESS = 31 
};

/// @brief Refer to the LSM6DSOX datasheet to gain more information.
enum class LSM6DSOX_Commands {
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10, CTRL2_G = 0x11, CTRL3_C = 0x12, CTRL8_XL = 0x17,
    CTRL10_C = 0x19, STATUS_REG = 0x1E,

    OUTX_L_GYROSCOPE = 0x22, OUTX_H_GYROSCOPE = 0x23,
    OUTY_L_GYROSCOPE = 0x24, OUTY_H_GYROSCOPE = 0x25, 
    OUTZ_L_GYROSCOPE = 0x26, OUTZ_H_GYROSCOPE = 0x27,

    OUTX_L_ACCELEROMETER = 0x28, OUTX_H_ACCELEROMETER = 0x29,
    OUTY_L_ACCELEROMETER = 0x2A, OUTY_H_ACCELEROMETER = 0x2B,
    OUTZ_L_ACCELEROMETER = 0x2C, OUTZ_H_ACCELEROMETER = 0x2D,

    TIMESTAMP0 = 0x40, TIMESTAMP1 = 0x41, TIMESTAMP2 = 0x42, TIMESTAMP3 = 0x43
};


/// @brief eventAvailable -> Bit 0. hasBeenClicked -> Bit 1. isPressed -> Bit 2.
struct ButtonStatusBits {
    bool eventAvailable : 1;
    bool hasBeenClicked : 1;
    bool isPressed : 1;
};

/// @brief tempAvailable -> Bit 2. gyroAvailable -> Bit 1. accelAvailable -> Bit 0.
struct SensorStatusBits {
    bool tempAvail : 1;
    bool gyroAvail : 1;
    bool accelAvail : 1;
};
/// @brief Struct that contians the xyz coordinates for both the accelerometer portion and gyroscope portion of the LSM6DSOX data.
struct LSM6DSOX_Data {
    struct {double x, y, z;} AccelData;
    struct {double omega_x,  omega_y, omega_z;} GyroData;

    friend std::ostream& operator<<(std::ostream& os, const LSM6DSOX_Data& data);
    LSM6DSOX_Data& operator+=(const LSM6DSOX_Data& other);
    LSM6DSOX_Data& operator/=(int val);

    LSM6DSOX_Data() : AccelData({}), GyroData({}) {};
};

struct CalibrationData {
    int channelNum;
    double x_offset, y_offset, z_offset;
    double omega_x_offset, omega_y_offset, omega_z_offset;
    double calibrationParams[3][3];

    CalibrationData() : channelNum(-1), x_offset(0), y_offset(0), z_offset(0), omega_x_offset(0), omega_y_offset(0), omega_z_offset(0), calibrationParams({}) {}
    std::string exportData();
};


/// @brief Sets the I2C_DEVICE_ID as the slave address and selects the I2C channel. 
/// @param channel Channel to be selected on the I2C Mux.
void selectChannel(uint8_t channel);

/// @brief Routine function to automatically select the button and set the address up for communication. Helps in guaranteeing intended functionality.
/// @param buttonChannel Channel the button is on for the I2C Multiplexor
void communicateWithButton(uint8_t buttonChannel);


/// @brief Routine function to automatically select the LSM6DSOX accelerometer and set the address up for communication. Helps in guaranteeing intended functionality.
/// @param LSM6DSOX_Channel Channel the LSM6DSOX is on for intended communication.
void communicateWithLSM6DOX(uint8_t LSM6DSOX_Channel);


/// @brief Sets led brightness given the parameter value.
/// @param value Brightness value. If it is not between the values of 0 and 255, the operation will not be written.
void setLEDBrightness(uint8_t value);


/// @brief Gets the current button status, noting if the button was pressed or clicked. Person can choose whether to clear the status immediately after.
/// @param clearStatus Decides whether to clear button status after reading. Defaults to true.
/// @return ButtonStatusBits stuct, which contains the bits 0, 1, and 2 of the response ==> 0 = eventAvailable, 1 = hasBeenClicked, 2 = isPressed.
ButtonStatusBits getButtonStatus (bool clearStatus=true);

/// @brief Finds all LSM6DSOX Channels based on the "WHOAMI" register. Searches for a 0x6C hex response from the 0x15 register. Can lead to collisions if other, non-accelerometer
/// devices send the same hex on the same register.
/// @param channels buffer for channel numbers on the I2C Multiplextor
/// @param channelSize the Max amount of available channels on the I2C Multiplexor
/// @return int -- The number of LSM6DSOX devices discovered
int findSensorChannels(int channels[], int channelSize);

#endif