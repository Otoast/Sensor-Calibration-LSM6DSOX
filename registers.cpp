#include "registers.h"

std::ostream& operator<<(std::ostream& os, const LSM6DSOX_Data& data) {
    os << data.AccelData.x << "," << data.AccelData.y << "," << data.AccelData.z << ",";
    os << data.GyroData.x << "," << data.GyroData.y << "," << data.GyroData.z;
    return os;
}

LSM6DSOX_Data& LSM6DSOX_Data::operator+=(const LSM6DSOX_Data& other) {
    AccelData.x += other.AccelData.x;
    AccelData.y += other.AccelData.y;
    AccelData.z += other.AccelData.z;
    GyroData.x  += other.GyroData.x;
    GyroData.y  += other.GyroData.y;
    GyroData.z  += other.GyroData.z;
    return *this;
}

LSM6DSOX_Data& LSM6DSOX_Data::operator/=(int val) {
    AccelData.x /= val;
    AccelData.y /= val;
    AccelData.z /= val;
    GyroData.x  /= val;
    GyroData.y  /= val;
    GyroData.z  /= val;
    return *this;
}

std::string CalibrationData::exportData() {    
    std::string data = std::to_string(channelNum) + "\t" + std::to_string(x_offset) + "\t" + std::to_string(y_offset) + "\t" + std::to_string(z_offset); 
    for (int i = 0; i < 3; ++i) 
        for (int j = 0; j < 3; ++j)
            data += "\t" + std::to_string(calibrationParams[i][j]);
    
    return data;
}


void selectChannel(uint8_t channel) {
    bcm2835_i2c_setSlaveAddress(I2C_DEVICE_ID);
    if (channel > 7) {
        std::cerr << "Invalid channel" << channel << std::endl;
        exit(1);
    }
    char channel_write = 1 << channel;
    if (bcm2835_i2c_write(&channel_write, 1) != BCM2835_I2C_REASON_OK) {
        std::cerr << "there was an issue setting channel write" << std::endl;
        exit(1);
    }
}

void communicateWithButton(uint8_t buttonChannel) {
    bcm2835_i2c_setSlaveAddress(I2C_DEVICE_ID);
    selectChannel(buttonChannel);
    bcm2835_i2c_setSlaveAddress(BUTTON_ADDRESS);
}

void communicateWithLSM6DOX(uint8_t LSM6DSOX_Channel) {
    bcm2835_i2c_setSlaveAddress(I2C_DEVICE_ID);
    selectChannel(LSM6DSOX_Channel);
    bcm2835_i2c_setSlaveAddress(LSM6DSOX_ADDRESS);
}


void setLEDBrightness(uint8_t value) {
    bcm2835_i2c_setSlaveAddress(BUTTON_ADDRESS);
    char args[2] = {(char)ButtonCommands::LED_BRIGHTNESS, (char)value};
    if (bcm2835_i2c_write(args, 2) != BCM2835_I2C_REASON_OK) {
        std::cerr << "Issue setting brightness" << std::endl;
    }
}


ButtonStatusBits getButtonStatus (bool clearStatus) {
    bcm2835_i2c_setSlaveAddress(BUTTON_ADDRESS);
    uint8_t buf[1] = {(uint8_t)ButtonCommands::BUTTON_STATUS};
    if (bcm2835_i2c_write((char*)buf, 1) != BCM2835_I2C_REASON_OK) {
        std::cerr << "Issue getting button status" << std::endl;
    }
    if (bcm2835_i2c_read((char*)buf, 1) != BCM2835_I2C_REASON_OK) {
        std::cerr << "Issue getting button status" << std::endl;
    }
    
    uint8_t i_buf = buf[0];
    ButtonStatusBits bsb = {
        (i_buf & 0b1) != 0, 
        (i_buf & 0b10) != 0, 
        (i_buf & 0b100) != 0
    };
    if (clearStatus) {
        char args[2] = {(char)ButtonCommands::BUTTON_STATUS, (char)0};
        if (bcm2835_i2c_write(args, 2) != BCM2835_I2C_REASON_OK) {
            std::cerr << "Issue clearing button status" << std::endl;
        }
    }
    return bsb;
}


int findSensorChannels(int channels[], int channelSize) {
    int size = 0; 
    for (int i = 0; i < channelSize; ++i) {
        communicateWithLSM6DOX(i);
        char buf[] = {(char)LSM6DSOX_Commands::WHO_AM_I};
        uint8_t status = bcm2835_i2c_read_register_rs(buf, buf, 1);
        if (status == BCM2835_I2C_REASON_OK && buf[0] == 0x6c) {
            channels[size++] = i;
        }
    }
    return size;
}
