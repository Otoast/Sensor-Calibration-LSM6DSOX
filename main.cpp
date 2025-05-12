#include "registers.h"
#include "sensor_reading.h"
#include "sensor_calibration.h"


using namespace std;


void blinkButton() {
    for (int i = 0; i < 3; ++i) {
        setLEDBrightness(255);
        bcm2835_delay(250);
        setLEDBrightness(0);
        bcm2835_delay(250);
    }
}


int main() {
    if (!bcm2835_init() | !bcm2835_i2c_begin()) {
        cout << "Issue starting bcim and/or i2c." << endl;
        exit(1);
    }
    bcm2835_i2c_set_baudrate(400000);
    communicateWithButton(BUTTON_CHANNEL);
    blinkButton();
    getButtonStatus(); // Clear the button status bits

    ButtonStatusBits bsb;


    while (true) {
        bsb = getButtonStatus();
        if (bsb.hasBeenClicked) {
            cout << "Starting program... Will be reading into file." << endl;
            sensor_reading();
            cout << "Sucessfully finished." << endl;
        }
        bcm2835_delay(10);
    }
    return 0;

}

