#include "../inc/gpio.hpp"

gpio::gpio(){
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Erreur d'initialisation de WiringPi\n";
    }
}
void gpio::write(const int pin, const bool state){
    if(!gpioIsInit[pin]) {
        pinMode(pin, OUTPUT);
        gpioIsInit[pin] = true;
    }
    digitalWrite(pin,state);
}

bool gpio::read(const int pin){
    if(!gpioIsInit[pin]) {
        pinMode(pin, INPUT);
        pullUpDnControl(pin, PUD_DOWN);
        gpioIsInit[pin] = true;
    }
    return digitalRead(pin);
}