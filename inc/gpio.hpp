#pragma once
#include <wiringPi.h>
#include <iostream>

class gpio {

public :
    gpio();

    void write(const int pin, const bool state);
    bool read(const int pin);

private:
    bool gpioIsInit[40]={false};
};