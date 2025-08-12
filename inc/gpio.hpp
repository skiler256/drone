#pragma once
#include <wiringPi.h>
#include <iostream>
#include <mutex>

class GPIO
{

public:
    GPIO();

    void write(const int pin, const bool state);
    bool read(const int pin);
    void writePWM(const int pin, int duty);

private:
    int gpioType[40] = {-1};
    std::mutex mtx;
};