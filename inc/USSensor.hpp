#pragma once
#include "gpio.hpp"
#include <unistd.h>
#include <chrono>

class USSensor {
public:
    USSensor(const int pin, gpio& PIN);
    double measure(const int pin);
private:
    int pinTrig;
    gpio& PIN;
};

