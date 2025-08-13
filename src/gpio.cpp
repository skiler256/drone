#include "../inc/gpio.hpp"

GPIO::GPIO()
{
    std::lock_guard<std::mutex> lock(mtx);
    std::fill(std::begin(gpioType), std::end(gpioType), -1);
    wiringPiSetup();
}

void GPIO::write(const int pin, const bool state)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (gpioType[pin] == -1)
    {
        pinMode(pin, OUTPUT);
        gpioType[pin] = OUTPUT;
        digitalWrite(pin, state);
    }
    else if (gpioType[pin] == OUTPUT)
        digitalWrite(pin, state);
}

bool GPIO::read(const int pin)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (gpioType[pin] == -1)
    {
        pinMode(pin, INPUT);
        gpioType[pin] = INPUT;
        return digitalRead(pin);
    }
    else if (gpioType[pin] == INPUT)
        return digitalRead(pin);

    return false;
}

void GPIO::writePWM(const int pin, int duty)
{

    std::lock_guard<std::mutex> lock(mtx);
    if (gpioType[pin] == -1)
    {
        pinMode(pin, PWM_OUTPUT);
        gpioType[pin] = PWM_OUTPUT;
        pwmSetMode(PWM_MODE_MS);
        pwmSetClock(384);
        pwmSetRange(1000);
        pwmWrite(pin, duty);
        std::cout << "fgrezgrte" << pin << std::endl;
    }
    else if (gpioType[pin] == PWM_OUTPUT)
        pwmWrite(pin, duty);
}