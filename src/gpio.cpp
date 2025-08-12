#include "../inc/gpio.hpp"

GPIO::GPIO()
{
    std::lock_guard<std::mutex> lock(mtx);
    std::fill(std::begin(gpioType), std::end(gpioType), -1);
    if (wiringPiSetupGpio() == -1)
    {
        std::cerr << "Erreur d'initialisation de WiringPi\n";
    }

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);
    pwmSetRange(1000);
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
        pwmWrite(pin, duty);
    }
    else if (gpioType[pin] == PWM_OUTPUT)
        pwmWrite(pin, duty);
}