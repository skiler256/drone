#pragma once
#include "../inc/gpio.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"

#include <mutex>
#include <atomic>
#include <optional>

#define SERVO1 23
#define SERVO2 26

#define CAM_LANDING 1
#define CAM_STAB 2

class GIMBALL
{
public:
    GIMBALL(eventManager &event, GPIO &gpio, std::optional<INS> &ins, const int refreshRate);
    ~GIMBALL();

    void runGimball();

    void set(const int MODE, const int value = 0);

private:
    eventManager &event;
    GPIO &gpio;
    std::optional<INS> &ins;
    int refreshRate;

    std::mutex mtx;
    std::mutex mtxLoop;
    std::atomic<bool> loop = true;

    int mode = 0;
    INS::state3D state;

    void setServo1(int angle);
    void setServo2(int angle);

    void idle();
};