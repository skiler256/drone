#pragma once
#include "../inc/gpio.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"

#include <mutex>
#include <atomic>
#include <optional>

#include <string>

#define SERVO1 23
#define SERVO2 26

#define CAM_LANDING 1
#define CAM_STAB 2
#define CAM_HORIZONTAL 3

class GIMBALL
{
public:
    GIMBALL(eventManager &event, GPIO &gpio, std::optional<INS> &ins, const int refreshRate);
    ~GIMBALL();

    struct config
    {
        int servo1;
        int servo2;
        int mode;
    };

    config getConfig();
    void runGimball();

    void doCommand(std::string command);

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
    int offsets[2] = {0, 0};
    INS::state3D state;

    config configCur;

    void setServo1(int angle);
    void setServo2(int angle);

    void idle();
};