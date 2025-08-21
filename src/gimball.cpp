#include "../inc/gimball.hpp"
#include <chrono>
#include <thread>

GIMBALL::GIMBALL(eventManager &event, GPIO &gpio, std::optional<INS> &ins, const int refreshRate) : event(event),
                                                                                                    gpio(gpio),
                                                                                                    ins(ins),
                                                                                                    refreshRate(refreshRate)
{
    idle();
    run = std::thread(&GIMBALL::runGimball, this);
}

GIMBALL::~GIMBALL()
{
    loop = false;
    if (run.joinable())
        run.join();
}

void GIMBALL::runGimball()
{
    while (loop)
    {
        const auto start = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mtx);
            if (ins)
            {
                state = ins->getState3D();
            }
            switch (configCur.mode)
            {
            case CAM_STAB:
            {
                int angle1 = 90 - state.att(0);
                int angle2 = 90 + (state.att(1) + offsets[0]);

                if (angle2 > 180)
                    angle2 = 180;
                else if (angle2 < 80)
                    angle2 = 80;

                setServo1(angle1);
                setServo2(angle2);
                break;
            }
            case CAM_HORIZONTAL:
            {
                setServo1(0);
                setServo2(90 + offsets[1]);
                break;
            }
            }
        }
        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
    }
}

void GIMBALL::setServo1(int angle)
{

    if (angle > 135)
        angle = 135;
    else if (angle < 0)
        angle = 0;

    configCur.servo1 = angle;

    const int min = 26;
    const int max = 110;
    const int duty = min + (max - min) * angle / 180;

    gpio.writePWM(SERVO1, duty);
}
void GIMBALL::setServo2(int angle)
{
    if (angle < 0)
        angle = 0;
    else if (angle > 180)
        angle = 180;

    configCur.servo2 = angle;
    const int min = 30;
    const int max = 105;
    const int duty = min + (max - min) * angle / 180;

    gpio.writePWM(SERVO2, duty);
}

void GIMBALL::idle()
{
    setServo2(90);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    setServo1(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void GIMBALL::set(const int MODE, const int value)
{
    std::lock_guard<std::mutex> lock(mtx);
    configCur.mode = MODE;
    idle();
    switch (configCur.mode)
    {
    case CAM_LANDING:

        setServo1(90);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        setServo2(180);
        break;

    case 0:
        idle();
        break;
    }
}

GIMBALL::config GIMBALL::getConfig()
{
    std::lock_guard<std::mutex> lock(mtx);
    return configCur;
}

void GIMBALL::doCommand(std::string command)
{
    const int pas = 5;
    if (command == "LAND")
        set(CAM_LANDING);
    else if (command == "STAB")
        set(CAM_STAB);
    else if (command == "HORIZONTAL")
        set(CAM_HORIZONTAL);
    else if (command == "UP")
        offsets[0] -= pas;
    else if (command == "DOWN")
        offsets[0] += pas;
    else if (command == "LEFT")
        offsets[1] += pas;
    else if (command == "RIGHT")
        offsets[1] -= pas;
}