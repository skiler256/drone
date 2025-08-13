#include "../inc/gimball.hpp"
#include <chrono>
#include <thread>

GIMBALL::GIMBALL(eventManager &event, GPIO &gpio, std::optional<INS> &ins, const int refreshRate) : event(event),
                                                                                                    gpio(gpio),
                                                                                                    ins(ins),
                                                                                                    refreshRate(refreshRate)
{
    idle();
}

GIMBALL::~GIMBALL()
{
    std::lock_guard<std::mutex> lock(mtx);
    loop = false;
    std::lock_guard<std::mutex> lockb(mtxLoop);
}

void GIMBALL::runGimball()
{
    while (loop)
    {
        std::lock_guard<std::mutex> lockb(mtxLoop);
        const auto start = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(mtx);
        if (ins)
        {
            state = ins->getState3D();
        }
        switch (mode)
        {
        case CAM_STAB:
            int angle1 = 90 - state.att(0);
            int angle2 = 90 + state.att(1);

            if (angle2 > 180)
                angle2 = 180;
            else if (angle2 < 80)
                angle2 = 80;

            setServo1(angle1);
            setServo2(angle2);
            break;
        }
        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
        if (!loop)
            return;
    }
}

void GIMBALL::setServo1(int angle)
{

    if (angle > 135)
        angle = 135;
    else if (angle < 0)
        angle = 0;

    const int min = 26;
    const int max = 110;
    const int duty = min + (max - min) * angle / 180;

    gpio.writePWM(SERVO1, duty);
}
void GIMBALL::setServo2(int angle)
{
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
    mode = MODE;

    switch (mode)
    {
    case CAM_LANDING:
        idle();
        setServo1(90);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        setServo2(180);
        break;

    case 0:
        idle();
        break;
    }
}