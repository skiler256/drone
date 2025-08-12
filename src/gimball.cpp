#include "../inc/gimball.hpp"
#include <chrono>
#include <thread>

GIMBALL::GIMBALL(eventManager &event, GPIO &gpio, std::optional<INS> &ins, const int refreshRate) : event(event),
                                                                                                    gpio(gpio),
                                                                                                    ins(ins),
                                                                                                    refreshRate(refreshRate)
{
    setAngle(SERVO1, 20);
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

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
        if (!loop)
            return;
    }
}

void GIMBALL::setAngle(const int servo, const int angle)
{
    const int duty = angle * 80 / 180 + 25;
    gpio.writePWM(servo, duty);
}