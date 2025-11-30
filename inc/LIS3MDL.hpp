#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <array>
#include <optional>

#include "eventManager.hpp"

class SensorFusion;

class LIS3MDL
{
public:
    LIS3MDL(eventManager &event, std::optional<SensorFusion> &sens, uint8_t address = 0x1C, const char *bus = "/dev/i2c-1");
    ~LIS3MDL();

    std::array<double, 3> getData();

private:
    eventManager &event;
    std::optional<SensorFusion> &sens;
    int file;

    std::array<double, 3> mag = {0, 0, 0};

    std::thread th;
    std::mutex mtx;
    std::atomic<bool> loop{true};

    void run();
};