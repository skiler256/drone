#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <optional>

#include "eventManager.hpp"

class SensorFusion;

class MS5611
{
public:
    struct Data
    {
        double temperature = 0.0;
        double pressure = 0.0;
    };

    struct IIRfilter
    {
        double y = 0;
        double update(double x, double alpha)
        {
            y = alpha * x + (1 - alpha) * y;
            return y;
        }
    };

    MS5611(eventManager &event, std::optional<SensorFusion> &sens, uint8_t address = 0x77, const char *bus = "/dev/i2c-4");
    ~MS5611();

    Data getData();

private:
    eventManager &event;
    std::optional<SensorFusion> &sens;
    int file;
    uint16_t C[7];
    Data data;
    IIRfilter tempFilter, presFilter;

    std::thread th;
    std::mutex mtx;
    std::atomic<bool> loop{true};

    void task(); // thread unique qui fait tout
};
