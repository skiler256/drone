#pragma once
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"

#include <mutex>
#include <optional>
#include <atomic>

class sysMonitoring
{
public:
    sysMonitoring(eventManager &event, std::optional<ESP32> &esp, std::optional<BMP280> &baro, std::optional<NEO6m> &gps, std::optional<INS> &ins, const int refreshRate);
    ~sysMonitoring();
    void runSysMonitoring();

    struct sensorData
    {
        ESPdata esp;
        BMP280::Data baro;
        NEO6m::gpsState gps;
    };
    struct PIperf
    {
        double CPUtemp;
        double RAMusage;
    };
    struct sysData
    {
        sensorData sensor;
        INS::state3D state3D;
        std::map<std::pair<component, subcomponent>, eventManager::eventLog> events;
        PIperf perf;
    };

    sysData getData();

private:
    eventManager &event;
    std::optional<ESP32> &esp;
    std::optional<BMP280> &baro;
    std::optional<NEO6m> &gps;
    std::optional<INS> &ins;
    int refreshRate;

    std::mutex mtx;
    sysData data;

    double getCPUTemp();
    double getRAMUsage();

    std::atomic<bool> loop = true;
};