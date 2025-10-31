#pragma once
#include "../inc/MS5611.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"
#include "../inc/gimball.hpp"
#include "../inc/TF-luna.hpp"

#include <mutex>
#include <optional>
#include <atomic>

class sysMonitoring
{
public:
    sysMonitoring(eventManager &event, std::optional<ESP32> &esp, std::optional<MS5611> &baro, std::optional<NEO6m> &gps, std::optional<INS> &ins,
                  std::optional<GIMBALL> &gimball, std::optional<TFluna> &tele, const int refreshRate);
    ~sysMonitoring();
    void runSysMonitoring();

    struct sensorData
    {
        ESPdata esp;
        MS5611::Data baro;
        NEO6m::gpsState gps;
        double Tele;
    };
    struct PIperf
    {
        double CPUtemp;
        double RAMusage;
    };
    struct stateModule
    {
        bool INS = false;
        bool GPS = false;
        bool ESP = false;
        bool BMP = false;
        bool GIM = false;
        bool TEL = false;
    };
    struct sysData
    {
        sensorData sensor;
        INS::state3D state3D;
        std::map<std::pair<component, subcomponent>, eventManager::eventLog> events;
        PIperf perf;
        GIMBALL::config gimball;
        stateModule moduleState;
    };
    struct telP1
    {
        int32_t latitude = 0;
        int32_t longitude = 0;
        int16_t pos[3] = {};
        int16_t att[3] = {};
        int16_t cpuTemp = 0;
        uint8_t state = 0;
    };
    struct telemetryPaket
    {
        telP1 p1;
    };

    sysData getData();
    telemetryPaket getTelemetryData();
    telemetryPaket &getTelemetryDataRef();

private:
    eventManager &event;
    std::optional<ESP32> &esp;
    std::optional<MS5611> &baro;
    std::optional<NEO6m> &gps;
    std::optional<INS> &ins;
    std::optional<GIMBALL> &gimball;
    std::optional<TFluna> &tele;
    int refreshRate;

    std::mutex mtx;
    std::mutex Telemtx;
    sysData data;
    telemetryPaket telemetryData;

    double getCPUTemp();
    double getRAMUsage();

    std::atomic<bool> loop = true;
    std::thread run;

    friend class behaviorCenter;
};