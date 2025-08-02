#pragma once
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"

#include <mutex>

class sysMonitoring
{
public:
    sysMonitoring(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS &ins, const int refreshRate);

    void runSysMonitoring();

    struct sensorData
    {
        ESPdata esp;
        BMP280::Data baro;
        NEO6m::gpsState gps;
    };
    struct sysData
    {
        sensorData sensor;
        INS::state3D state3D;
        std::map<std::pair<component, subcomponent>, eventManager::eventLog> events;
    };

    sysData getData();

private:
    eventManager &event;
    ESP32 &esp;
    BMP280 &baro;
    NEO6m &gps;
    INS &ins;
    int refreshRate;

    std::mutex mtx;
    sysData data;
};