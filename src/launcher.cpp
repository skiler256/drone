#include "../inc/launcher.hpp"
#include <iostream>
#include <thread>

void launcher::startCOM()
{
    std::lock_guard<std::mutex> lock(mtx);
    event.emplace();
    if (event)
        monitoring.emplace(*event, esp, baro, gps, ins, parameters.COMrefreshRate);
    if (monitoring)
        com.emplace(*monitoring, parameters.COMrefreshRate, parameters.COMport);

    std::thread a(&sysMonitoring::runSysMonitoring, &(*monitoring));
    std::thread b(&COM::runCOM, &(*com));

    a.detach();
    b.detach();
}

void launcher::startSensor()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        esp.emplace(*event);
        gps.emplace(*event);
        baro.emplace(*event);
    }
}

void launcher::startINS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event && esp && baro && gps)
        ins.emplace(*event, *esp, *baro, *gps, parameters.insSettings);
}