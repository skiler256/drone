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

void launcher::startBARO()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
        baro.emplace(*event);
}
void launcher::startESP()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        esp.emplace(*event);
        std::thread a(&ESP32::runESP32, &(*esp));
        a.detach();
    }
}
void launcher::startGPS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        gps.emplace(*event);
        std::thread a(&NEO6m::runNEO6m, &(*gps));
        a.detach();
    }
}

void launcher::startINS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event && esp && baro && gps)
    {
        ins.emplace(*event, *esp, *baro, *gps, parameters.insSettings);
        std::thread a(&INS::runINS, &(*ins));
        a.detach();
    }
}

void launcher::startPCA()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        pca.emplace(*event);
        std::thread a(&PCA9685::runPCA9685, &(*pca));
        a.detach();
    }
}