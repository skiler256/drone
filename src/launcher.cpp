#include "../inc/launcher.hpp"
#include <iostream>
#include <thread>

launcher::launcher(behaviorCenter &behavior) : behavior(behavior), gpio() {}

void launcher::startCOM()
{
    std::lock_guard<std::mutex> lock(mtx);
    event.emplace();
    if (event)
        monitoring.emplace(*event, esp, baro, gps, ins, gimball, tele, parameters.COMrefreshRate);
    if (monitoring)
        com.emplace(*monitoring, *this, parameters.COMrefreshRate, parameters.COMport);
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
    }
}
void launcher::startGPS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        gps.emplace(*event);
    }
}

void launcher::startINS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event && esp && baro && gps)
    {
        ins.emplace(*event, *esp, *baro, *gps, parameters.insSettings);
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

void launcher::startGIMBALL()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        gimball.emplace(*event, gpio, ins, parameters.GIMBALLrate);
    }
}

void launcher::startTele()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        tele.emplace(*event);
    }
}