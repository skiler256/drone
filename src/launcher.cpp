#include "../inc/launcher.hpp"
#include <iostream>
#include <thread>

launcher::launcher(behaviorCenter &behavior) : behavior(behavior), gpio()
{
    std::lock_guard<std::mutex> lock(mtx);
    event.emplace();
    if (event)
    {
        monitoring.emplace(*this, parameters.COMrefreshRate);
        telemetry.emplace(*event, monitoring);

        sens.emplace(*this);
        ins = sens->getINSptr();
    }
    if (monitoring)
        com.emplace(*monitoring, *this, parameters.COMrefreshRate, parameters.COMport);
}

void launcher::startCOM()
{
    // std::lock_guard<std::mutex> lock(mtx);
    // event.emplace();
    // if (event)
    // {
    //     monitoring.emplace(*this, parameters.COMrefreshRate);
    //     telemetry.emplace(*event, monitoring);

    //     sens.emplace(*this);
    //     ins = sens->getINSref();
    // }
    // if (monitoring)
    //     com.emplace(*monitoring, *this, parameters.COMrefreshRate, parameters.COMport);
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
        esp.emplace(*event, *ins);
        mag.emplace(*event);
    }
}
void launcher::startGPS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        gps.emplace(*event, *ins);
    }
}

void launcher::startINS()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (event)
    {
        // ins.emplace(*event, baro, parameters.insSettings);
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
        gimball.emplace(*event, gpio, *ins, parameters.GIMBALLrate);
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