#include "../inc/sysMonitoring.hpp"
#include <chrono>
#include <thread>
#include <iostream>

sysMonitoring::sysMonitoring(eventManager &event, std::optional<ESP32> &esp, std::optional<BMP280> &baro, std::optional<NEO6m> &gps, std::optional<INS> &ins,
                             std::optional<GIMBALL> &gimball, std::optional<TFluna> &tele, const int refreshRate) : event(event),
                                                                                                                    esp(esp),
                                                                                                                    baro(baro),
                                                                                                                    gps(gps),
                                                                                                                    ins(ins),
                                                                                                                    gimball(gimball),
                                                                                                                    tele(tele),
                                                                                                                    refreshRate(refreshRate) {}

sysMonitoring::~sysMonitoring()
{
    std::lock_guard<std::mutex> lock(mtx);
    loop = false;
    std::lock_guard<std::mutex> lockb(mtxLoop);
}

void sysMonitoring::runSysMonitoring()
{
    while (loop)
    {
        const auto start = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mtx);
            if (esp)
                data.sensor.esp = esp->getData();
            if (baro)
                data.sensor.baro = baro->getData(); // peut etre ah .. voilou voilou
            if (gps)
                data.sensor.gps = gps->getGPSState();

            if (ins)
                data.state3D = ins->getState3D();
            if (gimball)
                data.gimball = gimball->getConfig();

            if (tele)
                data.sensor.Tele = tele->getDist();

            data.events = event.getEvents();

            data.perf.CPUtemp = getCPUTemp();
            data.perf.RAMusage = getRAMUsage();
        }

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
        if (!loop)
            return;
    }
}
sysMonitoring::sysData sysMonitoring::getData()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

double sysMonitoring::getCPUTemp()
{
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    int tempMilli;
    file >> tempMilli;
    return (double)tempMilli / 1000.0;
}

double sysMonitoring::getRAMUsage()
{
    std::ifstream file("/proc/meminfo");
    std::string label;
    unsigned long memTotal = 0, memFree = 0, buffers = 0, cached = 0;

    while (file >> label)
    {
        if (label == "MemTotal:")
            file >> memTotal;
        else if (label == "MemFree:")
            file >> memFree;
        else if (label == "Buffers:")
            file >> buffers;
        else if (label == "Cached:")
            file >> cached;

        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    unsigned long usedKB = memTotal - memFree - buffers - cached;
    float usedMB = usedKB / 1024.0;
    float usedGB = usedMB / 1024.0;

    return (double)usedMB;
}
