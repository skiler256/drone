#include "../inc/sysMonitoring.hpp"
#include "../inc/launcher.hpp"
#include <chrono>
#include <thread>
#include <iostream>

sysMonitoring::sysMonitoring(launcher &lau, const int refreshRate) : lau(lau),
                                                                     refreshRate(refreshRate) { run = std::thread(&sysMonitoring::runSysMonitoring, this); }

sysMonitoring::~sysMonitoring()
{
    loop = false;
    if (run.joinable())
        run.join();
}

void sysMonitoring::runSysMonitoring()
{
    while (loop)
    {
        const auto start = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mtx);
            if (lau.esp)
                data.sensor.esp = lau.esp->getData();
            if (lau.baro)
                data.sensor.baro = lau.baro->getData(); // peut etre ah .. voilou voilou
            if (lau.gps)
                data.sensor.gps = lau.gps->getGPSState();

            if (lau.ins)
                data.state3D = lau.ins->getState3D();
            if (lau.gimball)
                data.gimball = lau.gimball->getConfig();

            if (lau.tele)
                data.sensor.Tele = lau.tele->getDist();
            if (lau.telemetry)
                data.vBat = lau.telemetry->vBat;

            data.events = lau.event->getEvents();
            data.IDs = lau.event->getIDs();

            data.perf.CPUtemp = getCPUTemp();
            data.perf.RAMusage = getRAMUsage();
        }

        {
            std::lock_guard<std::mutex> lockTel(Telemtx);
            std::lock_guard<std::mutex> lock(mtx);
            telemetryData.p1.latitude = static_cast<int32_t>(std::lround(data.sensor.gps.coord.latitude * 100000));
            telemetryData.p1.longitude = static_cast<int32_t>(std::lround(data.sensor.gps.coord.longitude * 100000));
            telemetryData.p1.cpuTemp = static_cast<int16_t>(std::lround(data.perf.CPUtemp * 10));
            for (int i = 0; i < 3; i++)
            {
                telemetryData.p1.pos[i] = static_cast<int16_t>(std::lround(data.state3D.pos(i) * 10));
                telemetryData.p1.att[i] = static_cast<int16_t>(std::lround(data.state3D.att(i) * 10));
            }
            uint8_t stateBuff = 0;

            if (data.moduleState.INS)
                stateBuff |= (1 << 0);
            if (data.moduleState.GPS)
                stateBuff |= (1 << 1);
            if (data.moduleState.ESP)
                stateBuff |= (1 << 2);
            if (data.moduleState.BMP)
                stateBuff |= (1 << 3);
            if (data.moduleState.GIM)
                stateBuff |= (1 << 4);
            if (data.moduleState.TEL)
                stateBuff |= (1 << 5);
            if (data.state3D.INSstate == 2)
                stateBuff |= (1 << 6);
            if (data.sensor.gps.gpsFixOk)
                stateBuff |= (1 << 7);

            telemetryData.p1.state = stateBuff;
        }

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
    }
}
sysMonitoring::sysData sysMonitoring::getData()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

sysMonitoring::telemetryPaket sysMonitoring::getTelemetryData()
{
    std::lock_guard<std::mutex> lock(Telemtx);
    return telemetryData;
}

sysMonitoring::telemetryPaket &sysMonitoring::getTelemetryDataRef()
{
    std::lock_guard<std::mutex> lock(Telemtx);
    return telemetryData;
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
    // float usedGB = usedMB / 1024.0;

    return (double)usedMB;
}
