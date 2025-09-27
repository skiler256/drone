#include "../inc/sysMonitoring.hpp"
#include "../inc/launcher.hpp"
#include <chrono>
#include <thread>
#include <iostream>

sysMonitoring::sysMonitoring(eventManager &event, std::optional<ESP32> &esp, std::optional<MS5611> &baro, std::optional<NEO6m> &gps, std::optional<INS> &ins,
                             std::optional<GIMBALL> &gimball, std::optional<TFluna> &tele, const int refreshRate) : event(event),
                                                                                                                    esp(esp),
                                                                                                                    baro(baro),
                                                                                                                    gps(gps),
                                                                                                                    ins(ins),
                                                                                                                    gimball(gimball),
                                                                                                                    tele(tele),
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
