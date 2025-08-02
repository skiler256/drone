#include "../inc/sysMonitoring.hpp"
#include <chrono>
#include <thread>

sysMonitoring::sysMonitoring(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS &ins, const int refreshRate) : event(event),
                                                                                                                           esp(esp),
                                                                                                                           baro(baro),
                                                                                                                           gps(gps),
                                                                                                                           ins(ins),
                                                                                                                           refreshRate(refreshRate) {}

void sysMonitoring::runSysMonitoring()
{
    while (true)
    {
        const auto start = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mtx);
            data.sensor.esp = esp.getData();
            data.sensor.baro = baro.getData(); // peut etre ah .. voilou voilou
            data.sensor.gps = gps.getGPSState();

            data.state3D = ins.getState3D();
            data.events = event.getEvents();
        }

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
    }
}