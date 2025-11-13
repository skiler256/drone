#pragma once
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"
#include "../inc/sysMonitoring.hpp"
#include "../inc/COM.hpp"
#include "../inc/gimball.hpp"
#include "../inc/TF-luna.hpp"
#include "../inc/MS5611.hpp"
#include "../inc/ATm328p.hpp"
#include "../inc/LIS3MDL.hpp"

#include <optional>
#include <mutex>

class behaviorCenter;

class launcher
{
public:
    launcher(behaviorCenter &behavior);

    void startCOM();

    void startINS();

    void startESP();
    void startGPS();
    void startBARO();
    void startTele();

    void startPCA();

    void startGIMBALL();

    behaviorCenter &behavior;

    struct PARAMETERS
    {
        int monitoringRefreshRate = 10;
        int COMrefreshRate = 5;
        int COMport = 9001;
        INS::settings insSettings;
        int GIMBALLrate = 10;
    };
    PARAMETERS parameters;

    std::optional<eventManager> event;

    std::optional<ESP32> esp;
    std::optional<NEO6m> gps;
    std::optional<MS5611> baro;
    std::optional<LIS3MDL> mag;
    std::optional<TFluna> tele;

    std::optional<INS> ins;

    std::optional<PCA9685> pca;

    std::optional<sysMonitoring> monitoring;
    std::optional<ATm328p> telemetry;
    std::optional<COM> com;

    std::optional<GIMBALL> gimball;

    GPIO gpio;

private:
    std::mutex mtx;
};