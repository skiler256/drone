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

#include <optional>
#include <mutex>

class launcher
{
public:
    launcher();

    void startCOM();

    void startINS();

    void startESP();
    void startGPS();
    void startBARO();

    void startPCA();

    void startGIMBALL();

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
    std::optional<BMP280> baro;

    std::optional<INS> ins;

    std::optional<PCA9685> pca;

    std::optional<sysMonitoring> monitoring;
    std::optional<COM> com;

    std::optional<GIMBALL> gimball;

    GPIO gpio;

private:
    std::mutex mtx;
};