#pragma once

#include "../inc/eventManager.hpp"
#include <fstream>
#include <mutex>
#include <termios.h>
#include <vector>

#include <atomic>
#include <thread>
#include <optional>

class sysMonitoring;

#pragma pack(push, 1)

struct ATmegaPaket
{

    float att[3];
    int16_t acc[3];
    int16_t vBat;
    uint8_t telCommand = 0;
};

#pragma pack(pop)

class SensorFusion;

class ATm328p
{
public:
    ATm328p(eventManager &event, std::optional<SensorFusion> &sens, std::optional<sysMonitoring> &monitoring, const char *portName = "/dev/ttyAMA2");
    ~ATm328p();

    void runTx();
    void runRx();

    float vBat = 0.0;

private:
    eventManager &event;
    std::optional<SensorFusion> &sens;
    std::optional<sysMonitoring> &monitoring;
    const char *portName;
    int SerialPort;
    termios tty;

    std::atomic<bool> loopTx = true;
    std::atomic<bool> loopRx = true;
    std::thread Tx;
    std::thread Rx;
};