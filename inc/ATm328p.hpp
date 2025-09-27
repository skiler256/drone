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

class ATm328p
{
public:
    ATm328p(eventManager &event, std::optional<sysMonitoring> &monitoring, const char *portName = "/dev/ttyAMA2");
    ~ATm328p();

    void runTx();

private:
    eventManager &event;
    std::optional<sysMonitoring> &monitoring;
    const char *portName;
    int SerialPort;
    termios tty;

    std::atomic<bool> loopTx = true;
    std::thread Tx;
};