#pragma once
#include "/usr/local/include/uWebSockets/App.h"
#include "../inc/sysMonitoring.hpp"

#include <mutex>

void killNodeJS();
void startNodeJS();

class COM
{
public:
    COM(sysMonitoring &monitoring, const int refreshRate, const int port = 9001);
    struct dataWS
    {
    };

    void runCOM();

private:
    sysMonitoring &monitoring;
    int refreshRate;
    const int port;

    void startWS();
    void sendData();
    void aquireData();

    std::mutex wsMTX;
    std::mutex dataMTX;

    std::list<uWS::WebSocket<false, true, dataWS> *> clients;
    sysMonitoring::sysData dataSystem;
};
