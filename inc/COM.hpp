#pragma once
#include "/usr/local/include/uWebSockets/App.h"
#include "../inc/sysMonitoring.hpp"

#include <opencv2/opencv.hpp>

#include <mutex>
#include <atomic>
#include <string>

class launcher; // pour éviter d'appeler un header qui s'appel soit meme

void killNodeJS();
void startNodeJS();

class COM
{
public:
    COM(sysMonitoring &monitoring, launcher &launch, const int refreshRate, const int port = 9001);
    ~COM();
    struct dataWS
    {
    };

    void runCOM();

private:
    sysMonitoring &monitoring;
    launcher &launch;
    int refreshRate;
    const int port;

    void startWS();
    void sendData();
    // static void sendData(struct us_timer_t *t);
    void aquireData();

    void handleVideoClients();
    void handleVideoServer(std::list<int> &clientsVideo, cv::VideoCapture &cap);

    void handleCommand(std::string command);

    std::mutex wsMTX;
    std::mutex dataMTX;
    std::mutex clientVideo;
    std::mutex mtxLoop;

    std::list<uWS::WebSocket<false, true, dataWS> *> clients;
    std::list<int> clientsVideo;
    sysMonitoring::sysData dataSystem;

    struct us_listen_socket_t *listenSocket = nullptr;
    std::atomic<bool> loop = true;
    uWS::Loop *sendLoop = nullptr;
};

std::string convertToStringAndPrecision(double value, const int precision);