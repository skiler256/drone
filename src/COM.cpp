#include "../inc/COM.hpp"
#include <thread>

COM::COM(sysMonitoring &monitoring, const int refreshRate, const int port) : monitoring(monitoring), refreshRate(refreshRate), port(port)
{
}

void COM::startWS()
{
    uWS::App({.key_file_name = "misc/key.pem",
              .cert_file_name = "misc/cert.pem",
              .passphrase = "1234"})
        .ws<dataWS>("/*", {/* Settings */
                           .compression = uWS::CompressOptions(uWS::DEDICATED_COMPRESSOR | uWS::DEDICATED_DECOMPRESSOR),
                           .maxPayloadLength = 100 * 1024 * 1024,
                           .idleTimeout = 16,
                           .maxBackpressure = 100 * 1024 * 1024,
                           .closeOnBackpressureLimit = false,
                           .resetIdleTimeoutOnSend = false,
                           .sendPingsAutomatically = true,
                           /* Handlers */
                           .upgrade = nullptr,
                           .open = [this](auto *ws)
                           {
                  std::lock_guard<std::mutex> lock(wsMTX);
                  clients.push_back(ws);
                    std::cout<< ws << std::endl; },
                           .message = [this](auto *ws, std::string_view message, uWS::OpCode opCode)
                           {
                    
                    std::string mess = "ta geule";
                    ws->send(mess, opCode, false); },

                           .close = [this](auto *ws, int code, std::string_view message)
                           {
                  std::lock_guard<std::mutex> lock(wsMTX);
                  clients.remove(ws); }})
        .listen(port, [](auto *listen_socket)
                {
if (listen_socket) {
std::cout << "Listening on port " << 9001 << std::endl;
} })
        .run();
}

void COM::aquireData()
{
    std::lock_guard<std::mutex> lock(dataMTX);
    dataSystem = monitoring.getData();
}

void COM::sendData()
{
    std::lock_guard<std::mutex> locka(dataMTX);
    std::lock_guard<std::mutex> lockb(wsMTX);

    for (auto *client : clients)
    {
        std::string mess = std::to_string((int)dataSystem.state3D.att(2));
        client->send(mess, uWS::TEXT, false);
    }
}

void COM::runCOM()
{
    std::thread server(&COM::startWS, this);

    while (true)
    {
        const auto start = std::chrono::steady_clock::now();

        aquireData();
        sendData();

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
    }

    server.join();
}