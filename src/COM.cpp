#include "../inc/COM.hpp"
#include <thread>
#include <signal.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void to_json(json &j, const ESPdata &d)
{
    j = json{
        {"event", d.event},
        {"roll", d.roll},
        {"pitch", d.pitch},
        {"yaw", d.yaw},
        {"ax", d.ax},
        {"ay", d.ay},
        {"az", d.az},
        {"mx", d.mx},
        {"my", d.my},
        {"mz", d.mz}};
}

void to_json(json &j, const BMP280::Data &d)
{
    j = json{
        {"temp", d.temperature},
        {"pressure", d.pressure}};
}

void to_json(json &j, const NEO6m::coordPaket &c)
{
    j = json{
        {"longitude", c.longitude},
        {"latitude", c.latitude}};
}

void to_json(json &j, const NEO6m::SatelliteInfo &s)
{
    j = json{
        {"ID", s.ID},
        {"strenght", s.strenght},
        {"quality", s.quality}};
}

void to_json(json &j, const NEO6m::gpsState &g)
{
    j = json{
        {"sats", g.sats},
        {"coord", g.coord},
        {"timeArray", g.timeArray},
        {"gpsFixOk", g.gpsFixOk},
        {"velNED", g.velNED},
        {"speed", g.speed},
        {"GS", g.GS},
        {"heading", g.heading}};
}

void to_json(json &j, const INS::state3D &s)
{
    j = json{
        {"pos", {s.pos(0), s.pos(1), s.pos(2)}},
        {"vel", {s.vel(0), s.vel(1), s.vel(2)}},
        {"att", {s.att(0), s.att(1), s.att(2)}}};
}

void to_json(json &j, const sysMonitoring::sensorData &s)
{
    j = json{
        {"esp", s.esp},
        {"baro", s.baro},
        {"gps", s.gps}};
}

void to_json(json &j, const sysMonitoring::PIperf &p)
{
    j = json{
        {"CPUtemp", p.CPUtemp},
        {"RAMusage", p.RAMusage}};
}

void to_json(json &j, const sysMonitoring::sysData &s)
{

    j = json{
        {"sensor", s.sensor},
        {"state3D", s.state3D},
        {"events", stringifyEventLogMap(s.events)},
        {"perf", s.perf}};
}

COM::COM(sysMonitoring &monitoring, const int refreshRate, const int port) : monitoring(monitoring), refreshRate(refreshRate), port(port)
{
    std::lock_guard<std::mutex> locka(dataMTX);
    std::lock_guard<std::mutex> lockb(wsMTX);
}

COM::~COM()
{
    std::lock_guard<std::mutex> locka(dataMTX);
    std::lock_guard<std::mutex> lockb(wsMTX);
    loop = false;
    us_listen_socket_close(0, listenSocket);
    std::lock_guard<std::mutex> lockB(mtxLoop);
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
                    /* std::cout<< ws << std::endl;*/ },
                           .message = [this](auto *ws, std::string_view message, uWS::OpCode opCode)
                           {
                    
                    std::string mess = "ta geule";
                    ws->send(mess, opCode, false); },

                           .close = [this](auto *ws, int code, std::string_view message)
                           {
                  std::lock_guard<std::mutex> lock(wsMTX);
                  clients.remove(ws); }})
        .listen(port, [this](auto *listen_socket)
                {
if (listen_socket) {
    listenSocket = listen_socket;

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

        json j = dataSystem;
        std::string jsonStr = j.dump();
        client->send(jsonStr, uWS::TEXT, false);
    }
}

void COM::runCOM()
{
    startNodeJS();
    std::thread server(&COM::startWS, this);

    while (loop)
    {
        std::lock_guard<std::mutex> lock(mtxLoop);

        const auto start = std::chrono::steady_clock::now();

        aquireData();
        sendData();

        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0 / refreshRate);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
        if (!loop)
            return;
    }

    server.join();
}

pid_t node_pid = -1; // processus serveur NodeJS

void startNodeJS()
{
    if (node_pid < 0)
    {
        node_pid = fork();

        if (node_pid == 0)
        {
            execlp("node", "node", "/home/jules/code/drone/app/server.js", (char *)nullptr);
            exit(1);
        }
        else if (node_pid > 0)
        {
            std::cout << "Node.js lancé avec PID " << node_pid << std::endl;
        }
        else
        {
            std::cerr << "Erreur fork" << std::endl;
        }
    }
}

void killNodeJS()
{
    if (node_pid > 0)
    {
        std::cout << "Arrêt du serveur Node.js (PID " << node_pid << ")..." << std::endl;
        kill(node_pid, SIGINT);
        std::cout << "node js stop" << std::endl;
        node_pid = -1;
    }
}