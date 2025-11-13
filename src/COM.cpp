#include "../inc/COM.hpp"
#include "../inc/launcher.hpp"
#include "../inc/behaviorCenter.hpp"
#include "../inc/eventManager.hpp"
#include <thread>
#include <signal.h>
#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>

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
        {"mz", d.mz},
        {"distances", {d.distances[0], d.distances[1], d.distances[2], d.distances[3], d.distances[4], d.distances[5]}}};
}

void to_json(json &j, const MS5611::Data &d)
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
        {"att", {s.att(0), s.att(1), s.att(2)}},
        {"accNED", {s.accNED(0), s.accNED(1), s.accNED(2)}},
        {"INSstate", s.INSstate}};
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

void to_json(json &j, const sysMonitoring::stateModule &m)
{
    j = json{
        {"INS", m.INS},
        {"ESP", m.ESP},
        {"GPS", m.GPS},
        {"BMP", m.BMP},
        {"GIM", m.GIM},
        {"TEL", m.TEL}};
}

void to_json(json &j, const sysMonitoring::sysData &s)
{

    j = json{
        {"sensor", s.sensor},
        {"state3D", s.state3D},
        {"events", strigifyEvents(s.events, s.IDs)},
        {"perf", s.perf},
        {"module", s.moduleState},
        {"vBat", s.vBat}};
}

COM::COM(sysMonitoring &monitoring, launcher &launch, const int refreshRate, const int port) : monitoring(monitoring), launch(launch), refreshRate(refreshRate), port(port)
{
    std::lock_guard<std::mutex> locka(dataMTX);
    std::lock_guard<std::mutex> lockb(wsMTX);
    run = std::thread(&COM::runCOM, this);
}

COM::~COM()
{
    loop = false;
    if (run.joinable())
        run.join();
}

void COM::startWS()
{
    uWS::App app({.key_file_name = "misc/key.pem",
                  .cert_file_name = "misc/cert.pem",
                  .passphrase = "1234"});
    sendLoop = uWS::Loop::get();
    app.ws<dataWS>("/*", {.compression = uWS::CompressOptions(uWS::DEDICATED_COMPRESSOR | uWS::DEDICATED_DECOMPRESSOR),
                          .maxPayloadLength = 100 * 1024 * 1024,
                          .idleTimeout = 16,
                          .maxBackpressure = 100 * 1024 * 1024,
                          .closeOnBackpressureLimit = false,
                          .resetIdleTimeoutOnSend = false,
                          .sendPingsAutomatically = true,

                          .upgrade = nullptr,

                          .open = [this](auto *ws)
                          {
            std::lock_guard<std::mutex> lk(wsMTX);
            clients.push_back(ws); },

                          .message = [this](auto *ws, std::string_view msg, uWS::OpCode)
                          { launch.behavior.interpretCommand(msg); },

                          .close = [this](auto *ws, int /*code*/, std::string_view /*message*/)
                          {
            std::lock_guard<std::mutex> lk(wsMTX);
            clients.remove(ws); }})
        .listen(port, [this](auto *listen_socket)
                {
        if (listen_socket) listenSocket = listen_socket; })
        .run();
}

void COM::handleCommand(std::string command)
{
    if (command.size() > 3)
    {
        std::string ID = command.substr(0, 3);
        std::string commandCore = command.erase(0, 3);
        if (ID == "GIM" || launch.gimball)
            launch.gimball->doCommand(commandCore);
    }
}

void COM::aquireData()
{
    std::lock_guard<std::mutex> lock(dataMTX);
    dataSystem = monitoring.getData();
}

void COM::sendData()
{
    if (sendLoop)
    {
        sendLoop->defer([this]()
                        {
        std::lock_guard<std::mutex> lock(wsMTX);
        std::lock_guard<std::mutex> lockb(dataMTX);
        json j = dataSystem;
        std::string jsonStr = j.dump();
        for (auto *client : clients) {
            if (client->getUserData()) // ou un flag custom pour "still open"
            client->send(jsonStr, uWS::TEXT, false);
        } });
    }
}

void COM::runCOM()
{
    startNodeJS();
    std::thread server(&COM::startWS, this);
    std::thread(&COM::handleVideoClients, this).detach();

    while (loop)
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

void COM::handleVideoClients()
{

    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12,framerate=20/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
        std::cerr << "Impossible d'ouvrir la caméra.\n";

    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(9002);

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0;
    listen(serverSocket, 5) < 0;

    std::cout << "En attente de clients VIDEO...\n";

    std::thread(&COM::handleVideoServer, this, std::ref(clientsVideo), std::ref(cap)).detach();
    while (loop)
    {

        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientLen);
        if (clientSocket >= 0)
        {

            std::lock_guard<std::mutex> lockB(clientVideo);
            clientsVideo.push_back(clientSocket);
            std::string header =
                "HTTP/1.0 200 OK\r\n"
                "Server: DIYDroneMJPEG\r\n"
                "Cache-Control: no-cache\r\n"
                "Pragma: no-cache\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

            send(clientSocket, header.c_str(), header.size(), 0);
        }

        if (!loop)
        {
            cap.release();
            close(serverSocket);
            return;
        }
    }
    return;
}

void drawWarning(cv::Mat &frame,
                 const cv::Scalar &bgColor,     // fond
                 const cv::Scalar &borderColor, // bordure
                 const cv::Scalar &textColor)   // texte
{
    // Taille du bloc (moitié largeur, moitié hauteur)
    int w = frame.cols / 2;
    int h = frame.rows / 2;

    // Position pour centrer le rectangle
    int x = (frame.cols - w) / 2;
    int y = (frame.rows - h) / 2;

    cv::Rect zone(x, y, w, h);

    // Bloc opaque
    cv::rectangle(frame, zone, bgColor, cv::FILLED);

    // Bordure
    cv::rectangle(frame, zone, borderColor, 4);

    // Texte
    std::string txt = "WARNING";
    int baseline = 0;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 2.0;
    int thickness = 4;

    cv::Size textSize = cv::getTextSize(txt, fontFace, fontScale, thickness, &baseline);

    cv::Point pos(
        x + (w - textSize.width) / 2,
        y + (h + textSize.height) / 2);

    // Ombre derrière le texte
    cv::putText(frame, txt, pos + cv::Point(3, 3), fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 2);

    // Texte avec couleur paramétrable
    cv::putText(frame, txt, pos, fontFace, fontScale, textColor, thickness);
}

void COM::handleVideoServer(std::list<int> &clientsVideo, cv::VideoCapture &cap)
{
    cv::Mat frame;
    std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 80};

    // Taille cible (format 16:9)
    int targetWidth = 1280;
    int targetHeight = 720;

    int messageInc = 0;
    bool isWarning = false;

    while (loop)
    {
        if (!cap.read(frame))
        {
            // std::cout << "Erreur capture caméra\n";
            continue;
        }

        // Créer un canvas noir (format fixe)
        cv::Mat output(targetHeight, targetWidth, frame.type(), cv::Scalar(0, 0, 0));

        // Rotation dynamique
        cv::Mat rotated;
        switch (dataSystem.gimball.mode)
        {
        // case CAM_STAB: // 90°
        //     cv::rotate(frame, rotated, cv::ROTATE_90_CLOCKWISE);
        //     break;
        // case 2: // 180°
        //     cv::rotate(frame, rotated, cv::ROTATE_180);
        //     break;
        case CAM_STAB: // 270°
            cv::rotate(frame, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        case CAM_LANDING: // 270°
            cv::rotate(frame, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        default:
            rotated = frame;
        }

        // Adapter dans le canvas (ratio conservé)
        double scaleX = (double)targetWidth / rotated.cols;
        double scaleY = (double)targetHeight / rotated.rows;
        double scale = std::min(scaleX, scaleY); // on garde le ratio

        cv::Mat resized;
        cv::resize(rotated, resized, cv::Size(), scale, scale);

        // Calcul position pour centrer
        int x = (targetWidth - resized.cols) / 2;
        int y = (targetHeight - resized.rows) / 2;

        // Copier dans le canvas
        resized.copyTo(output(cv::Rect(x, y, resized.cols, resized.rows)));

        // Ajouter la légende (toujours au même endroit du canvas)
        {
            std::lock_guard<std::mutex> lock(dataMTX);

            char buf[64];
            snprintf(buf, sizeof(buf), "Vbat: %.2f V", dataSystem.vBat);
            cv::putText(output, buf, {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            snprintf(buf, sizeof(buf), "Cap: %.0f deg.", dataSystem.state3D.att[2]);
            cv::putText(output, buf, {600, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            snprintf(buf, sizeof(buf), "Vel: %.2f km/h", sqrt(pow(dataSystem.state3D.vel[0], 2) + pow(dataSystem.state3D.vel[1], 2) + pow(dataSystem.state3D.vel[2], 2)) * 3.6);
            cv::putText(output, buf, {20, 550}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            snprintf(buf, sizeof(buf), "Alt: %.2f m", dataSystem.state3D.pos[2]);
            cv::putText(output, buf, {20, 600}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            snprintf(buf, sizeof(buf), "Nord: %.2f m", dataSystem.state3D.pos[0]);
            cv::putText(output, buf, {20, 650}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            snprintf(buf, sizeof(buf), "Est: %.2f m", dataSystem.state3D.pos[1]);
            cv::putText(output, buf, {20, 700}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

            int cx = frame.cols / 2;
            int cy = frame.rows / 2;
            int crossSize = 20;
            int thickness = 2;
            cv::line(output, cv::Point(cx - crossSize, cy), cv::Point(cx + crossSize, cy), {255, 255, 255}, thickness);
            cv::line(output, cv::Point(cx, cy - crossSize), cv::Point(cx, cy + crossSize), {255, 255, 255}, thickness);

            std::string text = convertToStringAndPrecision(dataSystem.sensor.Tele, 1) + "m";
            cv::Point textPos(cx + 30, cy + crossSize + 30);
            cv::putText(output, text, textPos, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);
        }

        if (isWarning)
        {
            if (messageInc < 2)
            {
                drawWarning(output,
                            cv::Scalar(0, 255, 255),
                            cv::Scalar(255, 255, 255),
                            cv::Scalar(0, 0, 255));

                messageInc++;
            }
            else if (messageInc < 5)
            {
                drawWarning(output,
                            cv::Scalar(0, 255, 255),
                            cv::Scalar(0, 0, 255),
                            cv::Scalar(255, 255, 255));

                messageInc++;
            }
            else
            {
                drawWarning(output,
                            cv::Scalar(0, 255, 255),
                            cv::Scalar(255, 255, 255),
                            cv::Scalar(0, 0, 255));
                messageInc = 1;
                if (launch.event)
                {
                    if (launch.event->hasSeverityOrAbove(severity::WARNING))
                        isWarning = true;
                    else
                        isWarning = false;
                }
            }
        }
        else
        {
            if (launch.event->hasSeverityOrAbove(severity::WARNING))
                isWarning = true;
            else
                isWarning = false;
        }

        // Encoder en JPEG
        std::vector<uchar> jpegBuffer;
        cv::imencode(".jpg", output, jpegBuffer, jpegParams);

        // Préparer header HTTP MJPEG
        std::ostringstream oss;
        oss << "--frame\r\n"
            << "Content-Type: image/jpeg\r\n"
            << "Content-Length: " << jpegBuffer.size() << "\r\n\r\n";

        // Envoi aux clients
        std::list<int> clientsVideoSUP;
        for (int i : clientsVideo)
        {
            if (
                send(i, oss.str().c_str(), oss.str().size(), MSG_NOSIGNAL) < 0 ||
                send(i, reinterpret_cast<char *>(jpegBuffer.data()), jpegBuffer.size(), MSG_NOSIGNAL) < 0 ||
                send(i, "\r\n", 2, MSG_NOSIGNAL) < 0)
                clientsVideoSUP.push_back(i);
        }

        if (!clientsVideoSUP.empty())
            for (int i : clientsVideoSUP)
            {
                close(i);
                clientsVideo.remove(i);
            }

        if (!loop)
        {
            for (int i : clientsVideo)
                close(i);
        }
    }

    for (int i : clientsVideo)
        close(i);
}

// NODE JS SERVER

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

std::string convertToStringAndPrecision(double value, const int precision)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}