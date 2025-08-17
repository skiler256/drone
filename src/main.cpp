// #include "../inc/BMP280.hpp"
// #include "../inc/ESP32.hpp"
// #include "../inc/eventManager.hpp"
// #include "../inc/INS.hpp"
// #include "../inc/NEO6m.hpp"
// #include "../inc/PCA9685.hpp"
// #include "../inc/sysMonitoring.hpp"
// #include "../inc/COM.hpp"
// #include "../inc/gpio.hpp"
// #include "../inc/gimball.hpp"

// #include <thread>
// #include <iostream>
// #include <signal.h>
// #include <fstream>

// void handle_sigint(int signum)
// {
//   killNodeJS();
//   std::cout << "\nArret du programme" << std::endl;
//   exit(0);
// }

// int main()
// {
//   signal(SIGINT, handle_sigint);

//   eventManager event;
//   event.doLog = false;
//   // ESP32 esp(event);
//   // NEO6m gps(event);
//   // PCA9685 pca(event);
//   // BMP280 baro(event);

//   // std::thread a(&ESP32::runESP32, &esp);
//   // std::thread b(&NEO6m::runNEO6m, &gps);
//   // std::thread d(&PCA9685::runPCA9685, &pca);

//   // INS::settings INSsettings;
//   // INSsettings.alphaHeading = 0.5;
//   // INSsettings.refreshRate = 200;
//   // INSsettings.NmoyGPScalib = 100;
//   // INSsettings.baseAltitude = 120;
//   // INS ins(event, esp, baro, gps, INSsettings);
//   // std::thread c(&INS::runINS, &ins);

//   sysMonitoring monitoring(event, esp, baro, gps, ins, 10);
//   COM com(monitoring, 5);

//   std::thread e(&sysMonitoring::runSysMonitoring, &monitoring);
//   std::thread f(&COM::runCOM, &com);
// std::fstream dataLOG("/home/jules/code/pos.txt");

// GPIO gpio;
// GIMBALL gimball(event)

// // double x=0;
// // double y=0;
// // double z=0;

// // for(int i = 0; i< 120; i++){
// //   INS::state3D state = ins.getState3D();
// //     dataLOG << state.pos(0) << " " << state.pos(1) << state.pos(2) << std::endl;
// //     x += state.pos(0);
// //     y += state.pos(1);
// //     z += state.pos(2);
// //     std::cout<< "coutjd " << i<<std::endl;
// //     usleep(1000000);
// // }

// // double X = x/120;
// // double Y = y/120;
// // double Z = z/120;

// // std::cout<<X<<" "<< Y<< " "<< Z << std::endl;

//   // while (true)
//   // {
//   //   // pca.setPWM(1, 50);
//   //   // std::cout << baro.getData().pressure << std::endl;
//   //   // std::cout << "boucle" << std::endl;
//   // }

//   a.join();
//   b.join();
//   c.join();
//   d.join();
//   e.join();
//   f.join();
//   return 0;
// }

#include "../inc/behaviorCenter.hpp"

#include <signal.h>

void handle_sigint(int signum)
{
  killNodeJS();
  std::cout << "\nArret du programme" << std::endl;
  exit(0);
}

int main()
{
  signal(SIGINT, handle_sigint);
  behaviorCenter behavior;

  while (true)
  {
    usleep(1000000);
  }

  return 0;
}

// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <vector>
// #include <thread>
// #include <string>
// #include <sstream>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <unistd.h>

// const int PORT = 8080;

// void clientHandler(int clientSocket, cv::VideoCapture &cap)
// {
//   std::string header =
//       "HTTP/1.0 200 OK\r\n"
//       "Server: DIYDroneMJPEG\r\n"
//       "Cache-Control: no-cache\r\n"
//       "Pragma: no-cache\r\n"
//       "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

//   send(clientSocket, header.c_str(), header.size(), 0);

//   cv::Mat frame;
//   double vbat = 15.2; // exemple de variable
//   std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 80};

//   while (true)
//   {
//     if (!cap.read(frame))
//     {
//       std::cerr << "Erreur lecture frame\n";
//       break;
//     }

//     // Ajouter légende
//     cv::putText(frame, "DIY Drone", {20, 40},
//                 cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
//     char buf[64];
//     snprintf(buf, sizeof(buf), "Vbat: %.2f V", vbat);
//     cv::putText(frame, buf, {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 255, 255}, 2);

//     // Encoder en JPEG
//     std::vector<uchar> jpegBuffer;
//     cv::imencode(".jpg", frame, jpegBuffer, jpegParams);

//     // Envoyer la frame au client
//     std::ostringstream oss;
//     oss << "--frame\r\n"
//         << "Content-Type: image/jpeg\r\n"
//         << "Content-Length: " << jpegBuffer.size() << "\r\n\r\n";
//     send(clientSocket, oss.str().c_str(), oss.str().size(), 0);
//     send(clientSocket, reinterpret_cast<char *>(jpegBuffer.data()), jpegBuffer.size(), 0);
//     send(clientSocket, "\r\n", 2, 0);
//   }

//   close(clientSocket);
// }

// int main()
// {
//   // Pipeline GStreamer pour caméra NV12
//   std::string pipeline =
//       "libcamerasrc ! "
//       "video/x-raw,width=1920,height=1080,format=NV12,framerate=20/1 ! "
//       "videoconvert ! video/x-raw,format=BGR ! appsink";

//   cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
//   if (!cap.isOpened())
//   {
//     std::cerr << "Impossible d'ouvrir la caméra.\n";
//     return 1;
//   }

//   std::cout << "Caméra ouverte ! MJPEG serveur sur port " << PORT << "\n";

//   // Créer socket serveur
//   int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
//   if (serverSocket == -1)
//   {
//     perror("socket");
//     return 1;
//   }

//   sockaddr_in serverAddr{};
//   serverAddr.sin_family = AF_INET;
//   serverAddr.sin_addr.s_addr = INADDR_ANY;
//   serverAddr.sin_port = htons(PORT);

//   int opt = 1;
//   setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

//   if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
//   {
//     perror("bind");
//     return 1;
//   }
//   if (listen(serverSocket, 5) < 0)
//   {
//     perror("listen");
//     return 1;
//   }

//   std::cout << "En attente de clients...\n";

//   while (true)
//   {
//     sockaddr_in clientAddr{};
//     socklen_t clientLen = sizeof(clientAddr);
//     int clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientLen);
//     if (clientSocket >= 0)
//     {
//       std::thread(clientHandler, clientSocket, std::ref(cap)).detach();
//     }
//   }

//   cap.release();
//   close(serverSocket);
//   return 0;
// }
