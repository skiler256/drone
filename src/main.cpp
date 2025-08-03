#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"
#include "../inc/sysMonitoring.hpp"
#include "../inc/COM.hpp"

#include <thread>
#include <iostream>
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

  eventManager event;
  event.doLog = false;
  ESP32 esp(event);
  NEO6m gps(event);
  PCA9685 pca(event);
  BMP280 baro(event);

  INS::settings INSsettings;
  INSsettings.alphaHeading = 0.5;
  INSsettings.refreshRate = 200;
  INS ins(event, esp, baro, gps, INSsettings);

  sysMonitoring monitoring(event, esp, baro, gps, ins, 10);
  COM com(monitoring, 5);

  std::thread a(&ESP32::runESP32, &esp);
  std::thread b(&NEO6m::runNEO6m, &gps);
  std::thread c(&INS::runINS, &ins);
  std::thread d(&PCA9685::runPCA9685, &pca);
  std::thread e(&sysMonitoring::runSysMonitoring, &monitoring);
  std::thread f(&COM::runCOM, &com);
  while (true)
  {
    pca.setPWM(1, 50);
    std::cout << baro.getData().pressure << std::endl;
    std::cout << "boucle" << std::endl;
    usleep(1000000);
  }

  a.join();
  b.join();
  c.join();
  d.join();
  e.join();
  f.join();
  return 0;
}

// #include "/usr/local/include/uWebSockets/App.h"
// #include <iostream>

// int main()
// {
//   /* ws->getUserData returns one of these */
//   struct data
//   {
//   };

//   /* Keep in mind that uWS::SSLApp({options}) is the same as uWS::App() when compiled without SSL support.
//    * You may swap to using uWS:App() if you don't need SSL */
//   uWS::App({/* There are example certificates in uWebSockets.js repo */
//             .key_file_name = "misc/key.pem",
//             .cert_file_name = "misc/cert.pem",
//             .passphrase = "1234"})
//       .ws<data>("/*", {/* Settings */
//                        .compression = uWS::CompressOptions(uWS::DEDICATED_COMPRESSOR | uWS::DEDICATED_DECOMPRESSOR),
//                        .maxPayloadLength = 100 * 1024 * 1024,
//                        .idleTimeout = 16,
//                        .maxBackpressure = 100 * 1024 * 1024,
//                        .closeOnBackpressureLimit = false,
//                        .resetIdleTimeoutOnSend = false,
//                        .sendPingsAutomatically = true,
//                        /* Handlers */
//                        .upgrade = nullptr,
//                        .open = [](auto *ws)
//                        {
//                                   std::cout<< ws << std::endl;
//                                   /* Open event here, you may access ws->getUserData() which points to a PerSocketData struct */ },
//                        .message = [](auto *ws, std::string_view message, uWS::OpCode opCode)
//                        {
//                                   /* This is the opposite of what you probably want; compress if message is LARGER than 16 kb
//                                    * the reason we do the opposite here; compress if SMALLER than 16 kb is to allow for
//                                    * benchmarking of large message sending without compression */

//                                   /* Never mind, it changed back to never compressing for now */
//                                   std::string mess = "ta geule";
//                                   ws->send(mess, opCode, false); },
//                        .dropped = [](auto * /*ws*/, std::string_view /*message*/, uWS::OpCode /*opCode*/)
//                        {
//           /* A message was dropped due to set maxBackpressure and closeOnBackpressureLimit limit */ },
//                        .drain = [](auto * /*ws*/)
//                        {
//           /* Check ws->getBufferedAmount() here */ },
//                        .ping = [](auto * /*ws*/, std::string_view)
//                        {
//           /* Not implemented yet */ },
//                        .pong = [](auto * /*ws*/, std::string_view)
//                        {
//           /* Not implemented yet */ },
//                        .close = [](auto * /*ws*/, int /*code*/, std::string_view /*message*/)
//                        {
//           /* You may access ws->getUserData() here */ }})
//       .listen(9001, [](auto *listen_socket)
//               {
//       if (listen_socket) {
//           std::cout << "Listening on port " << 9001 << std::endl;
//       } })
//       .run();
// }