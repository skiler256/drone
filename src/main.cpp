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
behaviorCenter behavior;

void handle_sigint(int signum)
{
  behavior.save();
  killNodeJS();
  std::cout << "\nArret du programme" << std::endl;
  exit(0);
}

int main()
{
  signal(SIGINT, handle_sigint);
  behavior;

  while (true)
  {
    usleep(1000000);
  }

  return 0;
}

// #include "../inc/behaviorCenter.hpp"
// #include <fstream>
// #include <iostream>
// #include <unistd.h> // pour usleep

// int main()
// {
//   std::ofstream file("/home/jules/mesure.txt", std::ios::out | std::ios::trunc);
//   if (!file.is_open())
//   {
//     std::cerr << "Erreur : impossible d'ouvrir le fichier !" << std::endl;
//     return 1;
//   }

//   eventManager event;
//   ESP32 esp(event);

//   while (true)
//   {
//     std::cout << "Acquisition de 500 échantillons..." << std::endl;
//     for (int i = 0; i < 500; i++)
//     {
//       ESPdata data = esp.getData();
//       file << data.ax << " " << data.ay << " " << data.az << std::endl;
//       usleep(5000); // 5 ms -> 200 Hz
//     }

//     file << "# --- Fin d'une série ---" << std::endl;
//     file.flush();

//     std::cout << "Série terminée. Appuyez sur [Entrée] pour continuer, ou 'q' + [Entrée] pour quitter." << std::endl;
//     std::string input;
//     std::getline(std::cin, input);
//     if (input == "q" || input == "Q")
//     {
//       break;
//     }
//   }

//   file.close();
//   std::cout << "Fin du programme." << std::endl;
//   return 0;
// }
