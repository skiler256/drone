#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"

#include <thread>
#include <iostream>

int main()
{
  eventManager event;
  event.doLog = true;
  ESP32 esp(event);
  NEO6m gps(event);
  PCA9685 pca(event);
  BMP280 baro(event);

  INS::settings INSsettings;
  INSsettings.alphaHeading = 0.5;
  INSsettings.refreshRate = 200;
  INS ins(event, esp, baro, gps, INSsettings);

  std::thread a(&ESP32::runESP32, &esp);
  std::thread b(&NEO6m::runNEO6m, &gps);
  std::thread c(&INS::runINS, &ins);
  std::thread d(&PCA9685::runPCA9685, &pca);
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
  return 0;
}
