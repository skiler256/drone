#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/eventManager.hpp"
#include "../inc/INS.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/PCA9685.hpp"

#include <thread>

int main()
{
  eventManager event;
  ESP32 esp(event);
  NEO6m gps(event);
  BMP280 baro(event);

  INS::settings INSsettings;
  INSsettings.alphaHeading = 0.5;
  INSsettings.refreshRate = 200;
  INS ins(event, esp, baro, gps, INSsettings);

  std::thread a(&ESP32::handleESP32, &esp);
  std::thread b(&NEO6m::handlNEO6m, &gps);
  std::thread c(&INS::runINS, &ins);

  while (true)
  {
    // ins.printData();
    std::cout << "baro mise a jour " << std::endl;
    std::cout << baro.getData().temp << std::endl;
    usleep(100000);
  }

  a.join();
  b.join();
  c.join();
  return 0;
}