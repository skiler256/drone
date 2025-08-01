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

  std::thread a(&ESP32::handleESP32, &esp);

  a.join();
  return 0;
}