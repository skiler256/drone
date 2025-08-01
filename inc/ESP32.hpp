#pragma once
#include "../inc/eventManager.hpp"
#include <fcntl.h>
#include <mutex>
#include <termios.h>

#pragma pack(push, 1) // désactive l'alignement mémoire
struct ESPdata
{
  uint8_t event = 0x00;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double mx = 0.0, my = 0.0, mz = 0.0;
};
#pragma pack(pop)

bool readSafe(int fd, uint8_t *output, size_t size, int timeout = 10);

class ESP32
{
public:
  ESP32(eventManager &event, const char *portName = "/dev/ttyAMA0");

  void handleESP32();

  ESPdata getData();

private:
  ESPdata data;
  eventManager &event;
  int SerialPort = -1;
  const char *portName;
  termios tty;

  std::mutex mtx;

  bool conect();
};