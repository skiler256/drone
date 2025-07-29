#pragma once
#include "../inc/eventManager.hpp"
#include <cstdint>
#include <mutex>

class PCA9685 {
public:
  PCA9685(eventManager &event, const uint8_t PCA9685_addr = 0x70,
          const char *bus = "/dev/i2c-1");
  ~PCA9685();

  void handlePCA9685();

  void setPWM(const int output, double pwm);

private:
  eventManager &event;
  int file;

  std::mutex mtx;

  const uint8_t MODE1 = 0x00;

  uint8_t OUTPUTregister[33] = {0};
};