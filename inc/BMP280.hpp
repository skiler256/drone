#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "eventManager.hpp"

class BMP280
{
public:
  struct Data
  {
    double temperature;
    double pressure;
  };

  BMP280(eventManager &event, uint8_t address = 0x76, const char *bus = "/dev/i2c-1");
  ~BMP280();

  Data getData();

private:
  eventManager &event;
  int file;
  uint8_t addr;
  std::mutex mtx;

  // Calibration coefficients
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  int32_t t_fine;

  static constexpr uint8_t COEF = 0x88; // premier registre de calibration
  static constexpr uint8_t CTRL_MEAS = 0xF4;
  static constexpr uint8_t CONFIG = 0xF5;
  static constexpr uint8_t TEMP_REG = 0xFA;
  static constexpr uint8_t PRESS_REG = 0xF7;
};