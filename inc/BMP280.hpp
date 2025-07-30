#pragma once
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

const uint8_t CTRL_MEAS = 0xF4;
const uint8_t TEMP = 0xFA;
const uint8_t PRESS = 0xF7;
const uint8_t COEF = 0x88;

class BMP280 {
public:
  BMP280(uint8_t address = 0x76, const char *bus = "/dev/i2c-1");
  ~BMP280();

  struct data {
    double temp;
    double press;
  };

  data getData();

private:
  int file;
  uint8_t addr;
  int32_t t_fine;

  // Coefficients de calibration
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
};
