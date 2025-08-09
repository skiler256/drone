#include "BMP280.hpp"

BMP280::BMP280(eventManager &event, uint8_t address, const char *bus)
    : event(event), file(-1), addr(address)
{
  std::lock_guard<std::mutex> lock(mtx);
  file = open(bus, O_RDWR);
  if (ioctl(file, I2C_SLAVE, addr) < 0)
  {
    event.reportEvent({component::BMP, subcomponent::i2c, eventSeverity::CRITICAL, "impossible d ouvrir port i2c"});
    close(file);
    file = -1;
    return;
  }
  else
    event.reportEvent({component::BMP, subcomponent::i2c, eventSeverity::INFO, "port i2c ouvert"});

  uint8_t reg = COEF;
  write(file, &reg, 1);
  usleep(1000);

  constexpr size_t CALIB_SIZE = 24;
  char calib[CALIB_SIZE];
  read(file, calib, CALIB_SIZE);

  dig_T1 = (uint16_t)((uint8_t)calib[1] << 8) | (uint8_t)calib[0];
  dig_T2 = (int16_t)((uint8_t)calib[3] << 8) | (uint8_t)calib[2];
  dig_T3 = (int16_t)((uint8_t)calib[5] << 8) | (uint8_t)calib[4];
  dig_P1 = (uint16_t)((uint8_t)calib[7] << 8) | (uint8_t)calib[6];
  dig_P2 = (int16_t)((uint8_t)calib[9] << 8) | (uint8_t)calib[8];
  dig_P3 = (int16_t)((uint8_t)calib[11] << 8) | (uint8_t)calib[10];
  dig_P4 = (int16_t)((uint8_t)calib[13] << 8) | (uint8_t)calib[12];
  dig_P5 = (int16_t)((uint8_t)calib[15] << 8) | (uint8_t)calib[14];
  dig_P6 = (int16_t)((uint8_t)calib[17] << 8) | (uint8_t)calib[16];
  dig_P7 = (int16_t)((uint8_t)calib[19] << 8) | (uint8_t)calib[18];
  dig_P8 = (int16_t)((uint8_t)calib[21] << 8) | (uint8_t)calib[20];
  dig_P9 = (int16_t)((uint8_t)calib[23] << 8) | (uint8_t)calib[22];

  uint8_t cfg1[] = {
      CTRL_MEAS,
      (0b101 << 5) | (0b101 << 2) | 0b11};
  write(file, cfg1, sizeof(cfg1));
  usleep(1000);

  uint8_t cfg2[] = {
      CONFIG,
      (0b000 << 5) | (0b000 << 2) // t_sb=000 (0.5ms), filter=000 (off)
  };
  write(file, cfg2, sizeof(cfg2));
  usleep(1000);
}

BMP280::~BMP280()
{
  std::lock_guard<std::mutex> lock(mtx);
  if (file >= 0)
  {
    close(file);
  }
}

BMP280::Data BMP280::getData()
{
  std::lock_guard<std::mutex> lock(mtx);

  if (file < 0)
  {
    return {0.0, 0.0};
  }

  uint8_t regT = TEMP_REG;
  write(file, &regT, 1);
  usleep(1000);

  char dataT[3];
  read(file, dataT, sizeof(dataT));
  int32_t adc_T = ((int32_t)dataT[0] << 12) |
                  ((int32_t)dataT[1] << 4) |
                  ((int32_t)dataT[2] >> 4);

  uint8_t regP = PRESS_REG;
  write(file, &regP, 1);
  usleep(1000);

  char dataP[3];
  read(file, dataP, sizeof(dataP));
  int32_t adc_P = ((int32_t)dataP[0] << 12) |
                  ((int32_t)dataP[1] << 4) |
                  ((int32_t)dataP[2] >> 4);

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) *
                  (int32_t)dig_T3) >>
                 14;
  t_fine = var1 + var2;
  double temperature = (t_fine * 5 + 128) / 25600.0;

  int64_t v1 = (int64_t)t_fine - 128000;
  int64_t v2 = v1 * v1 * (int64_t)dig_P6;
  v2 += ((v1 * (int64_t)dig_P5) << 17);
  v2 += ((int64_t)dig_P4 << 35);
  int64_t p_acc = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
  p_acc = ((((int64_t)1 << 47) + p_acc) * (int64_t)dig_P1) >> 33;

  double pressure;
  if (p_acc == 0)
  {
    pressure = 0; // protection div0
  }
  else
  {
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / p_acc;
    v1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4);
    pressure = (double)p / 25600.0;
  }

  if (temperature > 50 || temperature < -10 || pressure < 900 || pressure > 1100)
  {
    event.reportEvent({component::BMP, subcomponent::computing, eventSeverity::CRITICAL,
                       "valeur calculee anormale"});
  }
  else
  {
    event.reportEvent({component::BMP, subcomponent::computing, eventSeverity::INFO,
                       "valeur calculee normale"});
  }

  return {temperature, pressure};
}