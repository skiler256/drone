#include "../inc/BMP280.hpp"

BMP280::BMP280(eventManager &event, uint8_t address, const char *bus) : event(event), addr(address)
{
  file = open(bus, O_RDWR);
  if (file < 0)
    if (ioctl(file, I2C_SLAVE, addr) < 0)
    {
      event.reportEvent({component::BMP, subcomponent::i2c, eventSeverity::CRITICAL, "impossible d ouvrir le port i2c "});
    }
  write(file, &COEF, 1);
  char calib[24];
  read(file, calib, 24);

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

  // Configuration du capteur (oversampling x16 temp, x16 press, mode normal)
  char config[2] = {
      CTRL_MEAS,
      0b10110111}; // osrs_t = 101 (x16), osrs_p = 101 (x16), mode = 11 (normal)
  write(file, config, 2);
}

BMP280::~BMP280()
{
  if (file >= 0)
    close(file);
}

BMP280::data BMP280::getData()
{
  std::lock_guard<std::mutex> lock(mtx);
  // Lecture température brute
  write(file, &TEMP, 1);
  char dataT[3];
  read(file, dataT, 3);
  int32_t adc_T = ((int32_t)dataT[0] << 12) | ((int32_t)dataT[1] << 4) |
                  ((int32_t)dataT[2] >> 4);

  // Lecture pression brute
  write(file, &PRESS, 1);
  char dataP[3];
  read(file, dataP, 3);
  int32_t adc_P = ((int32_t)dataP[0] << 12) | ((int32_t)dataP[1] << 4) |
                  ((int32_t)dataP[2] >> 4);

  // Compensation température
  int32_t var1 =
      (((adc_T >> 3) - ((int32_t)dig_T1 << 1)) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 =
      (((((adc_T >> 4) - (int32_t)dig_T1) * ((adc_T >> 4) - (int32_t)dig_T1)) >>
        12) *
       (int32_t)dig_T3) >>
      14;
  t_fine = var1 + var2;
  float temp = (t_fine * 5 + 128) / 25600.0;

  // Compensation pression
  int64_t var1p = ((int64_t)t_fine) - 128000;
  int64_t var2p = var1p * var1p * (int64_t)dig_P6;
  var2p += ((var1p * (int64_t)dig_P5) << 17);
  var2p += (((int64_t)dig_P4) << 35);
  var1p = (((var1p * var1p * (int64_t)dig_P3) >> 8) +
           ((var1p * (int64_t)dig_P2) << 12));
  var1p = (((((int64_t)1) << 47) + var1p)) * ((int64_t)dig_P1) >> 33;

  float pressure;
  if (var1p == 0)
  {
    pressure = 0; // division par zéro protection
  }
  else
  {
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2p = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1p + var2p) >> 8) + (((int64_t)dig_P7) << 4);
    pressure = (float)p / 25600.0; // en hPa
  }

  if (temp > 50 || temp < -10 || pressure < 900 || pressure > 1100)
    event.reportEvent({component::BMP, subcomponent::computing, eventSeverity::CRITICAL, "valeur calculee anormale"});

  else
    event.reportEvent({component::BMP, subcomponent::computing, eventSeverity::INFO, "valeur calculee normale"});

  return {(double)temp, (double)pressure};
}
