#include "../inc/PCA9685.hpp"
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

PCA9685::PCA9685(eventManager &event, const uint8_t PCA9685_addr,
                 const char *bus)
    : event(event), file(open(bus, O_RDWR))
{
  //   std::lock_guard<std::mutex> lock(mtx);

  //   if (ioctl(file, I2C_SLAVE, PCA9685_addr) != 1)

  //     else

  //         char config[2] = {MODE1, 0b00100001};
  //   write(file, config, 2);

  //   std::fill(std::begin(OUTPUTregister), std::end(OUTPUTregister), 0);

  //   OUTPUTregister[0] = 0x06;

  //   usleep(100000);
}

PCA9685::~PCA9685()
{
  std::lock_guard<std::mutex> lock(mtx);
  close(file);
  loop = false;
  std::lock_guard<std::mutex> lockb(mtxloop);
}

void PCA9685::runPCA9685()
{

  while (loop)
  {
    std::lock_guard<std::mutex> lockb(mtxloop);
    {
      std::lock_guard<std::mutex> lock(mtx);

      write(file, OUTPUTregister, 33);
    }

    usleep(50000);
    if (!loop)
      return;
  }
}

void PCA9685::setPWM(const int output, double pwm)
{
  std::lock_guard<std::mutex> lock(mtx);

  if (pwm < 0.0)
    pwm = 0.0;
  if (pwm > 100.0)
    pwm = 100.0;

  uint16_t ontick = 0;

  uint16_t offtick = static_cast<uint16_t>((pwm / 100.0) * 4096);

  if (offtick >= 4096)
    offtick = 4095;

  OUTPUTregister[1 + output * 4] = ontick & 0xFF;
  OUTPUTregister[2 + output * 4] = (ontick >> 8) & 0x0F;
  OUTPUTregister[3 + output * 4] = offtick & 0xFF;
  OUTPUTregister[4 + output * 4] = (offtick >> 8) & 0x0F;
}