#include "../inc/ESP32.hpp"
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

ESP32::ESP32(eventManager &event, const char *portName)
    : event(event), portName(portName)
{

  uint8_t buffer[200];
  int attempt = 0;
  while (SerialPort < 0 && attempt < 50)
  {
    conect();
    if (SerialPort < 0)
    {
      usleep(10000);
      continue;
    }
    if (readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 200, 50))
    {
      for (int i = 0; i < 199; i++)
      {
        if (buffer[i] == 0x24 && buffer[i + 1] == 0x09)
        {

          return;
        }
      }
      if (SerialPort >= 0)
      {
        close(SerialPort);
        SerialPort = -1;
      }
    }
    else
    {
      if (SerialPort >= 0)
      {
        close(SerialPort);
        SerialPort = -1;
        usleep(10000);
      }
    }
    attempt++;
  }
}

bool ESP32::conect()
{
  SerialPort = open(portName, O_RDWR | O_NOCTTY);
  if (tcgetattr(SerialPort, &tty) != 0)
    return false;

  cfsetospeed(&tty, B460800);
  cfsetispeed(&tty, B460800);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  tcsetattr(SerialPort, TCSANOW, &tty);
  return true;
}

bool readSafe(int fd, uint8_t *output, size_t size, int timeout)
{
  auto t1 = std::chrono::steady_clock::now();
  size_t totalRead = 0;
  while (totalRead < size &&
         std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - t1)
                 .count() < timeout)
  {
    ssize_t n = read(fd, output + totalRead, size - totalRead);
    if (n < 0)
    {
      return false;
    }
    else if (n == 0)
    {
      usleep(1000);
      continue;
    }

    totalRead += n;
  }
  return true;
}

void ESP32::handleESP32()
{

  while (true)
  {
    ESPdata dataBuffer;
    uint8_t buffer[2] = {0x00, 0x00};

    bool n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 2);

    if (n != true)
      continue;

    if (buffer[0] == 0x24 && buffer[1] == 0x09)
    {
      n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&dataBuffer), 73);
      n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 2);
      if (n != true)
        continue;
      if (buffer[0] == 0x20 && buffer[1] == 0x08)
      {
        {
          std::lock_guard<std::mutex> lock(mtx);
          data = dataBuffer;
        }
        std::cout << (int)data.event << std::endl;
        switch (data.event)
        {
        case 0x01:
          const uint8_t message[1] = {0x02};
          write(SerialPort, message, 2);
          break;

        case 0x02:
          break;
        }
      }
    }
  }
}

ESPdata ESP32::getData()
{
  std::lock_guard<std::mutex> lock(mtx);
  return data;
}