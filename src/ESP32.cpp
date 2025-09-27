#include "../inc/ESP32.hpp"
#include "../inc/INS.hpp"
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <cstring>

ESP32::ESP32(eventManager &event, std::optional<INS> &ins, const char *portName)
    : event(event), ins(ins), portName(portName)
{
  std::lock_guard<std::mutex> lock(mtx);
  uint8_t buffer[200];
  int attempt = 0;
  while (SerialPort < 0 && attempt < 50)
  {
    conect();
    if (SerialPort < 0)
    {
      attempt++;
      std::cout << SerialPort << std::endl;
      usleep(10000);
      continue;
    }

    if (readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 200, 50))
    {

      for (int i = 0; i < 199; i++)
      {
        if (buffer[i] == 0x24 && buffer[i + 1] == 0x09)
        {
          event.reportEvent({component::ESP, subcomponent::serial, eventSeverity::INFO, "port serie ouvert"});
          run = std::thread(&ESP32::runESP32, this);
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
  event.reportEvent({component::ESP, subcomponent::serial, eventSeverity::CRITICAL, "impossible d ouvrir le port serie"});
}

ESP32::~ESP32()
{
  loop = false;
  if (run.joinable())
    run.join();
  close(SerialPort);
}

bool ESP32::conect()
{
  SerialPort = open(portName, O_RDWR | O_NOCTTY);
  memset(&tty, 0, sizeof tty);
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

bool readSafe(int fd, uint8_t *output, size_t size, int timeoutMs)
{
  size_t totalRead = 0;
  auto startTime = std::chrono::steady_clock::now();

  while (totalRead < size)
  {
    // Calcul du temps restant avant timeout
    auto now = std::chrono::steady_clock::now();
    int elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    int timeLeft = timeoutMs - elapsedMs;
    if (timeLeft <= 0)
      return false; // Timeout dépassé

    // Préparation de la structure timeval pour select
    struct timeval tv;
    tv.tv_sec = timeLeft / 1000;
    tv.tv_usec = (timeLeft % 1000) * 1000;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    // Attend que fd soit prêt en lecture
    int ret = select(fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0)
    {
      // Erreur select
      return false;
    }
    else if (ret == 0)
    {
      // Timeout sur select
      return false;
    }

    // fd prêt à la lecture
    ssize_t n = read(fd, output + totalRead, size - totalRead);
    if (n < 0)
    {
      return false; // Erreur read
    }
    else if (n == 0)
    {
      // EOF (ou rien à lire) — optionnel: faire une petite pause ?
      usleep(1000);
      continue;
    }

    totalRead += n;
  }

  return true;
}

void ESP32::runESP32()
{

  while (loop)
  {
    ESPdata dataBuffer;
    uint8_t buffer[2] = {0x00, 0x00};

    bool n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 2);

    if (n != true)
      continue;

    if (buffer[0] == 0x24 && buffer[1] == 0x09)
    {
      n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&dataBuffer), 97);
      n = readSafe(SerialPort, reinterpret_cast<uint8_t *>(&buffer), 2);
      if (n != true)
        continue;
      if (buffer[0] == 0x20 && buffer[1] == 0x08)
      {
        {
          std::lock_guard<std::mutex> lock(mtx);
          data = dataBuffer;
        }
        switch (data.event)
        {
        case 0x01:
        {
          const uint8_t data_[1] = {0x02};
          write(SerialPort, data_, 1);
          break;
        }

        case 0x02:
        {
          event.reportEvent({component::ESP, subcomponent::parser, eventSeverity::INFO, "reception d un paquet"});
          if (ins)
            ins->updateMPU(data);
          std::cout << data.distances[0] << "\n";
          break;
        }
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