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
  event.doLog = true;
  ESP32 esp(event);
  NEO6m gps(event);
  PCA9685 pca(event);
  BMP280 baro(event);

  INS::settings INSsettings;
  INSsettings.alphaHeading = 0.5;
  INSsettings.refreshRate = 200;
  // INS ins(event, esp, baro, gps, INSsettings);

  std::thread a(&ESP32::runESP32, &esp);
  std::thread b(&NEO6m::runNEO6m, &gps);
  // std::thread c(&INS::runINS, &ins);
  std::thread d(&PCA9685::runPCA9685, &pca);
  while (true)
  {
    pca.setPWM(1, 50);
    std::cout << baro.getData().temp << std::endl;
    std::cout << "boucle" << std::endl;
    usleep(100000);
  }

  a.join();
  b.join();
  // c.join();
  d.join();
  return 0;
}

// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #include <iostream>
// #include <cstring>
// #include <errno.h>

// int main(int argc, char *argv[])
// {
//   const char *portName = argv[1];
//   int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);

//   // Configuration du port série
//   struct termios tty;
//   memset(&tty, 0, sizeof tty);
//   tcgetattr(fd, &tty);

//   cfsetospeed(&tty, B460800);
//   cfsetispeed(&tty, B460800);

//   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
//   tty.c_cflag &= ~PARENB;                     // pas de parité
//   tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
//   tty.c_cflag |= CREAD | CLOCAL;              // activer la lecture, ignorer modem control lines

//   tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
//   tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // désactiver contrôle logiciel
//   tty.c_oflag &= ~OPOST;                          // raw output

//   tty.c_cc[VMIN] = 0;   // lecture non bloquante
//   tty.c_cc[VTIME] = 10; // timeout lecture (1 sec)

//   tcsetattr(fd, TCSANOW, &tty);

//   std::cout << "Port " << portName << " ouvert et configuré. Lecture des données..." << std::endl;

//   uint8_t buf[256];
//   while (true)
//   {
//     ssize_t n = read(fd, buf, sizeof(buf));
//     if (n < 0)
//     {
//       std::cerr << "Erreur lecture: " << strerror(errno) << std::endl;
//       break;
//     }
//     else if (n == 0)
//     {
//       // Timeout, pas de données
//       continue;
//     }

//     std::cout << "Données reçues (" << n << " octets): ";
//     for (ssize_t i = 0; i < n; ++i)
//     {
//       std::cout << std::hex << (int)buf[i] << " ";
//     }
//     std::cout << std::dec << std::endl;
//   }

//   close(fd);
//   return 0;
// }
