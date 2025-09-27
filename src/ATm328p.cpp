#include "../inc/ATm328p.hpp"
#include "../inc/sysMonitoring.hpp"
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

ATm328p::ATm328p(eventManager &event, std::optional<sysMonitoring> &monitoring, const char *portName)
    : event(event),
      portName(portName),
      SerialPort(open(portName, O_RDWR | O_NOCTTY | O_SYNC)),
      monitoring(monitoring)
{
    //     if (tcgetattr(SerialPort, &tty) < 0)
    //     event.reportEvent({component::GPS, subcomponent::serial, eventSeverity::CRITICAL, "impossible d ouvrir le port serie"});
    //   else
    //     event.reportEvent({component::GPS, subcomponent::serial, eventSeverity::INFO, "port serie ouvert"});

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

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

    Tx = std::thread(&ATm328p::runTx, this);
}
ATm328p::~ATm328p()
{
    loopTx = false;
    if (Tx.joinable())
        Tx.join();
    close(SerialPort);
}

void ATm328p::runTx()
{

    while (loopTx)
    {
        const auto start = std::chrono::steady_clock::now();
        if (monitoring)
        {
            auto data = monitoring->getTelemetryDataRef();

            // {
            //     uint8_t mess[] = {0x24, 0x09};
            //     write(SerialPort, mess, 2);

            // }

            // uint8_t buff[64];
            // std::memcpy(buff, &data.p1, sizeof(sysMonitoring::telP1));
            // buff[63] = 0x01;

            // for (int i = 0; i < 64; i++)
            // {
            //     std::cout << std::hex << std::uppercase
            //               << std::setw(2) << std::setfill('0')
            //               << static_cast<int>(buff[i]) << " ";

            //     if ((i + 1) % 16 == 0)
            //         std::cout << "\n"; // retour ligne toutes les 16 cases
            // }

            // write(SerialPort, buff, 64);
            // tcdrain(SerialPort);

            // {
            //     uint8_t mess[] = {0x20, 0x08};
            //     write(SerialPort, mess, 2);
            //     tcdrain(SerialPort);
            // }

            uint8_t packet[68];

            packet[0] = 0x24;
            packet[1] = 0x09;

            std::memcpy(packet + 2, &data.p1, sizeof(sysMonitoring::telP1));
            packet[59] = 0x01;

            packet[61] = 0x20;
            packet[62] = 0x08;

            // for (int i = 0; i < 63; i++)
            // {
            //     std::cout << std::hex << std::uppercase
            //               << std::setw(2) << std::setfill('0')
            //               << static_cast<int>(packet[i]) << " ";
            //     if ((i + 1) % 16 == 0)
            //         std::cout << "\n";
            // }

            write(SerialPort, packet, 63);
            tcdrain(SerialPort);
        }
        const auto end = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int target_period_ms = static_cast<int>(1000.0);
        int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
    }
}