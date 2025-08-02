#include "../inc/NEO6m.hpp"
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

NEO6m::NEO6m(eventManager &event, const char *portName)
    : event(event),
      SerialPort(open(portName, O_RDWR | O_NOCTTY)), portName(portName)
{
  tcgetattr(SerialPort, &tty);

  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);

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

  const uint8_t disable_gga[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x00, 0x24};

  const uint8_t disable_gll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x01, 0x2B};

  const uint8_t disable_gsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x02, 0x32};

  const uint8_t disable_gsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x03, 0x39};

  const uint8_t disable_rmc[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x04, 0x40};

  const uint8_t disable_vtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x01, 0x05, 0x47};

  const uint8_t enable_posllh[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                   0x01, 0x02, 0x00, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x13, 0xBE};

  const uint8_t enable_status[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                   0x01, 0x03, 0x00, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x14, 0xC5};

  const uint8_t enable_velned[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                   0x01, 0x12, 0x00, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x23, 0x2E};

  const uint8_t enable_sol[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06,
                                0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDA};

  const uint8_t enable_timeutc[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                    0x01, 0x21, 0x00, 0x01, 0x00, 0x00,
                                    0x00, 0x00, 0x32, 0x97};

  const uint8_t enable_svinfo[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                   0x01, 0x30, 0x00, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x41, 0x00};

  const uint8_t airborne1g[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
      0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};

  const uint8_t set_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4,
                              0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};

  write(SerialPort, disable_gga, sizeof(disable_gga));
  tcdrain(SerialPort);

  write(SerialPort, disable_gll, sizeof(disable_gll));
  tcdrain(SerialPort);

  write(SerialPort, disable_gsa, sizeof(disable_gsa));
  tcdrain(SerialPort);

  write(SerialPort, disable_gsv, sizeof(disable_gsv));
  tcdrain(SerialPort);

  write(SerialPort, disable_rmc, sizeof(disable_rmc));
  tcdrain(SerialPort);

  write(SerialPort, disable_vtg, sizeof(disable_vtg));
  tcdrain(SerialPort);

  write(SerialPort, enable_posllh, sizeof(enable_posllh));
  tcdrain(SerialPort);

  write(SerialPort, enable_status, sizeof(enable_status));
  tcdrain(SerialPort);

  write(SerialPort, enable_velned, sizeof(enable_velned));
  tcdrain(SerialPort);

  write(SerialPort, enable_sol, sizeof(enable_sol));
  tcdrain(SerialPort);

  write(SerialPort, enable_timeutc, sizeof(enable_timeutc));
  tcdrain(SerialPort);

  write(SerialPort, enable_svinfo, sizeof(enable_svinfo));
  tcdrain(SerialPort);

  write(SerialPort, airborne1g, sizeof(airborne1g));
  tcdrain(SerialPort);

  write(SerialPort, set_rate, sizeof(set_rate));
  tcdrain(SerialPort);
}
std::string charToHex(char c)
{
  std::stringstream ss;
  ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)c;
  return ss.str();
}

int NEO6m::makeI4(const int cursor)
{
  return static_cast<int32_t>(
      (static_cast<uint32_t>(payloadBuffer[cursor])) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 1]) << 8) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 2]) << 16) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 3]) << 24));
}

uint32_t NEO6m::makeU4(const int cursor)
{
  return static_cast<uint32_t>(
      (static_cast<uint32_t>(payloadBuffer[cursor])) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 1]) << 8) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 2]) << 16) |
      (static_cast<uint32_t>(payloadBuffer[cursor + 3]) << 24));
}

void NEO6m::runNEO6m()
{

  usleep(1000000);

  uint8_t CLASS, ID, LENGHTL, LENGHTM;
  uint16_t state = 1;
  int payloadSize;
  int payloadCursor = 0;

  // std::string paket = "B5 ";

  unsigned char byte;
  while (true)
  {
    int n = read(SerialPort, &byte, 1);
    if (n > 0)
    {
      switch (state)
      {
      case 1:
        if (byte == 0xB5)
          state++;
        else
          state = 1;
        break;

      case 2:
        if (byte == 0x62)
          state++;
        else
          state = 1;
        break;

      case 3:
        CLASS = byte;
        state++;

        break;

      case 4:
        ID = byte;
        state++;

        break;

      case 5:
        LENGHTL = byte;
        state++;

        break;

      case 6:
        LENGHTM = byte;
        payloadSize = LENGHTL | (LENGHTM << 8);
        if (payloadSize > 1024)
          state = 1;
        else
        {
          payloadCursor = 0;
          state++;
        }
        break;

      case 7:
        payloadBuffer[payloadCursor++] = byte;
        if (payloadCursor == payloadSize)
        {
          state++;
        }
        break;

      case 8:
        state++;
        break;

      case 9:
        if (CLASS == 0x01 && (ID == 0x30 || ID == 0x06 || ID == 0x21 ||
                              ID == 0x12 || ID == 0x03 || ID == 0x02))
          handlUBX(CLASS, ID, payloadSize);
        state = 1;
        break;
      }
      //  if(n> 0 && byte == 0xB5) {
      //     std::cout<< paket<<" taille "<< payloadSize <<" cursor " <<
      //     payloadCursor<<std::endl; paket="B5 ";
      // }
      // else paket +=charToHex(byte) + " ";
    }
  }
}

void NEO6m::handlUBX(uint8_t CLASS, uint8_t ID, uint16_t payloadSize)
{
  // std::cout << " Classe : " << charToHex(CLASS) << " ID : " << charToHex(ID)
  //           << " taille du payload : " << payloadSize << std::endl;

  std::lock_guard<std::mutex> lock(mtx);

  if (CLASS == 0x01)
    switch (ID)
    {

    case 0x21:
      if (payloadSize == 20)
      {
        timeArray[0] = (payloadBuffer[13] << 8) | payloadBuffer[12];
        timeArray[1] = payloadBuffer[14];
        timeArray[2] = payloadBuffer[15];
        timeArray[3] = payloadBuffer[16];
        timeArray[4] = payloadBuffer[17];
        timeArray[5] = payloadBuffer[18];
        // for(int i : timeArray){
        //     std::cout<<i << std::endl;
        // }
      }
      break;

    case 0x03:
      if (payloadSize == 16)
      {
        if (payloadBuffer[4] == 0x03)
          gpsFixOk = true;
        else
          gpsFixOk = false;
      }

    case 0x12:
      if (payloadSize == 36)
      {
        velNED[0] = makeI4(4);
        velNED[1] = makeI4(8);
        velNED[2] = makeI4(12);

        speed = makeU4(16);
        GS = makeU4(20);
        heading = makeI4(24) * 1e-5;

        // std::cout<< speed << " cm/s "<<GS<<" cm/s "<< heading <<std::endl;
      }
      break;

    case 0x02:
      if (payloadSize == 28)
      {

        longitude = makeI4(4) * 1e-7;
        latitude = makeI4(8) * 1e-7;

        // std::cout<<std::setprecision(7) <<longitude<<" "
        // <<std::setprecision(9) << latitude << std::endl;
      }
      break;

    case 0x30:
      sats.clear();
      const uint8_t N = payloadBuffer[4];
      for (int n = 0; n < N; n++)
      {
        sats.push_back({payloadBuffer[9 + 12 * n], payloadBuffer[12 + 12 * n],
                        payloadBuffer[11 + 12 * n]});
      }
      // std::cout << (int)N << std::endl;

      // std::cout << "Liste des satellites visibles :\n";
      // std::cout << "-------------------------------\n";
      // std::cout << " ID  | Strenght | Quality\n";
      // std::cout << "-------------------------------\n";

      // for (const auto &sat : sats)
      // {
      //   std::cout << " "
      //             << std::setw(3) << static_cast<int>(sat.ID) << " | "
      //             << std::setw(8) << static_cast<int>(sat.strenght) << " | "
      //             << std::setw(7) << static_cast<int>(sat.quality) << "\n";
      // }

      // std::cout << "-------------------------------\n";

      break;
    }
}

NEO6m::coordPaket NEO6m::getGPSCoord()
{
  std::lock_guard<std::mutex> lock(mtx);
  return {longitude, latitude};
}

// NEO6m::gpsState NEO6m::getGPSState(){
//   std::lock_guard<std::mutex> lock(mtx);
//   gpsState state;
//   state.
// }