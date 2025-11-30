#include "../inc/NEO6m.hpp"
#include "../inc/SensorFusion.hpp"
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

NEO6m::NEO6m(eventManager &event, std::optional<SensorFusion> &sens, const char *portName)
    : event(event), sens(sens), portName(portName),
      SerialPort(open(portName, O_RDWR | O_NOCTTY))
{
  event.declare(this, component::GPS);
  if (sens)
    sens->ident(this, sensor::GPS);

  std::lock_guard<std::mutex>
      lock(mtx);

  if (tcgetattr(SerialPort, &tty) < 0)
    event.report(this, category::connection, {severity::CRITICAL, "impossible d'ouvrir le port série"});
  else
    event.report(this, category::connection, {severity::INFO, "port série ouvert"});

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

  const uint8_t CFG_BAUDRATE[] = {
      0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
      0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
      0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};

  write(SerialPort, CFG_BAUDRATE, sizeof(CFG_BAUDRATE));
  tcdrain(SerialPort);

  close(SerialPort);

  SerialPort = -1;

  SerialPort = open(portName, O_RDWR | O_NOCTTY);

  if (tcgetattr(SerialPort, &tty) < 0)
    event.report(this, category::connection, {severity::CRITICAL, "impossible d'ouvrir le port série"});
  else
    event.report(this, category::connection, {severity::INFO, "port série ouvert"});

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

  // GNSS config
  const uint8_t CFG_GNSS[] = {
      0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00,
      0x01, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08,
      0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04,
      0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01,
      0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x30, 0xAD};

  // NAV5 config
  const uint8_t CFG_NAV5[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03,
      0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00,
      0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x02, 0x3C,
      0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x85, 0xCE};

  // Désactivation GxGGA
  const uint8_t CFG_MSG_GGA_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};

  // Désactivation GxGLL
  const uint8_t CFG_MSG_GLL_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};

  // Désactivation GxGSA
  const uint8_t CFG_MSG_GSA_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};

  // Désactivation GxGSV
  const uint8_t CFG_MSG_GSV_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};

  // Désactivation GxRMC
  const uint8_t CFG_MSG_RMC_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};

  // Désactivation GxVTG
  const uint8_t CFG_MSG_VTG_OFF[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  // POSLLH ON
  const uint8_t CFG_MSG_POSLLH_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE};

  // STATUS ON
  const uint8_t CFG_MSG_STATUS_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5};

  // VELNED ON
  const uint8_t CFG_MSG_VELNED_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x23, 0x2E};

  // SOL ON
  const uint8_t CFG_MSG_SOL_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDA};

  // SVINFO ON
  const uint8_t CFG_MSG_SVINFO_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00};

  // TIMEUTC ON
  const uint8_t CFG_MSG_TIMEUTC_ON[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x32, 0x97};

  const uint8_t CFG_MSG_PVT_ON[] = {
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
      0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
      0x18, 0xE1};

  // rate
  const uint8_t CFG_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

  write(SerialPort, CFG_GNSS, sizeof(CFG_GNSS));
  tcdrain(SerialPort);
  write(SerialPort, CFG_NAV5, sizeof(CFG_NAV5));
  tcdrain(SerialPort);

  write(SerialPort, CFG_MSG_GGA_OFF, sizeof(CFG_MSG_GGA_OFF));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_GLL_OFF, sizeof(CFG_MSG_GLL_OFF));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_GSA_OFF, sizeof(CFG_MSG_GSA_OFF));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_GSV_OFF, sizeof(CFG_MSG_GSV_OFF));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_RMC_OFF, sizeof(CFG_MSG_RMC_OFF));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_VTG_OFF, sizeof(CFG_MSG_VTG_OFF));
  tcdrain(SerialPort);

  write(SerialPort, CFG_MSG_POSLLH_ON, sizeof(CFG_MSG_POSLLH_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_STATUS_ON, sizeof(CFG_MSG_STATUS_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_VELNED_ON, sizeof(CFG_MSG_VELNED_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_SOL_ON, sizeof(CFG_MSG_SOL_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_SVINFO_ON, sizeof(CFG_MSG_SVINFO_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_TIMEUTC_ON, sizeof(CFG_MSG_TIMEUTC_ON));
  tcdrain(SerialPort);
  write(SerialPort, CFG_MSG_PVT_ON, sizeof(CFG_MSG_PVT_ON));
  tcdrain(SerialPort);

  write(SerialPort, CFG_RATE, sizeof(CFG_RATE));
  tcdrain(SerialPort);

  run = std::thread(&NEO6m::runNEO6m, this);
}

NEO6m::~NEO6m()
{
  loop = false;
  if (run.joinable())
    run.join();

  event.erase(this);
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
  while (loop)
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
                              ID == 0x12 || ID == 0x03 || ID == 0x02 || ID == 0x07))
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

  event.report(this, category::parser, {severity::INFO, "reception d un paquet ID : " + std::to_string((int)ID)});

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
        if (payloadBuffer[4] >= 0x03)
          gpsFixOk = true;
        else
          gpsFixOk = false;
      }
      break;

    case 0x12:
      if (payloadSize == 36)
      {
        velNED[0] = makeI4(4);
        velNED[1] = makeI4(8);
        velNED[2] = makeI4(12);

        speed = makeU4(16);
        GS = makeU4(20);
        heading = makeI4(24) * 1e-5;

        // std::cout << velNED[0] << " " << velNED[1] << " " << velNED[2] << "\n";

        // std::cout<< speed << " cm/s "<<GS<<" cm/s "<< heading <<std::endl;
      }
      break;

    case 0x02:
      if (payloadSize == 28)
      {

        longitude = makeI4(4) * 1e-7;
        latitude = makeI4(8) * 1e-7;

        if (gpsFixOk && sens) // Mise a jour INS
        {
          // NEO6m::coordPaket coord = {latitude, longitude};
          getGPSState(); // maj du paquet state
          sens->update(this, &state);
        }

        // std::cout<<std::setprecision(7) <<longitude<<" "
        // <<std::setprecision(9) << latitude << std::endl;
      }
      break;

    case 0x30:
    {
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
    }
    break;

    case 0x07:

    {
      hAcc = makeI4(40);
      sAcc = makeU4(68);
      // std::cout << "Précision GPS : " << hAcc << " " << sAcc << "\n";
    }
    break;
    }
}

NEO6m::coordPaket NEO6m::getGPSCoord()
{
  std::lock_guard<std::mutex> lock(mtx);
  if (gpsFixOk)
    return {latitude, longitude};
  else
  {
    coordPaket coord;
    return coord;
  }
}

NEO6m::gpsState NEO6m::getGPSState()
{
  std::lock_guard<std::mutex> lock(mtx);

  state.sats = sats;
  state.coord = {longitude, latitude};
  state.gpsFixOk = gpsFixOk;

  for (int i = 0; i < 6; i++)
  {
    state.timeArray[i] = timeArray[i];
  }
  for (int i = 0; i < 3; i++)
  {
    state.velNED[i] = velNED[i];
  }

  state.speed = speed;
  state.GS = GS;
  state.heading = heading;

  return state;
}

bool NEO6m::isFix()
{
  std::lock_guard<std::mutex> lock(mtx);
  return gpsFixOk;
}