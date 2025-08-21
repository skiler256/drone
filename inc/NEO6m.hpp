#pragma once
#include "../inc/eventManager.hpp"
#include <fstream>
#include <mutex>
#include <termios.h>
#include <vector>

#include <atomic>
#include <thread>

std::string charToHex(char c);

class NEO6m
{
public:
  NEO6m(eventManager &event, const char *portName = "/dev/ttyAMA1");
  ~NEO6m();
  void runNEO6m();

  struct coordPaket
  {
    double latitude = 44;
    double longitude = 0.7;
  };

  struct SatelliteInfo
  {
    uint8_t ID;
    uint8_t strenght;
    uint8_t quality;
  };

  struct gpsState
  {
    std::vector<SatelliteInfo> sats;
    coordPaket coord;
    int timeArray[6] = {0, 0, 0, 0, 0, 0};
    bool gpsFixOk = false; // fix 3D only

    int velNED[3] = {0, 0, 0}; // velned --> vNORTH vEAST vDOWN
    uint32_t speed = 0;
    uint32_t GS = 0;
    double heading = 0.0;
  };

  coordPaket getGPSCoord();
  gpsState getGPSState();
  bool isFix();

private:
  void handlUBX(uint8_t CLASS, uint8_t ID, uint16_t payloadSize);

  int makeI4(const int cursor);
  uint32_t makeU4(const int cursor);

  int SerialPort;
  eventManager &event;
  const char *portName;
  termios tty;

  std::mutex mtx;

  std::atomic<bool> loop = true;
  std::thread run;

  uint8_t payloadBuffer[1024];

  // GPS data

  int timeArray[6] = {0, 0, 0, 0, 0, 0};

  bool gpsFixOk = false; // fix 3D only

  int velNED[3] = {0, 0, 0}; // velned --> vNORTH vEAST vDOWN
  uint32_t speed = 0;
  uint32_t GS = 0;
  double heading = 0.0;

  double longitude = 0.0;
  double latitude = 0.0;

  std::vector<SatelliteInfo> sats;
};
