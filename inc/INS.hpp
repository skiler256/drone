#pragma once
#include "../inc/eventManager.hpp"
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/NEO6m.hpp"
#include <mutex>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>

class INS
{
public:
  struct settings
  {
    int refreshRate;
    int zRefreshRate = 1;
    double alphaHeading;
    int NGPSattempt = 500;
    int NmoyGPScalib = 10;
    double baseAltitude;
  };
  struct state3D
  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Matrix<double, 3, 1> vel;
    Eigen::Matrix<double, 3, 1> att;
  };
  struct Kalman1D
  {
    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 2> P;
    Eigen::Matrix<double, 2, 2> Q;
    double R;

    Kalman1D(double q, double r);
    void update(double z, double dt);
    double getvalue();
    double getvelocity();
  };

  INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS::settings &set);

  void runINS();
  state3D getState3D();
  void printData();
  bool doDebug = false;

private:
  eventManager &event;
  ESP32 &esp;
  BMP280 &baro;
  NEO6m &gps;

  void computeHeading();
  void computeZ();
  void acquireMPU();
  void acquireZ();

  // data
  state3D state;

  // raw sensor data
  ESPdata dataESP;
  NEO6m::coordPaket coord;
  BMP280::Data bmpData;

  // clibration
  Eigen::Matrix<double, 3, 3> calMagMatrix;
  Eigen::Matrix<double, 3, 1> magBiasVec;
  double basePressure;

  // Sensor
  Eigen::Matrix<double, 3, 1> calibratedMag;
  Eigen::Matrix<double, 3, 1> z;
  std::chrono::_V2::steady_clock::time_point tz;
  GeographicLib::LocalCartesian projGPS;

  // filter
  std::chrono::_V2::steady_clock::time_point tMag;
  Eigen::Matrix<double, 3, 1> FilteredCalibratedMag;
  Kalman1D kx, ky, kz;

  // setting
  settings set;

  // mutex
  std::mutex mtxDataESP;
  std::mutex mtxState3D;
  std::mutex mtxZ;
};