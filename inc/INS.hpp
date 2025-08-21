#pragma once
#include "../inc/eventManager.hpp"
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/NEO6m.hpp"
#include <mutex>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <atomic>

class INS
{
public:
  struct settings
  {
    int refreshRate = 200;
    int zRefreshRate = 1;
    double alphaHeading = 0.5;
    int NGPSattempt = 500;
    int NmoyGPScalib = 10;
    double baseAltitude = 120;
  };

  struct CALIBRATION
  {
    double pressure = 1013;
    double latitude = 44.09;
    double longitude = 0.76;
  };

  struct state3D
  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Matrix<double, 3, 1> vel;
    Eigen::Matrix<double, 3, 1> att;
    uint8_t INSstate = 0;
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

  struct KalmanLinear
  {
    std::chrono::_V2::steady_clock::time_point t;
    std::mutex mtx;

    Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix<double, 6, 6> getF(double dt);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 3> getB(double dt);

    Eigen::Matrix<double, 6, 6> getQ(double dt);
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();

    KalmanLinear();
    void pred(Eigen::Matrix<double, 3, 1> u);
    void update(Eigen::Matrix<double, 6, 1> z);
    Eigen::Matrix<double, 6, 1> getX();
  };

  INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS::settings &set);
  ~INS();
  void runINS();
  state3D getState3D();
  void printData();
  bool doDebug = false;

  void setCalibration(const CALIBRATION &calibration_);
  settings getSettings();
  void setSettings(settings set);

private:
  eventManager &event;
  ESP32 &esp;
  BMP280 &baro;
  NEO6m &gps;

  void computeHeading();
  void computeZ();
  void acquireMPU();
  void acquireZ();

  void linearizeAcc();

  // data
  state3D state;

  // raw sensor data
  ESPdata dataESP;
  NEO6m::coordPaket coord;
  BMP280::Data bmpData;

  // clibration
  Eigen::Matrix<double, 3, 3> calMagMatrix;
  Eigen::Matrix<double, 3, 1> magBiasVec;
  CALIBRATION calibration;

  // Sensor
  Eigen::Matrix<double, 3, 1> calibratedMag;
  Eigen::Matrix<double, 3, 1> z;
  std::chrono::_V2::steady_clock::time_point tz;
  GeographicLib::LocalCartesian projGPS;
  Eigen::Matrix<double, 3, 1> linearizedAcc;

  // filter
  std::chrono::_V2::steady_clock::time_point tMag;
  Eigen::Matrix<double, 3, 1> FilteredCalibratedMag;
  Kalman1D kx, ky, kz;
  KalmanLinear kalman;

  // setting
  settings set;

  // mutex
  std::mutex mtxDataESP;
  std::mutex mtxState3D;
  std::mutex mtxZ;

  std::atomic<bool> loop = true;
  std::thread run;

  friend class PROCEDURE;
  friend class behaviorCenter;
};