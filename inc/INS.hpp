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
#include <stdint.h>
#include <optional>

#define NB_vs 2

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
    Eigen::Matrix<double, 3, 1> accNED;
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
    Eigen::Matrix<double, 6, 3> getB(double dt);

    Eigen::Matrix<double, 6, 6> getQ(double dt);

    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();

    KalmanLinear();
    void pred(Eigen::Matrix<double, 3, 1> u);
    void update(Eigen::Matrix<double, 6, 1> z, int hAcc, uint32_t sAcc, double dt);
    Eigen::Matrix<double, 6, 1> getX();
  };

  struct HPfilter
  {
    double a;
    double x[2] = {0, 0};
    double y[2] = {0, 0};

    HPfilter(double dt, double fc);
    double update(double X);
  };

  struct IIRfilter
  {
    double y = 0;
    double update(double x, double alpha);
  };

  INS(eventManager &event, std::optional<BMP280> &bmp, INS::settings &set);
  ~INS();

  state3D getState3D();
  void printData();
  bool doDebug = false;

  void setCalibration(const CALIBRATION &calibration_);
  settings getSettings();
  void setSettings(settings set);

  void updateMPU(ESPdata data);
  void updateGPS(NEO6m::coordPaket coord, int velNED[3], uint32_t pAcc, uint32_t sAcc);
  void updataBMP(BMP280::Data data);

private:
  eventManager &event;
  std::optional<BMP280> &bmp;
  // setting
  settings set;
  Kalman1D kx, ky, kz;
  HPfilter HPxacc, HPyacc, HPzacc;
  IIRfilter altIIR;

  void computeHeading();
  void computeZ();
  void linearizeAcc();

  // data
  state3D state;

  // raw sensor data
  ESPdata dataESP;

  // clibration
  Eigen::Matrix<double, 3, 3> calMagMatrix;
  Eigen::Matrix<double, 3, 1> magBiasVec;
  CALIBRATION calibration;

  // Sensor
  Eigen::Matrix<double, 3, 1> calibratedMag;
  GeographicLib::LocalCartesian projGPS;
  Eigen::Matrix<double, 3, 1> linearizedAcc;
  std::chrono::_V2::steady_clock::time_point tz;
  double prevAlt[NB_vs] = {};

  // filter
  std::chrono::_V2::steady_clock::time_point tMag;
  Eigen::Matrix<double, 3, 1> FilteredCalibratedMag;
  // Kalman1D kx, ky, kz;
  KalmanLinear kalman;

  // setting
  // settings set;

  // mutex
  std::mutex mtxDataESP;
  std::mutex mtxState3D;
  std::mutex mtxZ;

  friend class PROCEDURE;
  friend class behaviorCenter;
};