#pragma once
#include "../inc/BMP280.hpp"
#include "../inc/ESP32.hpp"
#include "../inc/NEO6m.hpp"
#include "../inc/eventManager.hpp"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <mutex>


class INS {
public:
  INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps);

  void acquireESP();

  void calibrateMag();

  void computeHeading();

  void printCalMag();
  Eigen::Matrix<double, 3, 1> calibratedMag;

  struct Kalman1D {
    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 2> P;
    Eigen::Matrix<double, 2, 2> Q;
    double R;

    Kalman1D(double q, double r);
    void update(double z, double dt);
    double getvalue();
    double getvelocity();
  };

private:
  eventManager &event;
  ESP32 &esp;
  BMP280 &baro;
  NEO6m &gps;

  ESPdata dataESP;

  std::mutex mtxDataESP;

  // clibration
  Eigen::Matrix<double, 3, 3> calMagMatrix;
  Eigen::Matrix<double, 3, 1> magBiasVec;

  // Sensor
  double prevHeading = 0.0;

  // filter
  std::chrono::_V2::steady_clock::time_point tMag;
  Eigen::Matrix<double, 3, 1> FilteredCalibratedMag;
  Kalman1D kx, ky, kz;

  // setting
  const double alphaHeading = 0.5;
};