#include "../inc/INS.hpp"
#include <cmath>
#include <iostream>


INS::INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps)
    : event(event), esp(esp), baro(baro), gps(gps), kx(100.0, 800.0),
      ky(100.0, 800.0), kz(100.0, 800.0) {
  tMag = std::chrono::steady_clock::now();
  calibratedMag << 0.0, 0.0, 0.0;
  FilteredCalibratedMag << 0.0, 0.0, 0.0;
  // Pmag << 1.0, 1.0, 1.0;
  // Qmag << 100, 100, 100; // Faible incertitude modèle (on suppose peu de
  // variation brute) Rmag << 800, 800.0, 800.0;

  // Magneto
  magBiasVec << 192.046411, -68.294232, 508.612908;
  calMagMatrix << 1.110525, -0.009313, -0.005285, -0.009313, 1.148244, 0.001096,
      -0.005285, 0.001096, 1.207234;
}

void INS::acquireESP() {
  std::lock_guard<std::mutex> lock(mtxDataESP);
  dataESP = esp.getData();
}

void INS::calibrateMag() {
  Eigen::Vector3d rawMag;
  {
    std::lock_guard<std::mutex> lock(mtxDataESP);
    rawMag << dataESP.my, dataESP.mx, -dataESP.mz;
  }

  calibratedMag = calMagMatrix * (rawMag - magBiasVec);

  const double pitch = dataESP.pitch * M_PI / 180.0;
  const double roll = -dataESP.roll * M_PI / 180.0;

  Eigen::Matrix<double, 3, 3> Rx_inv;
  Rx_inv << 1, 0, 0, 0, cos(roll), sin(roll), 0, -sin(roll), cos(roll);

  Eigen::Matrix<double, 3, 3> Ry_inv;
  Ry_inv << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 0, cos(pitch);

  Eigen::Matrix<double, 3, 3> R = Ry_inv * Rx_inv;

  calibratedMag = R * calibratedMag;

  const double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - tMag)
                        .count() *
                    1e-6;
  tMag = std::chrono::steady_clock::now();

  kx.update(calibratedMag(0), dt);
  FilteredCalibratedMag(0) = kx.getvalue();

  ky.update(calibratedMag(1), dt);
  FilteredCalibratedMag(1) = ky.getvalue();

  kz.update(calibratedMag(2), dt);
  FilteredCalibratedMag(2) = kz.getvalue();
}

void INS::computeHeading() {
  double heading =
      std::atan2(FilteredCalibratedMag(1), FilteredCalibratedMag(0));
  heading *= 180.0 / M_PI;
  // if (heading < 0)
  //     heading += 360.0;

  double delta = heading - prevHeading;
  if (delta > 180)
    delta -= 360;
  if (delta < -180)
    delta += 360;

  heading = prevHeading + alphaHeading * delta;
  prevHeading = heading;

  std::cout << "Heading: " << heading << "°" << std::endl;
  std::cout << "euler : " << dataESP.roll << " " << dataESP.pitch << std::endl;
}

void INS::printCalMag() {
  std::cout << "calibrated : " << calibratedMag(0) << " " << calibratedMag(1)
            << " " << calibratedMag(2) << std::endl;
}

// Kalman 1D :

INS::Kalman1D::Kalman1D(double q, double r) {
  x << 0, 0;
  P.setIdentity();
  Q << q, 0, 0, q;
  R = r;
}

void INS::Kalman1D::update(double z, double dt) {
  // Matrices
  Eigen::Matrix2d A;
  A << 1, dt, 0, 1;

  Eigen::RowVector2d H;
  H << 1, 0;

  Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

  // Predict
  x = A * x;
  P = A * P * A.transpose() + Q;

  // Update
  double y = z - H * x;
  double S = (H * P * H.transpose())(0, 0) + R;
  Eigen::Vector2d K = P * H.transpose() / S;

  x = x + K * y;
  P = (I - K * H) * P;
}

double INS::Kalman1D::getvalue() { return x(0); }
double INS::Kalman1D::getvelocity() { return x(1); }