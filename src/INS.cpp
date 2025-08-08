#include "../inc/INS.hpp"
#include <cmath>
#include <iostream>
#include <thread>

INS::INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS::settings &set)
    : event(event), esp(esp), baro(baro), gps(gps), set(set),
      kx(100.0, 800.0),
      ky(100.0, 800.0),
      kz(100.0, 800.0)
{
  tMag = std::chrono::steady_clock::now();
  tz = std::chrono::steady_clock::now();

  calibratedMag << 0.0, 0.0, 0.0;
  FilteredCalibratedMag << 0.0, 0.0, 0.0;

  // Magneto
  magBiasVec << 192.046411, -68.294232, 508.612908;
  calMagMatrix << 1.110525, -0.009313, -0.005285, -0.009313, 1.148244, 0.001096,
      -0.005285, 0.001096, 1.207234;

  // calib GPS et BARO
  int GPSattempt = 0;
  while (!gps.isFix() || GPSattempt > set.NGPSattempt)
  {
    GPSattempt++;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "loiloi" << gps.isFix() << std::endl;
  }
  if (!gps.isFix())
    event.reportEvent({component::INS, subcomponent::dataLink, eventSeverity::CRITICAL, "impossible d'obtenir un FIX 3D"});
  double latitude = 0;
  double longitude = 0;
  double pressure = 0;

  for (int i = 0; i < set.NmoyGPScalib; i++)
  {
    NEO6m::coordPaket coord = gps.getGPSCoord();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    latitude += coord.latitude;
    longitude += coord.longitude;

    BMP280::Data bmp;
    bmp = baro.getData();

    pressure += bmp.pressure;
  }
  latitude /= set.NmoyGPScalib;
  longitude /= set.NmoyGPScalib;
  basePressure = pressure / set.NmoyGPScalib;

  projGPS.Reset(latitude, longitude, set.baseAltitude);
}

void INS::runINS()
{
  while (true)
  {
    const auto start = std::chrono::steady_clock::now();

    acquireMPU();
    computeHeading();

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tz).count() >= (1000 / set.zRefreshRate))
    {
      tz = std::chrono::steady_clock::now();
      acquireZ();
      computeZ();
    }

    const auto end = std::chrono::steady_clock::now();
    int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    int target_period_ms = static_cast<int>(1000.0 / set.refreshRate);
    int remaining_sleep_ms = std::max(0, target_period_ms - elapsed_ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(remaining_sleep_ms));
  }
}

void INS::acquireMPU()
{
  {
    std::lock_guard<std::mutex> lock(mtxDataESP);
    dataESP = esp.getData();
    event.reportEvent({component::INS, subcomponent::dataLink, eventSeverity::INFO, "reception donne esp32"});
  }
  {
    std::lock_guard<std::mutex> lockState3D(mtxState3D);
    state.att(0) = -dataESP.roll;
    state.att(1) = dataESP.pitch;
  }
}

void INS::acquireZ()
{
  std::lock_guard<std::mutex> lock(mtxZ);

  coord = gps.getGPSCoord();

  bmpData = baro.getData();
}

void INS::computeHeading()
{
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

  double heading =
      std::atan2(FilteredCalibratedMag(1), FilteredCalibratedMag(0));
  heading *= 180.0 / M_PI;
  // if (heading < 0)
  //     heading += 360.0;

  {

    std::lock_guard<std::mutex> lockState3D(mtxState3D);
    double delta = heading - state.att(2);
    if (delta > 180)
      delta -= 360;
    if (delta < -180)
      delta += 360;

    state.att(2) = state.att(2) + set.alphaHeading * delta;
  }
}

void INS::computeZ()
{
  std::lock_guard<std::mutex> lock(mtxZ);

  double altitude = ((bmpData.temperature + 273.15) / 0.0065) * (pow(basePressure / bmpData.pressure, 0.1903) - 1) + set.baseAltitude;
  std::cout << altitude << std::endl;
  projGPS.Forward(coord.latitude, coord.longitude, altitude, z(0), z(1), z(2));
}

void INS::printData()
{
  std::lock_guard<std::mutex> lock(mtxState3D);
  std::cout << "yaw : " << state.att(2) << "roll : " << state.att(0) << "pitch : " << state.att(1) << std::endl;
}

// Kalman 1D :

INS::Kalman1D::Kalman1D(double q, double r)
{
  x << 0, 0;
  P.setIdentity();
  Q << q, 0, 0, q;
  R = r;
}

void INS::Kalman1D::update(double z, double dt)
{
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

INS::state3D INS::getState3D()
{
  std::lock_guard<std::mutex> lock(mtxState3D);
  state.pos = z; // A ENLEVER!!!
  return state;
}