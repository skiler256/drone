#include "../inc/INS.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <thread>
#include <random>

INS::INS(eventManager &event, ESP32 &esp, BMP280 &baro, NEO6m &gps, INS::settings &set)
    : event(event), esp(esp), baro(baro), gps(gps), set(set),
      kx(100.0, 800.0),
      ky(100.0, 800.0),
      kz(100.0, 800.0),
      kalman()
{

  tMag = std::chrono::steady_clock::now();
  tz = std::chrono::steady_clock::now();

  calibratedMag << 0.0, 0.0, 0.0;
  FilteredCalibratedMag << 0.0, 0.0, 0.0;
  linearizedAcc << 0.0, 0.0, 0.0;

  // Magneto
  magBiasVec << 192.046411, -68.294232, 508.612908;
  calMagMatrix << 1.110525, -0.009313, -0.005285, -0.009313, 1.148244, 0.001096,
      -0.005285, 0.001096, 1.207234;

  run = std::thread(&INS::runINS, this);
}

INS::~INS()
{
  loop = false;
  if (run.joinable())
    run.join();
}

void INS::runINS()
{
  while (loop)
  {
    const auto start = std::chrono::steady_clock::now();

    acquireMPU();
    computeHeading();
    linearizeAcc();
    kalman.pred(linearizedAcc);
    {
      std::lock_guard<std::mutex> lock(mtxState3D);
      auto x = kalman.getX();
      state.pos << x(0), x(1), x(2);
      state.vel << x(3), x(4), x(5);
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tz).count() >= (1000 / set.zRefreshRate))
    {
      tz = std::chrono::steady_clock::now();
      acquireZ();
      computeZ();
      static std::random_device rd;
      static std::mt19937 gen(rd());

      // Position horizontale dans un cercle de 2.5 m
      std::uniform_real_distribution<double> dist_angle(0, 2 * M_PI);
      std::uniform_real_distribution<double> dist_radius(0, 2);
      double r = dist_radius(gen);
      double theta = dist_angle(gen);
      double x = r * cos(theta);
      double y = r * sin(theta);

      // Position verticale +/- 0.5 m
      std::uniform_real_distribution<double> dist_z(-0.25, 0.25);
      double z_pos = dist_z(gen);

      // Vitesse +/- 2 m/s
      std::uniform_real_distribution<double> dist_vel(-1, 1);
      double vx = dist_vel(gen);
      double vy = dist_vel(gen);
      double vz = dist_vel(gen);

      Eigen::Matrix<double, 6, 1> z;
      z << x, y, z_pos, vx, vy, vz;

      kalman.update(z);
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

  double pitch;
  double roll;

  {
    std::lock_guard<std::mutex> lockState3D(mtxState3D);
    pitch = state.att(1) * M_PI / 180.0;
    roll = state.att(0) * M_PI / 180.0;
  }

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
    // std::cout << state.att(2) << "\n";
  }
}

void INS::linearizeAcc()
{
  double pitch;
  double roll;
  double yaw;

  Eigen::Matrix<double, 3, 1> rawAcc;
  {
    std::lock_guard<std::mutex> lock(mtxDataESP);
    rawAcc << dataESP.ax, dataESP.ay, dataESP.az;
  }

  // Eigen::Matrix<double, 3, 1> b;
  // b << 0.038001, 0.002424, 0.003792;

  // Eigen::Matrix<double, 3, 3> Ainv;
  // Ainv << 0.997986, 0.000071, 0.000114,
  //     0.000071, 0.997792, 0.000190,
  //     0.000114, 0.000190, 0.987754;

  // rawAcc = Ainv * (rawAcc - b); // Moyen bof ...

  {
    std::lock_guard<std::mutex> lockState3D(mtxState3D);
    pitch = state.att(1) * M_PI / 180.0;
    roll = state.att(0) * M_PI / 180.0;
    yaw = state.att(2) * M_PI / 180.0;
  }

  Eigen::Matrix<double, 3, 3> Rx_inv;
  Rx_inv << 1, 0, 0, 0, cos(roll), sin(roll), 0, -sin(roll), cos(roll);

  Eigen::Matrix<double, 3, 3> Ry_inv;
  Ry_inv << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 0, cos(pitch);

  Eigen::Matrix<double, 3, 3> Rz_inv;
  Rz_inv << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;

  Eigen::Matrix<double, 3, 3> R = Rz_inv * Ry_inv * Rx_inv;

  linearizedAcc = R * rawAcc;
  linearizedAcc(2) -= 1;
  linearizedAcc(1) *= -1; // inverser axe y --> NED

  linearizedAcc *= 9.81;
  linearizedAcc = linearizedAcc.unaryExpr([](double v)
                                          { return std::round(v * 10.0) / 10.0; });
  linearizedAcc(2) -= 0.1;

  std::cout
      << std::fixed << std::setprecision(1)
      << linearizedAcc(0) << " "
      << linearizedAcc(1) << " "
      << linearizedAcc(2) << "\n";
}

void INS::computeZ()
{
  std::lock_guard<std::mutex> lock(mtxZ);

  double altitude = ((bmpData.temperature + 273.15) / 0.0065) * (pow(calibration.pressure / bmpData.pressure, 0.1903) - 1) + set.baseAltitude;
  std::cout << coord.latitude << std::endl;
  projGPS.Forward(coord.latitude, coord.longitude, altitude, z(0), z(1), z(2));
}

void INS::setCalibration(const INS::CALIBRATION &calibration_)
{
  std::lock_guard<std::mutex> lock(mtxZ); // ne touche que GPS et BARO
  calibration = calibration_;
  std::cout << set.baseAltitude << "\n";
  projGPS.Reset(calibration.latitude, calibration.longitude, set.baseAltitude);
}

INS::settings INS::getSettings()
{
  return set;
}

void INS::setSettings(INS::settings set_)
{
  std::lock_guard<std::mutex> lock(mtxZ);
  std::lock_guard<std::mutex> lockb(mtxState3D);

  set = set_;
  projGPS.Reset(calibration.latitude, calibration.longitude, set.baseAltitude);
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
  // state.pos = z; // A ENLEVER!!!
  return state;
}

INS::KalmanLinear::KalmanLinear() : t(std::chrono::steady_clock::now())
{

  R(0, 0) = 0.5 * 0.5; // variance de position X (m²)
  R(1, 1) = 0.5 * 0.5; // variance Y
  R(2, 2) = 0.5 * 0.5; // variance Z
  R(3, 3) = 0.2 * 0.2; // variance vitesse X
  R(4, 4) = 0.2 * 0.2; // Y
  R(5, 5) = 0.2 * 0.2; // Z
}

Eigen::Matrix<double, 6, 6> INS::KalmanLinear::getQ(double dt)
{
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  double sigma_a = 0.1; // incertitude sur l'accélération (m/s²)
  Q(0, 0) = Q(1, 1) = Q(2, 2) = pow(dt, 4) / 4 * sigma_a * sigma_a;
  Q(3, 3) = Q(4, 4) = Q(5, 5) = pow(dt, 2) * sigma_a * sigma_a;
  Q(0, 3) = Q(1, 4) = Q(2, 5) = pow(dt, 3) / 2 * sigma_a * sigma_a;
  Q(3, 0) = Q(4, 1) = Q(5, 2) = pow(dt, 3) / 2 * sigma_a * sigma_a;
  return Q;
}

Eigen::Matrix<double, 6, 6> INS::KalmanLinear::getF(double dt)
{

  Eigen::Matrix<double, 6, 6> F;
  F << 1, 0, 0, dt, 0, 0,
      0, 1, 0, 0, dt, 0,
      0, 0, 1, 0, 0, dt,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;
  return F;
}

Eigen::Matrix<double, 6, 3> INS::KalmanLinear::getB(double dt)
{

  Eigen::Matrix<double, 6, 3> B;
  B << pow(dt, 2) / 2, 0, 0,
      0, pow(dt, 2) / 2, 0,
      0, 0, pow(dt, 2) / 2,
      dt, 0, 0,
      0, dt, 0,
      0, 0, dt;
  return B;
}

void INS::KalmanLinear::pred(Eigen::Matrix<double, 3, 1> u)
{
  std::lock_guard<std::mutex> lock(mtx);
  double dt = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t).count() * 1e-6;
  t = std::chrono::steady_clock::now();

  auto F = getF(dt);
  auto B = getB(dt);
  auto Q = getQ(dt);

  x = F * x + B * u;
  P = F * P * F.transpose() + Q;
}

void INS::KalmanLinear::update(Eigen::Matrix<double, 6, 1> z)
{
  std::lock_guard<std::mutex> lock(mtx);

  Eigen::Matrix<double, 6, 6> S;
  S = H * P * H.transpose() + R;

  Eigen::Matrix<double, 6, 6> K;
  K = P * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

  Eigen::Matrix<double, 6, 1> y;
  y = z - H * x;

  x = x + K * y;
}

Eigen::Matrix<double, 6, 1> INS::KalmanLinear::getX()
{
  std::lock_guard<std::mutex> lock(mtx);
  return x;
}