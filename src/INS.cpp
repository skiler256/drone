#include "../inc/INS.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <thread>
#include <random>

INS::INS(eventManager &event, std::optional<MS5611> &bmp, INS::settings &set)
    : event(event), bmp(bmp), set(set),
      kx(100.0, 800.0),
      ky(100.0, 800.0),
      kz(100.0, 800.0),
      HPxacc(0.005, 1),
      HPyacc(0.005, 1),
      HPzacc(0.005, 1),
      kalman()
{
  event.declare(this, component::INS);

  tMag = std::chrono::steady_clock::now();
  tz = std::chrono::steady_clock::now();

  calibratedMag << 0.0, 0.0, 0.0;
  FilteredCalibratedMag << 0.0, 0.0, 0.0;
  linearizedAcc << 0.0, 0.0, 0.0;

  // Magneto
  magBiasVec << 192.046411, -68.294232, 508.612908;
  calMagMatrix << 1.110525, -0.009313, -0.005285, -0.009313, 1.148244, 0.001096,
      -0.005285, 0.001096, 1.207234;
}

INS::~INS()
{
  event.erase(this);
}

void INS::updateMPU(ESPdata data)
{
  {
    std::lock_guard<std::mutex> lock(mtxDataESP);
    dataESP = data;
    // event.report({component::INS, subcomponent::dataLink, eventSeverity::INFO, "reception donne esp32"});
  }
  {
    std::lock_guard<std::mutex> lockState3D(mtxState3D);
    state.att(0) = -dataESP.roll;
    state.att(1) = dataESP.pitch;
  }

  computeHeading();
  linearizeAcc();
  kalman.pred(linearizedAcc);

  {
    std::lock_guard<std::mutex> lock(mtxState3D);
    auto x = kalman.getX();
    state.pos << x(0), x(1), currAlt;
    state.vel << x(3), x(4), x(5);
    state.accNED = linearizedAcc;
  }
}

void INS::updateGPS(NEO6m::coordPaket coord, int velNED[3], uint32_t pAcc, uint32_t sAcc)
{
  double alt = set.baseAltitude;
  Eigen::Matrix<double, 6, 1> z;
  const double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - tz)
                        .count() *
                    1e-6;
  tz = std::chrono::steady_clock::now();

  if (bmp)
  {
    MS5611::Data bmpData = bmp->getData();
    // alt = ((bmpData.temperature + 273.15) / 0.0065) * (pow(calibration.pressure / bmpData.pressure, 0.1903) - 1) + set.baseAltitude;
    alt = 44330.0 * (1.0 - pow(bmpData.pressure / calibration.pressure, 0.1903)) + set.baseAltitude;
    alt = altIIR.update(alt, 0.5);
    alt = std::round(alt * 10) / 10;

    currAlt = alt - set.baseAltitude;
    // for (int i = 0; i < NB_vs - 1; i++)
    //   prevAlt[i + 1] = prevAlt[i];
    // prevAlt[0] = alt;

    // double t[NB_vs];
    // for (int i = 0; i < NB_vs; i++)
    //   t[i] = dt * (i);

    // double tmoy = 0, ymoy = 0;
    // for (int i = 0; i < NB_vs; i++)
    // {
    //   tmoy += t[i];
    //   ymoy += prevAlt[i];
    // }
    // tmoy /= NB_vs;
    // ymoy /= NB_vs;

    // double ySUM = 0;
    // for (int i = 0; i < NB_vs; i++)
    //   ySUM += (t[i] - tmoy) * (prevAlt[i] - ymoy);

    // double tSum = 0;
    // for (int i = 0; i < NB_vs; i++)
    //   tSum += pow((t[i] - tmoy), 2);

    // z(2) = std::round((ySUM / tSum) * 10) / -10;

    // std::cout << "VS mon con : " << z(2) << " " << alt << "\n";
  }

  projGPS.Forward(coord.latitude, coord.longitude, alt, z(1), z(0), z(2));

  // std::cout << coord.latitude << " " << coord.longitude << " " << alt << "\n";

  z(3) = velNED[0] * 1e-2;
  z(4) = velNED[1] * 1e-2;
  z(5) = velNED[2] * 1e-2;

  kalman.update(z, pAcc, sAcc, dt);
  // state.pos << z(0), z(1), z(2);

  {
    std::lock_guard<std::mutex> lock(mtxState3D);
    auto x = kalman.getX();
    state.pos << x(0), x(1), currAlt;
    state.vel << x(3), x(4), x(5);
  }
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

  // // Biais combiné
  // Eigen::Vector3d b;
  // b << 0.037606, 0.002619, 0.002727;

  // // Correction pour scale/misalignment/soft-iron : A^{-1}
  // Eigen::Matrix3d Ainv;
  // Ainv << 0.997876, 0.000120, 0.000503,
  //     0.000120, 0.997789, 0.000197,
  //     0.000503, 0.000197, 0.988591;

  // rawAcc = Ainv * (rawAcc - b);

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
  linearizedAcc(0) = HPxacc.update(linearizedAcc(0));
  linearizedAcc(1) = HPyacc.update(linearizedAcc(1));
  linearizedAcc(2) = HPzacc.update(linearizedAcc(2));

  // std::cout
  //     << std::fixed << std::setprecision(1)
  //     << linearizedAcc(0) << " "
  //     << linearizedAcc(1) << " "
  //     << linearizedAcc(2) << "\n";
}

void INS::setCalibration(const INS::CALIBRATION &calibration_)
{
  std::lock_guard<std::mutex> lock(mtxZ); // ne touche que GPS et BARO
  calibration = calibration_;
  // std::cout << set.baseAltitude << " " << calibration.pressure << " " << calibration.longitude << " " << calibration.pressure << "\n";
  projGPS.Reset(calibration.latitude, calibration.longitude, set.baseAltitude);
  kalman.resetX();
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

INS::state3D INS::getState3D()
{
  std::lock_guard<std::mutex> lock(mtxState3D);
  // state.pos = z; // A ENLEVER!!!
  return state;
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

INS::KalmanLinear::KalmanLinear() : t(std::chrono::steady_clock::now())
{
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

void INS::KalmanLinear::update(Eigen::Matrix<double, 6, 1> z, int hAcc, uint32_t sAcc, double dt)
{
  std::lock_guard<std::mutex> lock(mtx);

  Eigen::Matrix<double, 6, 6> R;

  double sigma_Alt = 0.2;

  R.setZero();
  // Position
  R(0, 0) = pow(hAcc * 1e-3, 2); // px
  R(1, 1) = pow(hAcc * 1e-3, 2); // py
  R(2, 2) = pow(sigma_Alt, 2);   // pz
  // Vitesse
  R(3, 3) = pow(sAcc * 1e-3, 2); // vx
  R(4, 4) = pow(sAcc * 1e-3, 2); // vy
  // R(5, 5) = (2 * pow(sigma_Alt, 2)) / (pow(dt, 2)); // vz
  R(5, 5) = pow(sAcc * 1e-3, 2);

  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();

  Eigen::Matrix<double, 6, 6> S;
  S = H * P * H.transpose() + R;

  Eigen::Matrix<double, 6, 6> K;
  K = P * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

  Eigen::Matrix<double, 6, 1> y;
  y = z - H * x;

  // double d2 = (y.transpose() * S.ldlt().solve(y))(0, 0);

  // if (d2 <= 11.070)
  x = x + K * y;
}

void INS::KalmanLinear::resetX()
{
  std::lock_guard<std::mutex> lock(mtx);
  x = Eigen::Matrix<double, 6, 1>::Zero();
}

Eigen::Matrix<double, 6, 1> INS::KalmanLinear::getX()
{
  std::lock_guard<std::mutex> lock(mtx);
  return x;
}

INS::HPfilter::HPfilter(double dt, double fc)
{
  double RC = 1 / (2 * M_PI * fc);
  a = RC / (RC + dt);
}

double INS::HPfilter::update(double X)
{
  x[1] = x[0];
  x[0] = X;
  y[1] = y[0];

  y[0] = a * (y[1] + x[0] - x[1]);
  return y[0];
}

double INS::IIRfilter::update(double x, double alpha)
{
  y = alpha * x + (1 - alpha) * y;
  return y;
}