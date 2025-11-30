#include "../inc/INS.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <thread>
#include <random>

INS::INS(eventManager &event, INS::settings &set)
    : event(event), set(set),
      HPxacc(0.005, 1),
      HPyacc(0.005, 1),
      HPzacc(0.005, 1),
      kalman()
{
  event.declare(this, component::INS);

  tz = std::chrono::steady_clock::now();
}

INS::~INS()
{
  event.erase(this);
}

void INS::updateAcc(Eigen::Matrix<double, 3, 1> rawAcc)
{
  double pitch, roll, yaw;

  { // recuperer les angles
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

  Eigen::Matrix<double, 3, 3> R = Rz_inv * Ry_inv * Rx_inv; // rotations

  linearizedAcc = R * rawAcc;
  linearizedAcc(2) -= 1;
  linearizedAcc(1) *= -1; // inverser axe y --> NED

  linearizedAcc *= 9.81;
  linearizedAcc = linearizedAcc.unaryExpr([](double v)
                                          { return std::round(v * 10.0) / 10.0; });
  linearizedAcc(2) -= 0.1;
  linearizedAcc(0) = HPxacc.update(linearizedAcc(0));
  linearizedAcc(1) = HPyacc.update(linearizedAcc(1));
  linearizedAcc(2) = HPzacc.update(linearizedAcc(2)); // filtrage

  kalman.pred(linearizedAcc);

  updateState3D();
}

void INS::updateState3D()
{
  std::lock_guard<std::mutex> lock(mtxState3D);
  auto x = kalman.getX();
  state.pos << x(0), x(1), currAlt;
  state.vel << x(3), x(4), x(5);
  state.accNED = linearizedAcc;
}

void INS::updatePR(double pitch, double roll)
{
  std::lock_guard<std::mutex> lock(mtxState3D);
  state.att(1) = pitch;
  state.att(0) = roll;
}

void INS::updateMag(Eigen::Matrix<double, 3, 1> mag)
{

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

  mag = R * mag;

  double heading =
      std::atan2(mag(1), mag(0));
  heading *= 180.0 / M_PI;
  // if (heading < 0)
  //     heading += 360.0;
  // std::cout << heading << "\n";

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

  // !! revoir pour PID
}

void INS::updateBaro(double press, double temp)
{
  std::lock_guard<std::mutex> lock(mtxZ);

  double alt = ((temp + 273.15) / 0.0065) * (pow(calibration.pressure / press, 0.1903) - 1);
  // alt = 44330.0 * (1.0 - pow(press / calibration.pressure, 0.1903)) + set.baseAltitude;
  alt = altIIR.update(alt, 0.5);
  alt = std::round(alt * 10) / 10;

  currAlt = alt;

  // std::cout << alt << "\n";
}

void INS::updateGPS(NEO6m::coordPaket coord, int velNED[3], uint32_t pAcc, uint32_t sAcc)
{
  std::lock_guard<std::mutex> lock(mtxZ);
  const double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - tz)
                        .count() *
                    1e-6;
  tz = std::chrono::steady_clock::now();

  Eigen::Matrix<double, 6, 1> z;

  projGPS.Forward(coord.latitude, coord.longitude, currAlt + set.baseAltitude, z(1), z(0), z(2));

  z(3) = velNED[0] * 1e-2;
  z(4) = velNED[1] * 1e-2;
  z(5) = velNED[2] * 1e-2;

  kalman.update(z, pAcc, sAcc, dt);
  // state.pos << z(0), z(1), z(2);

  updateState3D();
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

  std::cout << std::fixed << std::setprecision(3); // 3 décimales, lisible

  std::cout << "\n===== INS State =====\n";

  std::cout << "Position (m)  : "
            << "X=" << state.pos(0) << "  "
            << "Y=" << state.pos(1) << "  "
            << "Z=" << state.pos(2) << "\n";

  std::cout << "Vitesse (m/s) : "
            << "VX=" << state.vel(0) << "  "
            << "VY=" << state.vel(1) << "  "
            << "VZ=" << state.vel(2) << "\n";

  std::cout << "Acc NED (m/s²): "
            << "N=" << state.accNED(0) << "  "
            << "E=" << state.accNED(1) << "  "
            << "D=" << state.accNED(2) << "\n";

  std::cout << "Attitude (rad): "
            << "Roll=" << state.att(0) << "  "
            << "Pitch=" << state.att(1) << "  "
            << "Yaw=" << state.att(2) << "\n";

  std::cout << "INS state     : " << (int)state.INSstate << "\n";

  std::cout << "=====================\n";
}

INS::state3D INS::getState3D()
{
  std::lock_guard<std::mutex> lock(mtxState3D);
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