// #include <iostream>     // pour cout, endl
// #include <thread>       // pour this_thread::sleep_for
// #include <chrono>       // pour steady_clock
// #include <cmath>

// struct PID
// {
//   PID(double Ki, double Kd, double Kp, double max, double min, bool doClamping);
//   double Ki, Kd, Kp;
//    double max, min;
//    bool doClamping;

//     bool isFirstUpdate = true;

//   double prev_e = 0.0;
//   double integral = 0.0;

//   std::chrono::_V2::steady_clock::time_point t;

//   double update(double E);
// };

// PID::PID(double Ki, double Kd, double Kp, double max, double min, bool doClamping) : Ki(Ki), Kd(Kd), Kp(Kp),
// max(max), min(min), doClamping(doClamping)
// {

//   t = std::chrono::steady_clock::now();
// }

// double PID::update(double E)
// {

// auto now = std::chrono::steady_clock::now();

// std::chrono::duration<double> elapsed = now - t;

// double dt = elapsed.count(); // secondes
// t = now;

// if(isFirstUpdate){
//     isFirstUpdate = false;
//     prev_e = E;
//     return 0.0;
// }

//   double newIntegral = integral + E * dt;

//   double command =  ((Kp * E) + (Ki * newIntegral) +( Kd * ((E - prev_e) / dt)));

//   if (command > max)
//   {
//     command = max;
//     if (doClamping && E > 0)
//       newIntegral = integral;
//   }
//   else if (command < min)
//   {
//     command = min;
//     if (doClamping && E < 0)
//       newIntegral = integral;
//   }

//   integral = newIntegral;
//   prev_e = E;

//   return command;
// }

// int main()
// {
//     PID pid(0.5, 0.1, 1.0, 100, -100, true);
//     double setpoint = 50.0;
//     double y = 0.0; // état du système

//     double tau = 1.5; // constante de temps du système (inertie)
//     double dt = 0.01; // période d’échantillonnage (10 ms)

//     for (int k = 0; k < 1000; k++)
//     {
//         // Erreur
//         double e = setpoint - y;

//         // PID
//         double u = pid.update(e);

//         // Système de premier ordre : y_dot = (-y + u)/tau
//         y += dt * ((-y + u) / tau);

//         // Affichage
//         std::cout << "t=" << k * dt << "s\t"
//                   << "e=" << e << "\t"
//                   << "u=" << u << "\t"
//                   << "y=" << y << std::endl;

//         std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
//     }

//     return 0;
// }
#include "../inc/behaviorCenter.hpp"

#include <signal.h>
behaviorCenter behavior;

void handle_sigint(int signum)
{
  save();
  killNodeJS();
  std::cout << "\nArret du programme" << std::endl;
  exit(0);
}

int main()
{
  signal(SIGINT, handle_sigint);
  // behavior;

  while (true)
  {
    usleep(1000000);
  }

  return 0;
}
