#include "../inc/USSensor.hpp"
#define timeout 0.025

USSensor::USSensor(const int pin, GPIO &PIN) : pinTrig(pin),
                                               PIN(PIN) {}

double USSensor::measure(const int pinEcho)
{
   PIN.write(pinTrig, false);
   usleep(10);
   PIN.write(pinTrig, true);
   usleep(10);
   PIN.write(pinTrig, false);

   auto t0 = std::chrono::high_resolution_clock::now();
   while (PIN.read(pinEcho) == 0)
   {
      if (((std::chrono::high_resolution_clock::now() - t0).count() / 1e9) > 1)
         return -1;
   }
   auto t1 = std::chrono::high_resolution_clock::now();

   while (PIN.read(pinEcho) == 1)
   {
      if (((std::chrono::high_resolution_clock::now() - t1).count() / 1e9) > 1)
         return -1;
   }
   auto t2 = std::chrono::high_resolution_clock::now();

   std::chrono::duration<double> duration = t2 - t1;
   double t = duration.count();

   return t * 340.0 / 2.0;
}