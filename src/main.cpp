#include "../inc/header.hpp"


int main() {
MPU9250 mpu;
IMU imu(mpu);
std::vector<double> acc;

while(true){
  acc = imu.getAcc();
  std::cout << acc[0] << " " << acc[1] << " " << acc[2] << std::endl;
  usleep(10000);
}
  return 0;
}
