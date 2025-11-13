#include "../inc/LIS3MDL.hpp"
#include <iostream>

LIS3MDL::LIS3MDL(eventManager &event, uint8_t address, const char *bus) : event(event)
{
    file = open(bus, O_RDWR);
    ioctl(file, I2C_SLAVE, address);

    th = std::thread(&LIS3MDL::run, this);
}

LIS3MDL::~LIS3MDL()
{
    loop = false;
    if (th.joinable())
        th.join();
    if (file >= 0)
        close(file);
}

std::array<double, 3> LIS3MDL::getData()
{
    std::lock_guard<std::mutex> lock(mtx);
    return mag;
}

void LIS3MDL::run()
{
    uint8_t ctrl[2] = {0x20, 0b01111000};
    write(file, &ctrl, 2);

    ctrl[0] = 0x21;
    ctrl[1] = 0b00100000;
    write(file, &ctrl, 2);

    ctrl[0] = 0x22;
    ctrl[1] = 0b00000000;
    write(file, &ctrl, 2);

    ctrl[0] = 0x23;
    ctrl[1] = 0b00001100;
    write(file, &ctrl, 2);

    constexpr double LIS3MDL_SCALE_8G = 0.29;

    while (loop)
    {

        uint8_t statusReg = 0x27;
        write(file, &statusReg, 1);
        uint8_t status;
        read(file, &status, 1);

        if (status & 0x08)
        {
            uint8_t reg = 0x28 | 0x80;
            write(file, &reg, 1);

            uint8_t data[6];
            read(file, data, 6);

            int16_t rawX = (int16_t)(data[1] << 8 | data[0]);
            int16_t rawY = (int16_t)(data[3] << 8 | data[2]);
            int16_t rawZ = (int16_t)(data[5] << 8 | data[4]);

            std::lock_guard<std::mutex> lock(mtx);
            mag[0] = rawX * LIS3MDL_SCALE_8G;
            mag[1] = rawY * LIS3MDL_SCALE_8G;
            mag[2] = rawZ * LIS3MDL_SCALE_8G;

            std::cout << "X=" << mag[0] << " mG, Y=" << mag[1] << " mG, Z=" << mag[2] << " mG\n";
        }
    }
    usleep(5000);
}