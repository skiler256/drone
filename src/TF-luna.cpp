#include "../inc/TF-luna.hpp"
#include "../inc/SensorFusion.hpp"

TFluna::TFluna(eventManager &event, std::optional<SensorFusion> &sens, const uint8_t TFluna_addr, const char *bus) : event(event), sens(sens), file(open(bus, O_RDWR))
{
    std::lock_guard<std::mutex> lock(mtx);

    if (sens)
        sens->ident(this, sensor::TEL);

    ioctl(file, I2C_SLAVE, TFluna_addr);

    char config[2] = {POWER, 0x00};
    write(file, config, 2);
    usleep(100000);
}

TFluna::~TFluna()
{
    std::lock_guard<std::mutex> lock(mtx);
    close(file);
}

void TFluna::setPower(const bool pow)
{
    std::lock_guard<std::mutex> lock(mtx);
    if (pow)
    {
        char config[2] = {POWER, 0x00};
        write(file, config, 2);
    }
    else
    {
        char config[2] = {POWER, 0x01};
        write(file, config, 2);
    }
}

double TFluna::getDist()
{
    std::lock_guard<std::mutex>
        lock(mtx);
    write(file, &DIST_LOW, 1);
    char data[2];
    read(file, data, 2);

    uint16_t raw = (uint8_t)data[0] | ((uint8_t)data[1] << 8);
    return (double)raw / 100.0;
}