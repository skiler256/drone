#include "../inc/TF-luna.hpp"

TFluna::TFluna(const uint8_t TFluna_addr,  const char *bus ):
file(open(bus, O_RDWR))
{
    ioctl(file, I2C_SLAVE, TFluna_addr);

    char config[2] = {POWER, 0x00};
    write(file, config, 2);
    usleep(100000);
}

TFluna::~TFluna(){
    close(file);
}

void TFluna::setPower(const bool pow){
    if(pow){
        char config[2] = {POWER, 0x00};
        write(file, config, 2);
    }
    else{
        char config[2] = {POWER, 0x01};
        write(file, config, 2);
    }
}

double TFluna::getDist(){
    write(file, &DIST_LOW, 1);
    char data[2];
    read(file, data, 2);

    uint16_t raw = (uint8_t)data[0] | ((uint8_t)data[1] << 8);
    return (double) raw  / 100.0;
}