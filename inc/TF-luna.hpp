#pragma once
#include <iostream>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>


const uint8_t DIST_LOW = 0x00;
const uint8_t POWER = 0x28;

class TFluna{
    public:

    TFluna(const uint8_t TFluna_addr=0x10,  const char *bus = "/dev/i2c-1" );
    ~TFluna();
    
    void setPower(const bool pow);
    double getDist();

    private :
    int file;
};