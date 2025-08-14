#pragma once
#include <iostream>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <mutex>

#include "../inc/eventManager.hpp"

const uint8_t DIST_LOW = 0x00;
const uint8_t POWER = 0x28;

class TFluna
{
public:
    TFluna(eventManager &event, const uint8_t TFluna_addr = 0x10, const char *bus = "/dev/i2c-4");
    ~TFluna();

    void setPower(const bool pow);
    double getDist();

private:
    eventManager &event;
    int file;

    std::mutex mtx;
};
