#pragma once
#include <mutex>
#include <string>

enum class eventSeverity
{
    INFO,
    WARNING,
    CRITICAL,
    FATAL
};

enum class component
{
    GPS,
    ESP,
    BMP,
    INS
};

enum class subcomponent
{
    serial,
    i2c,
    computing,
    dataLink
};

struct event
{
    component comp;
    subcomponent subcomp;
    eventSeverity severity;
    std::string mess;
};

class eventManager
{
public:
    void reportEvent(const event &event);

private:
    std::mutex mtx;
};