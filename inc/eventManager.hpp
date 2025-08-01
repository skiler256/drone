#pragma once
#include <mutex>
#include <string>
#include <map>
#include <utility>

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
    void clearEvent(const event &event);

    struct eventLog
    {
        eventSeverity severity;
        std::string mess;
    };

private:
    std::mutex mtx;

    // data
    std::map<std::pair<component, subcomponent>, eventLog> events;
};
