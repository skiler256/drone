#include "../inc/eventManager.hpp"

void eventManager::reportEvent(const event &event)
{
    std::lock_guard<std::mutex> lock(mtx);
    events[{event.comp, event.subcomp}] = {event.severity, event.mess};
}

void eventManager::clearEvent(const event &event)
{
    std::lock_guard<std::mutex> lock(mtx);
    events.erase({event.comp, event.subcomp});
}

std::string toString(eventSeverity severity)
{
    switch (severity)
    {
    case eventSeverity::INFO:
        return "INFO";
    case eventSeverity::WARNING:
        return "WARNING";
    case eventSeverity::CRITICAL:
        return "CRITICAL";
    case eventSeverity::FATAL:
        return "FATAL";
    default:
        return "UNKNOWN_SEVERITY";
    }
}

std::string toString(component comp)
{
    switch (comp)
    {
    case component::GPS:
        return "GPS";
    case component::ESP:
        return "ESP";
    case component::BMP:
        return "BMP";
    case component::INS:
        return "INS";
    default:
        return "UNKNOWN_COMPONENT";
    }
}

std::string toString(subcomponent sub)
{
    switch (sub)
    {
    case subcomponent::serial:
        return "serial";
    case subcomponent::i2c:
        return "i2c";
    case subcomponent::computing:
        return "computing";
    case subcomponent::dataLink:
        return "dataLink";
    default:
        return "UNKNOWN_SUBCOMPONENT";
    }
}