#include "../inc/eventManager.hpp"
#include <iostream>

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

void eventManager::reportEvent(const event &event)
{
    std::lock_guard<std::mutex> lock(mtx);
    events[{event.comp, event.subcomp}] = {event.severity, event.mess};
    if (doLog)
    {
        std::cout << toString(event.comp) << " " << toString(event.subcomp) << " " << toString(event.severity) << " " << event.mess << std::endl;
    }
}

void eventManager::clearEvent(const event &event)
{
    std::lock_guard<std::mutex> lock(mtx);
    events.erase({event.comp, event.subcomp});
}

std::map<std::pair<component, subcomponent>, eventManager::eventLog> eventManager::getEvents()
{
    std::lock_guard<std::mutex> lock(mtx);
    return events;
}

std::string stringifyEventLogMap(const std::map<std::pair<component, subcomponent>, eventManager::eventLog> &events)
{
    std::string result;

    for (const auto &[key, log] : events)
    {
        if (log.mess.empty())
            continue; // ignorer les logs vides

        const auto &[comp, subcomp] = key;
        result += toString(comp) + " | " +
                  toString(subcomp) + " | " +
                  toString(log.severity) + " | " +
                  log.mess + "\n";
    }

    return result;
}
