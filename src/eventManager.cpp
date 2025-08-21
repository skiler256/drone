#include "../inc/eventManager.hpp"
#include <iostream>
#include <chrono>

std::chrono::_V2::steady_clock::time_point t;

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

eventManager::eventManager() : logTXT("/home/jules/code/drone/app/public/logs/Glog.txt")
{
    t = std::chrono::steady_clock::now();
}

void eventManager::reportEvent(const event &event)
{
    std::lock_guard<std::mutex> lock(mtx);

    if (events[{event.comp, event.subcomp}].mess != event.mess || event.severity != eventSeverity::INFO)
    {
        // logTXT << toString(event.comp) << " " << toString(event.subcomp) << " " << toString(event.severity) << " " << event.mess << std::endl;
    }

    events[{event.comp, event.subcomp}] = {event.severity, event.mess};
    if (doLog)
    {
        std::cout << toString(event.comp) << " " << toString(event.subcomp) << " " << toString(event.severity) << " " << event.mess << std::endl;
    }
    // if (event.comp == component::ESP)
    // {
    //     std::cout << "ESP!!!!" << std::endl;
    //     const int temps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t).count();
    //     std::cout << temps << std::endl;
    //     t = std::chrono::steady_clock::now();
    // }
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
