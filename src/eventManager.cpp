
// #include <chrono>

// std::chrono::_V2::steady_clock::time_point t;

// std::string toString(eventSeverity severity)
// {
//     switch (severity)
//     {
//     case eventSeverity::INFO:
//         return "INFO";
//     case eventSeverity::WARNING:
//         return "WARNING";
//     case eventSeverity::CRITICAL:
//         return "CRITICAL";
//     case eventSeverity::FATAL:
//         return "FATAL";
//     default:
//         return "UNKNOWN_SEVERITY";
//     }
// }

// std::string toString(component comp)
// {
//     switch (comp)
//     {
//     case component::GPS:
//         return "GPS";
//     case component::ESP:
//         return "ESP";
//     case component::BMP:
//         return "BMP";
//     case component::INS:
//         return "INS";
//     default:
//         return "UNKNOWN_COMPONENT";
//     }
// }

// std::string toString(subcomponent sub)
// {
//     switch (sub)
//     {
//     case subcomponent::serial:
//         return "serial";
//     case subcomponent::i2c:
//         return "i2c";
//     case subcomponent::computing:
//         return "computing";
//     case subcomponent::dataLink:
//         return "dataLink";
//     default:
//         return "UNKNOWN_SUBCOMPONENT";
//     }
// }

// eventManager::eventManager() : logTXT("/home/jules/code/drone/app/public/logs/Glog.txt")
// {
//     t = std::chrono::steady_clock::now();
// }

// void eventManager::reportEvent(const event &event)
// {
//     std::lock_guard<std::mutex> lock(mtx);

//     if (events[{event.comp, event.subcomp}].mess != event.mess || event.severity != eventSeverity::INFO)
//     {
//         // logTXT << toString(event.comp) << " " << toString(event.subcomp) << " " << toString(event.severity) << " " << event.mess << std::endl;
//     }

//     events[{event.comp, event.subcomp}] = {event.severity, event.mess};
//     if (doLog)
//     {
//         std::cout << toString(event.comp) << " " << toString(event.subcomp) << " " << toString(event.severity) << " " << event.mess << std::endl;
//     }
//     // if (event.comp == component::ESP)
//     // {
//     //     std::cout << "ESP!!!!" << std::endl;
//     //     const int temps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t).count();
//     //     std::cout << temps << std::endl;
//     //     t = std::chrono::steady_clock::now();
//     // }
// }

// void eventManager::clearEvent(const event &event)
// {
//     std::lock_guard<std::mutex> lock(mtx);
//     events.erase({event.comp, event.subcomp});
// }

// std::map<std::pair<component, subcomponent>, eventManager::eventLog> eventManager::getEvents()
// {
//     std::lock_guard<std::mutex> lock(mtx);
//     return events;
// }

// std::string stringifyEventLogMap(const std::map<std::pair<component, subcomponent>, eventManager::eventLog> &events)
// {
//     std::string result;

//     for (const auto &[key, log] : events)
//     {
//         if (log.mess.empty())
//             continue; // ignorer les logs vides

//         const auto &[comp, subcomp] = key;
//         result += toString(comp) + " | " +
//                   toString(subcomp) + " | " +
//                   toString(log.severity) + " | " +
//                   log.mess + "\n";
//     }

//     return result;
// }

// bool eventManager::hasEventAtLeast(eventSeverity minSeverity)
// {
//     std::lock_guard<std::mutex> lock(mtx);

//     for (const auto &kv : events)
//     {
//         const eventSeverity sev = kv.second.severity;
//         if (static_cast<int>(sev) >= static_cast<int>(minSeverity))
//             return true;
//     }

//     return false;
// }

#include "../inc/eventManager.hpp"
#include <iostream>

void eventManager::declare(const void *ptr, component module)
{

    std::lock_guard<std::mutex> lock(mtx);

    if (IDs.find(ptr) == IDs.end())
    {
        for (auto &kv : IDs)
        {
            if (kv.second == module)
                return;
        }

        IDs[ptr] = module;
    }
    else
        return;
}

void eventManager::erase(const void *ptr)
{
    std::lock_guard<std::mutex> lock(mtx);
    IDs.erase(ptr);
}

void eventManager::report(const void *ptr, category cat, event e)
{

    std::lock_guard<std::mutex> lock(mtx);

    events[{ptr, cat}] = e;
}

void eventManager::clear(const void *ptr, category cat)
{
    std::lock_guard<std::mutex> lock(mtx);
    events.erase({ptr, cat});
}

std::string componentToStr(component c)
{
    switch (c)
    {
    case component::GPS:
        return "GPS";
    case component::ESP:
        return "ESP";
    case component::BMP:
        return "BMP";
    case component::INS:
        return "INS";
    }
    return "?";
}

std::string categoryToStr(category c)
{
    switch (c)
    {
    case category::connection:
        return "connection";
    case category::calculation:
        return "calculation";
    case category::parser:
        return "parser";
    case category::signal:
        return "signal";
    }
    return "?";
}

std::string severityToStr(severity s)
{
    switch (s)
    {
    case severity::INFO:
        return "INFO";
    case severity::WARNING:
        return "WARNING";
    case severity::CRITICAL:
        return "CRITICAL";
    case severity::FATAL:
        return "FATAL";
    }
    return "?";
}

std::string strigifyEvents(const std::map<std::pair<const void *, category>, event> &events, const std::map<const void *, component> &IDs)
{

    std::string out;

    for (const auto &kv : events)
    {
        const void *ptr = kv.first.first;
        category cat = kv.first.second;
        const event &ev = kv.second;

        // retrouver le component
        auto it = IDs.find(ptr);
        if (it == IDs.end())
            continue; // si non déclaré, on ignore

        component comp = it->second;

        out += "[" + componentToStr(comp) + "]";
        out += "[" + categoryToStr(cat) + "]";
        out += " " + severityToStr(ev.sev);
        out += " : " + ev.msg + "\n";
    }

    return out;
}

std::map<const void *, component> &eventManager::getIDs()
{
    std::lock_guard<std::mutex> lock(mtx);
    return IDs;
}
std::map<std::pair<const void *, category>, event> &eventManager::getEvents()
{
    std::lock_guard<std::mutex> lock(mtx);
    return events;
}

bool eventManager::hasSeverityOrAbove(severity sev)
{
    std::lock_guard<std::mutex> lock(mtx);

    for (const auto &kv : events)
    {
        const event &e = kv.second;

        if (static_cast<int>(e.sev) >= static_cast<int>(sev))
            return true;
    }
    return false;
}