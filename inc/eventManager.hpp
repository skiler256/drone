

// enum class eventSeverity
// {
//     INFO,
//     WARNING,
//     CRITICAL,
//     FATAL
// };

// enum class subcomponent
// {
//     serial,
//     i2c,
//     computing,
//     dataLink,
//     parser
// };

// struct event
// {
//     component comp;
//     subcomponent subcomp;
//     eventSeverity severity;
//     std::string mess;
// };

// class eventManager
// {
// public:
//     eventManager();

//     void reportEvent(const event &event);
//     void clearEvent(const event &event);

//     struct eventLog
//     {
//         eventSeverity severity;
//         std::string mess;
//     };

//     std::map<std::pair<component, subcomponent>, eventLog> getEvents();

//     bool hasEventAtLeast(eventSeverity minSeverity);

//     bool doLog = false;

// private:
//     std::mutex mtx;
//     std::ofstream logTXT;

//     // data
//     std::map<std::pair<component, subcomponent>, eventLog> events;
// };

// std::string stringifyEventLogMap(const std::map<std::pair<component, subcomponent>, eventManager::eventLog> &events);

#pragma once
#include <mutex>
#include <string>
#include <map>
#include <utility>
#include <fstream>

enum class component
{
    GPS,
    ESP,
    BMP,
    INS,
};

enum class category
{
    connection,
    calculation,
    parser,
    signal

};

enum class severity
{
    INFO,
    WARNING,
    CRITICAL,
    FATAL
};

struct event
{
    severity sev;
    std::string msg;
};

class eventManager
{

public:
    void declare(const void *ptr, component module);
    void erase(const void *ptr);

    void report(const void *ptr, category cat, event e);
    void clear(const void *ptr, category cat);

    bool hasSeverityOrAbove(severity sev);

    std::map<const void *, component> &getIDs();
    std::map<std::pair<const void *, category>, event> &getEvents();

private:
    std::mutex mtx;
    std::map<const void *, component> IDs;

    std::map<std::pair<const void *, category>, event> events;
};

std::string strigifyEvents(const std::map<std::pair<const void *, category>, event> &events, const std::map<const void *, component> &IDs);

std::string componentToStr(component c);
std::string categoryToStr(category c);
std::string severityToStr(severity s);