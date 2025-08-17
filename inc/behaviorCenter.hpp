#pragma once

#include "../inc/launcher.hpp"

#include <string>

class behaviorCenter
{
public:
    behaviorCenter();
    void interpretCommand(std::string_view msg);

private:
    launcher launch;

};