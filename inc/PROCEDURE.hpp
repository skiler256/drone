#pragma once
#include "../inc/launcher.hpp"

#include <mutex>

class PROCEDURE
{
public:
    PROCEDURE(launcher &launch);

    void calibrateZ();

private:
    launcher &launch;
};