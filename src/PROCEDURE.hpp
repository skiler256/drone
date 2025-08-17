#include "../inc/PROCEDURE.hpp"

PROCEDURE::PROCEDURE(launcher &launch) : launch(launch) {}

void PROCEDURE::calibrateZ()
{
    if (launch.ins)
    {
        INS::CALIBRATION calibration;
        INS::settings set = launch.ins->getSettings();
    }
}