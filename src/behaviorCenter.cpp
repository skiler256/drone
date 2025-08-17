#include "../inc/behaviorCenter.hpp"

behaviorCenter::behaviorCenter() : launch(*this)
{
    launch.startCOM();
}

void behaviorCenter::interpretCommand(std::string_view msg)
{
    std::cout << std::string(msg) << std::endl;
    std::string command = std::string(msg);
    if (command.size() > 3)
    {
        std::string ID = command.substr(0, 3);
        std::string commandCore = command.erase(0, 3);
        if (ID == "GIM" || launch.gimball)
            launch.gimball->doCommand(commandCore);
        if (ID == "LAU")
        {
            if (commandCore == "INS")
            {
                launch.monitoring->data.moduleState.INS ^= true;
                if (launch.monitoring->data.moduleState.INS)
                    launch.startINS();
                else
                    launch.ins.reset();
            }
            if (commandCore == "GPS")
            {
                launch.monitoring->data.moduleState.GPS ^= true;
                if (launch.monitoring->data.moduleState.GPS)
                    launch.startGPS();
                else
                    launch.gps.reset();
            }
            if (commandCore == "BMP")
            {
                launch.monitoring->data.moduleState.BMP ^= true;
                if (launch.monitoring->data.moduleState.BMP)
                    launch.startBARO();
                else
                    launch.baro.reset();
            }
            if (commandCore == "ESP")
            {
                launch.monitoring->data.moduleState.ESP ^= true;
                if (launch.monitoring->data.moduleState.ESP)
                    launch.startESP();
                else
                    launch.esp.reset();
            }
            if (commandCore == "TEL")
            {
                launch.monitoring->data.moduleState.TEL ^= true;
                if (launch.monitoring->data.moduleState.TEL)
                    launch.startTele();
                else
                    launch.tele.reset();
            }
            if (commandCore == "GIM")
            {
                launch.monitoring->data.moduleState.GIM ^= true;
                if (launch.monitoring->data.moduleState.GIM)
                    launch.startGIMBALL();
                else
                    launch.gimball.reset();
            }
        }
    }
}