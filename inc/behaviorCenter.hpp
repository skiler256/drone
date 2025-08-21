#pragma once

#include "../inc/launcher.hpp"
#include "../inc/PROCEDURE.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <cstdint>

struct saveData
{
    INS::settings INSsettings;
    INS::CALIBRATION INScal;
};
extern saveData dataSave;

class behaviorCenter
{
public:
    behaviorCenter();
    void interpretCommand(std::string_view msg);
    void save();

private:
    launcher launch;
    PROCEDURE procedure;
    const char *saveFile = "/home/jules/save.drn";

    std::mutex saveMtx;

    void save(saveData data);

    saveData getSave();
};