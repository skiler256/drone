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

extern const char *saveFile;
extern std::mutex saveMtx;
void save(saveData data);
void save();
void save(saveData data);

class behaviorCenter
{
public:
    behaviorCenter();
    void interpretCommand(std::string_view msg);

private:
    launcher launch;
    PROCEDURE procedure;

    saveData getSave();
};