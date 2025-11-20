#include "../inc/behaviorCenter.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;
saveData dataSave;
const char *saveFile = "/home/jules/save.drn";
std::mutex saveMtx;

void from_json(const json &j, INS::settings &s)
{
    s.refreshRate = j.value("refreshRate", s.refreshRate);
    s.zRefreshRate = j.value("zRefreshRate", s.zRefreshRate);
    s.alphaHeading = j.value("alphaHeading", s.alphaHeading);
    s.NGPSattempt = j.value("NGPSattempt", s.NGPSattempt);
    s.NmoyGPScalib = j.value("NmoyGPScalib", s.NmoyGPScalib);
    s.baseAltitude = j.value("baseAltitude", s.baseAltitude);
}

behaviorCenter::behaviorCenter() : launch(*this), procedure(launch)
{
    // launch.startCOM(); integrer au constructeur
}

void behaviorCenter::interpretCommand(std::string_view msg)
{
    std::cout << std::string(msg) << std::endl;
    std::string command = std::string(msg);
    if (command.size() > 3)
    {
        std::string ID = command.substr(0, 3);
        std::string commandCore = command.erase(0, 3);
        if (ID == "GIM" && launch.gimball)
            launch.gimball->doCommand(commandCore);
        else if (ID == "LAU")
        {
            if (commandCore == "INS")
            {
                launch.monitoring->data.moduleState.INS ^= true;
                if (launch.monitoring->data.moduleState.INS && launch.ins)
                    launch.startINS();
                else if (launch.ins) // vérifie si le pointeur existe
                    launch.ins->reset();
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
                {
                    launch.esp.reset();
                    launch.mag.reset();
                }
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
        else if (ID == "PRO")
        {
            if (commandCore == "CALZ")
                std::thread(&PROCEDURE::calibrateZ, procedure).detach();
            else if (commandCore == "USEINSSAVE" && launch.ins && launch.ins->has_value())
            {
                saveData save_ = getSave();
                launch.ins->value().setCalibration(save_.INScal);
                launch.ins->value().setSettings(save_.INSsettings);
                launch.ins->value().state.INSstate = 2;
                dataSave = save_;
                save();
            }
        }
        else if (ID == "IPA")
        {
            json j = json::parse(commandCore);
            INS::settings set = j.get<INS::settings>();
            dataSave.INSsettings = set;
            save();
            if (launch.ins && launch.ins->has_value())
                launch.ins->value().setSettings(set);
        }
    }
}

void write(std::ofstream &file, int addr, uint8_t c)
{
    file.seekp(addr, std::ios::beg);
    file.put(c);
}

uint8_t read(std::ifstream &file, int addr)
{
    file.seekg(addr, std::ios::beg);
    return file.get();
}

void save(saveData data)
{
    std::lock_guard<std::mutex> lock(saveMtx);
    std::ofstream file(saveFile, std::ios::trunc);
    write(file, 0, 0x24);
    write(file, 1, 0x09);

    uint8_t *ptr = reinterpret_cast<uint8_t *>(&data);
    size_t size = sizeof(saveData);

    for (size_t i = 0; i < size; ++i)
    {
        write(file, static_cast<int>(i + 2), ptr[i]);
    }
    file.close();
}

void save()
{
    std::lock_guard<std::mutex> lock(saveMtx);
    std::ofstream file(saveFile, std::ios::trunc);
    write(file, 0, 0x24);
    write(file, 1, 0x09);

    uint8_t *ptr = reinterpret_cast<uint8_t *>(&dataSave);
    size_t size = sizeof(saveData);

    for (size_t i = 0; i < size; ++i)
    {
        write(file, static_cast<int>(i + 2), ptr[i]);
    }
    file.close();
}

saveData behaviorCenter::getSave()
{
    saveData save;
    std::lock_guard<std::mutex> lock(saveMtx);
    std::ifstream file(saveFile, std::ios::binary | std::ios::ate);

    if (file.is_open())
    {
        std::streamsize size = file.tellg();
        if ((int)size != ((int)sizeof(saveData) + 2))
            return save;
    }
    if (read(file, 0) != 0x24 || read(file, 1) != 0x09)
        return save;

    // Lire le reste du fichier (données)
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&save);
    for (size_t i = 0; i < sizeof(saveData); ++i)
    {
        ptr[i] = read(file, static_cast<int>(i + 2));
    }

    file.close();
    return save;
}