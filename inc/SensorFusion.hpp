#pragma once
#include <optional>
#include <map>
#include "../inc/INS.hpp"
#include <fstream>

class launcher;

enum class sensor
{
    ESP,
    GPS,
    MAG,
    MPUaux,
    BARO,
    TEL
};

class SensorFusion
{
public:
    SensorFusion(launcher &lau);
    ~SensorFusion();

    std::optional<INS> *getINSptr();

    void ident(const void *ptr, sensor sen);

    void update(const void *ptrID, const void *ptrData);

private:
    launcher &lau;

    std::ofstream file;

    std::mutex mtx;

    std::optional<INS> ins;
    std::thread thMonitoring;
    std::atomic<bool> loop = true;

    std::map<const void *, sensor> IDs;

    void Monitoring();
};