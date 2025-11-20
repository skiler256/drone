#pragma once
#include <optional>
#include <map>
#include "../inc/INS.hpp"

class launcher;

enum class sensor
{
    ESP,
    GPS,
    MAG,
    MPUaux,
    BARO
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

    std::mutex mtx;

    std::optional<INS> ins;
    std::thread thMonitoring;
    std::atomic<bool> loop = true;

    std::map<const void *, sensor> IDs;

    void Monitoring();
};