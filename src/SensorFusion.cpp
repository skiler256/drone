#include "../inc/SensorFusion.hpp"
#include "../inc/launcher.hpp"

SensorFusion::SensorFusion(launcher &lau) : lau(lau)
{
    thMonitoring = std::thread(&SensorFusion::Monitoring, this);
}

SensorFusion::~SensorFusion()
{
    loop = false;

    if (thMonitoring.joinable())
        thMonitoring.join();
}

void SensorFusion::ident(const void *ptr, sensor sen)
{
    std::lock_guard<std::mutex> lock(mtx);
    IDs[ptr] = sen;
}

std::optional<INS> *SensorFusion::getINSptr()
{
    return &ins;
}

void SensorFusion::Monitoring()
{

    while (loop)
    {
        usleep(5000);
    }
}

void SensorFusion::update(const void *ptrID, const void *ptrData)
{
    std::lock_guard<std::mutex> lock(mtx);

    auto sen = IDs[ptrID];
}