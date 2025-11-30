#include "../inc/SensorFusion.hpp"
#include "../inc/launcher.hpp"

SensorFusion::SensorFusion(launcher &lau) : lau(lau)
{
    file.open("mesures.txt", std::ios::app);

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

    switch (IDs[ptrID])
    {
    case sensor::ESP:

    {

        auto data = *static_cast<const ESPdata *>(ptrData);
        // std::cout << "aux :" << data.pitch << "\n";

        ins->updatePR(data.pitch, data.roll);
        Eigen::Matrix<double, 3, 1> rawAcc;

        rawAcc << data.ax, data.ay, data.az;

        if (ins)
            ins->updateAcc(rawAcc);

        break;
    }
    case sensor::GPS:
    {

        auto data = *static_cast<const NEO6m::gpsState *>(ptrData);
        if (ins)
            ins->updateGPS(data.coord, data.velNED, data.pAcc, data.sAcc);

        break;
    }
    case sensor::MAG:
    {
        auto data = *static_cast<const std::array<double, 3> *>(ptrData);

        Eigen::Matrix<double, 3, 1> rawMag;
        rawMag << data[0], data[1], data[2];

        Eigen::Matrix<double, 3, 1> b;
        b << 68.057306,
            -388.114582,
            -341.218969;

        Eigen::Matrix<double, 3, 3> Ainv;
        Ainv << 1.206966, 0.014500, -0.029115,
            0.014500, 1.192428, -0.018127,
            -0.029115, -0.018127, 1.199759;

        Eigen::Matrix<double, 3, 1> mag;

        mag = Ainv * (rawMag - b);

        // file << data[0] << " " << data[1] << " " << data[2] << std::endl;

        if (ins)
            ins->updateMag(mag);

        // std::cout << "X=" << mag(0) << " mG, Y=" << mag(1) << " mG, Z=" << mag(2) << " mG\n";

        break;
    }
    case sensor::BARO:
    {
        auto data = *static_cast<const MS5611::Data *>(ptrData);

        if (ins)
            ins->updateBaro(data.pressure, data.temperature);

        // std::cout << lau.monitoring->getData().state3D.pos(2) << "\n";

        break;
    }
    }
}