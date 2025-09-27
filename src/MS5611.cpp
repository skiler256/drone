#include "../inc/MS5611.hpp"

MS5611::MS5611(eventManager &event, uint8_t address, const char *bus) : event(event)
{
    file = open(bus, O_RDWR);
    ioctl(file, I2C_SLAVE, address);

    th = std::thread(&MS5611::task, this);
}

MS5611::~MS5611()
{
    loop = false;
    if (th.joinable())
        th.join();
    if (file >= 0)
        close(file);
}

void MS5611::task()
{
    // reset
    uint8_t cmd = 0x1E;
    write(file, &cmd, 1);
    usleep(3000);

    // lecture PROM
    for (int i = 0; i < 6; i++)
    {
        uint8_t reg = 0xA2 + (i * 2);
        write(file, &reg, 1);
        uint8_t buf[2];
        read(file, buf, 2);
        C[i + 1] = (buf[0] << 8) | buf[1];
    }

    while (loop)
    {
        // D1 pression
        cmd = 0x48;
        write(file, &cmd, 1);
        usleep(10000);
        cmd = 0x00;
        write(file, &cmd, 1);
        uint8_t pbuf[3];
        read(file, pbuf, 3);
        uint32_t D1 = (pbuf[0] << 16) | (pbuf[1] << 8) | pbuf[2];

        // D2 température
        cmd = 0x58;
        write(file, &cmd, 1);
        usleep(10000);
        cmd = 0x00;
        write(file, &cmd, 1);
        uint8_t tbuf[3];
        read(file, tbuf, 3);
        uint32_t D2 = (tbuf[0] << 16) | (tbuf[1] << 8) | tbuf[2];

        // compensation
        int32_t dT = D2 - ((uint32_t)C[5] << 8);
        int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
        int64_t OFF = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT) / 128;
        int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT) / 256;

        double temp = TEMP / 100.0;
        double pres = (((D1 * SENS) / 2097152 - OFF) / 32768) / 100.0;

        // filtrage IIR
        std::lock_guard<std::mutex> lock(mtx);
        data.temperature = tempFilter.update(temp, 0.05);
        data.pressure = presFilter.update(pres, 0.05);
    }
}

MS5611::Data MS5611::getData()
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}
