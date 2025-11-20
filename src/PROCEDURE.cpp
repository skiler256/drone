#include "../inc/PROCEDURE.hpp"
#include "../inc/behaviorCenter.hpp"

PROCEDURE::PROCEDURE(launcher &launch) : launch(launch) {}

void PROCEDURE::calibrateZ()
{
    if (launch.ins && launch.ins->has_value() && launch.gps && launch.baro)
    {
        if (launch.ins->value().state.INSstate != 1)
        {
            INS::CALIBRATION calibration{0, 0, 0};
            INS::settings set = launch.ins->value().getSettings();

            launch.ins->value().state.INSstate = 1;

            int GPSattempt = 0;
            while (!launch.gps->isFix() && GPSattempt < set.NGPSattempt)
            {
                GPSattempt++;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::cout << "FIX : " << launch.gps->isFix() << std::endl;
            }
            if (!launch.gps->isFix())
            {
                launch.event->report(this, category::signal, {severity::WARNING, "pas de fix3D"});
                launch.ins->value().state.INSstate = 0;
                return;
            }
            for (int i = 0; i < set.NmoyGPScalib; i++)
            {
                NEO6m::coordPaket coord = launch.gps->getGPSCoord();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                calibration.latitude += coord.latitude;
                calibration.longitude += coord.longitude;

                MS5611::Data bmp;
                bmp = launch.baro->getData();

                calibration.pressure += bmp.pressure;
            }
            calibration.latitude /= set.NmoyGPScalib;
            calibration.longitude /= set.NmoyGPScalib;
            calibration.pressure /= set.NmoyGPScalib;

            launch.ins->value().setCalibration(calibration);
            dataSave.INScal = calibration;
            save();

            launch.ins->value().state.INSstate = 2;
        }
    }
}