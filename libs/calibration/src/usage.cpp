#include "usage.h"
#include <iostream>

void usage::prompt(const int& code)
{
    switch (code) {
    case (USAGE):
        std::cout << "INSTRUCTIONS:" << std::endl;
        std::cout << "-- press ENTER to take images" << std::endl;
        std::cout << "-- to at least 20 chessboard images" << std::endl;
        std::cout << "-- press ESCAPE to exit capture mode" << std::endl;
        std::cout << "-- once calibration is done, see output directory for "
                     "calibration file"
                  << std::endl;
        break;
    case (CALIBRATING):
        std::cout << "-- computing calibration parameters" << std::endl;
        break;
    case (MORE_IMAGES_REQUIRED):
        std::cout << "-- take more chessboard images" << std::endl;
        break;
    case (SAVING_PARAMETERS):
        std::cout << "-- saving calibration parameters to disk" << std::endl;
        break;

    case (LOADING_CALIBRATION_PARAMETERS):
        std::cout << "-- loading calibration parameters from disk" << std::endl;
        break;
    case (FINDING_ARUCO_MARKERS):
        std::cout << "-- searching for aruco markers" << std::endl;
        break;
    default:
        std::cout << "-- well now, wasn't expecting that one bit" << std::endl;
        break;
    }
}
