#ifndef USAGE_H
#define USAGE_H

#define LOADING_CALIBRATION_PARAMETERS 20
#define FINDING_ARUCO_MARKERS 21
#define USAGE 22
#define CALIBRATING 23
#define MORE_IMAGES_REQUIRED 24
#define SAVING_PARAMETERS 25

#define ENTER_KEY 13
#define ESCAPE_KEY 27

namespace usage {
void prompt(const int& code);
}

#endif // USAGE_H
