#pragma once

#include "HalconCpp.h"

#include <istream>

namespace halcon_benchmark {

struct CalibrationData {
    HalconCpp::HTuple cameraMatrix;   // 3-by-4, stored row by row
    HalconCpp::HTuple worldPoints;    // Planar (x, y) pairs in metres
    HalconCpp::HTuple imagePoints;    // Undistorted (u, v) pairs, grouped by view
    HalconCpp::HTuple baseInTcp;      // Row-major 4-by-4 poses
    HalconCpp::HTuple boardInCamera;  // Row-major 4-by-4 poses
    int pointsPerView = 0;
    int views = 0;
};

struct HandEyeData {
    HalconCpp::HTuple worldX, worldY, worldZ;
    HalconCpp::HTuple imageRows, imageColumns, pointsPerView;
    HalconCpp::HTuple cameraParameters, robotPoses;
};

// Read the tagged hand_eye_calibration_input.txt file.
CalibrationData readCalibrationData(std::istream& input);

// Prepare arguments for HALCON HandEyeCalibration.
HandEyeData prepareHandEyeData(const CalibrationData& data, int imageWidth, int imageHeight);

} // namespace halcon_benchmark
