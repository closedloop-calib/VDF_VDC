#include "HalconCalibrationData.h"

#include <array>
#include <cmath>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace halcon_benchmark {
namespace {

enum class Section { None, CameraMatrix, WorldPoints, ImageFrame, RobotPose, CameraPose };

std::array<double, 4> readNumericRow(const std::string& line, int width, int lineNumber) {
    std::istringstream row(line);
    row.imbue(std::locale::classic());
    std::array<double, 4> values{};
    for (int column = 0; column < width; ++column) {
        if (!(row >> values[column]) || !std::isfinite(values[column]))
            throw std::runtime_error("Invalid numeric value at input line " + std::to_string(lineNumber) + ".");
    }
    std::string extra;
    if (row >> extra)
        throw std::runtime_error("Unexpected extra value at input line " + std::to_string(lineNumber) + ".");
    return values;
}

void appendRow(HalconCpp::HTuple& tuple, const std::array<double, 4>& values, int width) {
    for (int column = 0; column < width; ++column)
        tuple = tuple.TupleConcat(values[column]);
}

} // namespace

CalibrationData readCalibrationData(std::istream& input) {
    CalibrationData data;
    Section section = Section::None;
    int rows = 0;
    int robotPoseCount = 0, cameraPoseCount = 0;
    bool hasCameraMatrix = false, hasWorldPoints = false;

    const auto finishSection = [&] {
        switch (section) {
        case Section::CameraMatrix:
            if (rows != 3) throw std::runtime_error("The camera matrix must have three rows.");
            break;
        case Section::WorldPoints:
            if (rows == 0) throw std::runtime_error("The calibration target has no world points.");
            data.pointsPerView = rows;
            break;
        case Section::ImageFrame:
            if (rows != data.pointsPerView || rows == 0)
                throw std::runtime_error("Image frame " + std::to_string(data.views + 1) +
                                         " does not match the world point count.");
            ++data.views;
            break;
        case Section::RobotPose:
        case Section::CameraPose:
            if (rows != 4) throw std::runtime_error("Each pose matrix must have four rows.");
            if (section == Section::RobotPose) ++robotPoseCount;
            else ++cameraPoseCount;
            break;
        case Section::None:
            break;
        }
        rows = 0;
    };

    int lineNumber = 0;
    for (std::string line; std::getline(input, line);) {
        ++lineNumber;
        if (lineNumber == 1 && line.compare(0, 3, "\xef\xbb\xbf") == 0) line.erase(0, 3);
        const auto start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos) continue;
        if (line[start] == '#') {
            std::istringstream header(line.substr(start + 1));
            std::string tag;
            header >> tag;
            Section next = Section::None;
            if (tag == "KM") next = Section::CameraMatrix;
            else if (tag == "Worldpts") next = Section::WorldPoints;
            else if (tag == "Frame") next = Section::ImageFrame;
            else if (tag == "RobotPose") next = Section::RobotPose;
            else if (tag == "CameraPose") next = Section::CameraPose;
            else if (tag != "imgPts" && tag != "BasePoseIntcpCoords" && tag != "gridPoseInCameraCoords")
                continue;

            finishSection();
            if (next == Section::CameraMatrix) {
                if (hasCameraMatrix) throw std::runtime_error("The input has more than one camera matrix.");
                hasCameraMatrix = true;
            } else if (next == Section::WorldPoints) {
                if (hasWorldPoints) throw std::runtime_error("The input has more than one world point list.");
                hasWorldPoints = true;
            }
            section = next;
            continue;
        }

        switch (section) {
        case Section::CameraMatrix:
            appendRow(data.cameraMatrix, readNumericRow(line, 4, lineNumber), 4);
            break;
        case Section::WorldPoints:
            appendRow(data.worldPoints, readNumericRow(line, 2, lineNumber), 2);
            break;
        case Section::ImageFrame:
            appendRow(data.imagePoints, readNumericRow(line, 2, lineNumber), 2);
            break;
        case Section::RobotPose:
            appendRow(data.baseInTcp, readNumericRow(line, 4, lineNumber), 4);
            break;
        case Section::CameraPose:
            appendRow(data.boardInCamera, readNumericRow(line, 4, lineNumber), 4);
            break;
        case Section::None:
            throw std::runtime_error("No data section for input line " + std::to_string(lineNumber) + ".");
        }
        ++rows;
    }

    if (input.bad()) throw std::runtime_error("An error occurred while reading the calibration input.");
    finishSection();
    if (!hasCameraMatrix || data.pointsPerView <= 0 || data.views <= 0 ||
        robotPoseCount != data.views || cameraPoseCount != data.views)
        throw std::runtime_error("Expected camera intrinsics and matching image, robot-pose and camera-pose counts.");

    return data;
}

HandEyeData prepareHandEyeData(const CalibrationData& data, int imageWidth, int imageHeight) {
    const int points = data.pointsPerView;
    const int views = data.views;
    if (points <= 0 || views <= 0 || imageWidth <= 0 || imageHeight <= 0 ||
        data.cameraMatrix.Length() != 12 ||
        static_cast<long long>(data.worldPoints.Length()) != 2LL * points ||
        static_cast<long long>(data.imagePoints.Length()) != 2LL * points * views ||
        static_cast<long long>(data.baseInTcp.Length()) != 16LL * views)
        throw std::runtime_error("Invalid calibration data dimensions.");

    HandEyeData result;
    for (int view = 0; view < views; ++view) {
        result.pointsPerView = result.pointsPerView.TupleConcat(points);
        for (int point = 0; point < points; ++point) {
            const auto offset = 2 * (view * points + point);
            result.worldX = result.worldX.TupleConcat(data.worldPoints[2 * point].D());
            result.worldY = result.worldY.TupleConcat(data.worldPoints[2 * point + 1].D());
            result.worldZ = result.worldZ.TupleConcat(0.0);
            // The input stores (u, v). HALCON accepts (row, column) = (v, u).
            result.imageRows = result.imageRows.TupleConcat(data.imagePoints[offset + 1].D());
            result.imageColumns = result.imageColumns.TupleConcat(data.imagePoints[offset].D());
        }
    }

    // Preserve the camera parameter construction used in the published benchmark.
    const double fx = data.cameraMatrix[0].D(), fy = data.cameraMatrix[5].D();
    const double cx = data.cameraMatrix[2].D(), cy = data.cameraMatrix[6].D();
    const double focalLength = (fx + fy) / 2.0;
    result.cameraParameters = HalconCpp::HTuple(focalLength).TupleConcat(0.0)
        .TupleConcat(1.0).TupleConcat(1.0).TupleConcat(cx).TupleConcat(cy)
        .TupleConcat(imageWidth).TupleConcat(imageHeight);

    for (int view = 0; view < views; ++view) {
        const auto first = 16 * view;
        const auto homMat3D = data.baseInTcp.TupleSelectRange(first, first + 11);
        HalconCpp::HTuple pose;
        HalconCpp::HomMat3dToPose(homMat3D, &pose);
        result.robotPoses = result.robotPoses.TupleConcat(pose);
    }
    return result;
}

} // namespace halcon_benchmark
