#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace vdf_vdc {

using CameraMatrix = Eigen::Matrix<double, 3, 4>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

struct CalibrationData {
    CameraMatrix cameraMatrix;
    Eigen::MatrixXd worldPoints;
    std::vector<Eigen::MatrixXd> imagePoints;
    std::vector<Eigen::Matrix4d> baseInTcp;
    std::vector<Eigen::Matrix4d> boardInCamera;
};

// Transform convention: A_i * boardInBase = cameraInTcp * B_i.
struct Calibration {
    Eigen::Matrix4d boardInBase;
    Eigen::Matrix4d cameraInTcp;
};

// Native paths preserve Unicode filenames in Visual Studio on Windows.
#ifdef _WIN32
using InputPath = std::wstring;
#else
using InputPath = std::string;
#endif

std::string inputPathToUtf8(const InputPath& path);
CalibrationData readCalibrationData(const InputPath& inputFile);
void validateCalibrationData(const CalibrationData& data);
Calibration Li(const std::vector<Eigen::Matrix4d>& baseInTcp,
               const std::vector<Eigen::Matrix4d>& boardInCamera);

Eigen::Matrix4d se3Exp(const Vector6& xi);
Vector6 se3Log(const Eigen::Matrix4d& transform);
Eigen::Matrix4d invertSE3(const Eigen::Matrix4d& transform);
Matrix6 adjointSE3(const Eigen::Matrix4d& transform);
Matrix6 rightJacobianInverseSE3(const Vector6& xi);

Eigen::MatrixXd projectPose(const Eigen::Matrix4d& transform,
                            const Eigen::MatrixXd& homogeneousWorldPoints,
                            const CameraMatrix& cameraMatrix);
void pixelNormalEquations(const Eigen::Matrix4d& transform,
                          const Eigen::MatrixXd& imagePoints,
                          const Eigen::MatrixXd& homogeneousWorldPoints,
                          const CameraMatrix& cameraMatrix,
                          Matrix6& JtJ, Vector6& Jtr, double& cost);

} // namespace vdf_vdc
