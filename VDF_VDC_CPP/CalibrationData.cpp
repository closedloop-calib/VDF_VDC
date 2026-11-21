#include "CalibrationData.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace vdf_vdc {
namespace {

enum class Section { None, CameraMatrix, WorldPoints, ImagePoints, RobotPoses, CameraPoses };

Eigen::RowVectorXd readRow(const std::string& line, Eigen::Index columns) {
    Eigen::RowVectorXd values(columns);
    std::istringstream stream(line);
    stream.imbue(std::locale::classic());
    for (Eigen::Index column = 0; column < columns; ++column) {
        if (!(stream >> values[column])) throw std::runtime_error("Invalid calibration data row: " + line);
    }
    std::string extra;
    if (stream >> extra || !values.allFinite())
        throw std::runtime_error("Invalid calibration data row: " + line);
    return values;
}

void appendPoints(const std::vector<Eigen::Vector2d>& points, Eigen::MatrixXd& matrix) {
    matrix.resize(static_cast<Eigen::Index>(points.size()), 2);
    for (Eigen::Index i = 0; i < matrix.rows(); ++i)
        matrix.row(i) = points[static_cast<std::size_t>(i)].transpose();
}

Eigen::Matrix3d closestRotation(const Eigen::Matrix3d& matrix) {
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0)
        correction(2, 2) = -1.0;
    return svd.matrixU() * correction * svd.matrixV().transpose();
}

} // namespace

std::string inputPathToUtf8(const InputPath& path) {
#ifdef _WIN32
    if (path.empty()) return {};
    const int size = WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (size <= 0) throw std::runtime_error("Cannot convert the input path to UTF-8.");
    std::string utf8(static_cast<std::size_t>(size), '\0');
    if (WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, &utf8[0], size, nullptr, nullptr) == 0)
        throw std::runtime_error("Cannot convert the input path to UTF-8.");
    utf8.pop_back();
    return utf8;
#else
    return path;
#endif
}

CalibrationData readCalibrationData(const InputPath& inputFile) {
    std::ifstream input(inputFile.c_str());
    if (!input) throw std::runtime_error("Cannot open input file: " + inputPathToUtf8(inputFile));

    CalibrationData data;
    Section section = Section::None;
    std::vector<Eigen::Vector2d> worldPoints;
    std::vector<Eigen::Vector2d> framePoints;
    Eigen::Matrix4d pose;
    int cameraMatrixRow = 0;
    int poseRow = 4;
    bool hasImageFrame = false;

    const auto finishImageFrame = [&] {
        if (!hasImageFrame) return;
        if (framePoints.empty()) throw std::runtime_error("An image frame has no feature points.");
        data.imagePoints.emplace_back();
        appendPoints(framePoints, data.imagePoints.back());
        framePoints.clear();
        hasImageFrame = false;
    };

    for (std::string line; std::getline(input, line);) {
        if (line.compare(0, 3, "\xef\xbb\xbf") == 0) line.erase(0, 3);
        const auto start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos) continue;
        if (line[start] == '#') {
            std::istringstream tagStream(line.substr(start + 1));
            std::string tag;
            tagStream >> tag;
            if (tag == "KM") section = Section::CameraMatrix;
            else if (tag == "Worldpts") section = Section::WorldPoints;
            else if (tag == "imgPts") {
                finishImageFrame();
                section = Section::ImagePoints;
            } else if (tag == "Frame") {
                finishImageFrame();
                section = Section::ImagePoints;
                hasImageFrame = true;
            } else if (tag == "BasePoseIntcpCoords" || tag == "gridPoseInCameraCoords") {
                finishImageFrame();
                if (poseRow != 4) throw std::runtime_error("Incomplete pose matrix.");
                section = Section::None;
            } else if (tag == "RobotPose" || tag == "CameraPose") {
                finishImageFrame();
                if (poseRow != 4) throw std::runtime_error("Incomplete pose matrix.");
                section = tag == "RobotPose" ? Section::RobotPoses : Section::CameraPoses;
                poseRow = 0;
            }
            continue;
        }
        switch (section) {
        case Section::CameraMatrix:
            if (cameraMatrixRow == 3) throw std::runtime_error("Too many camera matrix rows.");
            data.cameraMatrix.row(cameraMatrixRow++) = readRow(line, 4);
            break;
        case Section::WorldPoints:
            worldPoints.emplace_back(readRow(line, 2).transpose());
            break;
        case Section::ImagePoints:
            if (!hasImageFrame) throw std::runtime_error("An image frame is missing its tag.");
            framePoints.emplace_back(readRow(line, 2).transpose());
            break;
        case Section::RobotPoses:
        case Section::CameraPoses:
            if (poseRow == 4) throw std::runtime_error("Too many pose matrix rows.");
            pose.row(poseRow++) = readRow(line, 4);
            if (poseRow == 4)
                (section == Section::RobotPoses ? data.baseInTcp : data.boardInCamera).push_back(pose);
            break;
        case Section::None:
            throw std::runtime_error("Unexpected calibration data row: " + line);
        }
    }
    finishImageFrame();
    if (cameraMatrixRow != 3 || poseRow != 4)
        throw std::runtime_error("Incomplete camera or pose matrix.");
    appendPoints(worldPoints, data.worldPoints);
    validateCalibrationData(data);
    return data;
}

void validateCalibrationData(const CalibrationData& data) {
    const auto numFrames = data.baseInTcp.size();
    if (numFrames < 3 || data.boardInCamera.size() != numFrames || data.imagePoints.size() != numFrames)
        throw std::invalid_argument("At least three matching robot poses, camera poses and image frames are required.");
    if (data.worldPoints.rows() == 0 || data.worldPoints.cols() != 2 || !data.worldPoints.allFinite())
        throw std::invalid_argument("World points must contain finite planar XY coordinates.");
    const auto& cameraMatrix = data.cameraMatrix;
    if (!cameraMatrix.allFinite() || cameraMatrix(0, 0) <= 0.0 || cameraMatrix(1, 1) <= 0.0 ||
        cameraMatrix(0, 1) != 0.0 || cameraMatrix(1, 0) != 0.0 ||
        cameraMatrix.row(2) != (Eigen::RowVector4d() << 0.0, 0.0, 1.0, 0.0).finished() ||
        !cameraMatrix.col(3).isZero(0.0))
        throw std::invalid_argument("Expected pinhole intrinsics [K, 0] with zero skew.");
    for (std::size_t frame = 0; frame < numFrames; ++frame) {
        const auto& imagePoints = data.imagePoints[frame];
        if (imagePoints.rows() != data.worldPoints.rows() || imagePoints.cols() != 2 ||
            !imagePoints.allFinite() || !data.baseInTcp[frame].allFinite() || !data.boardInCamera[frame].allFinite())
            throw std::invalid_argument("Pose or image dimensions or values are invalid at frame " +
                                        std::to_string(frame + 1) + ".");
    }
}

// Kronecker-product method of Aiguo Li, Lin Wang and Defeng Wu.
// Matches the linear system in method.Li (Mili Shah, July 2014).
Calibration Li(const std::vector<Eigen::Matrix4d>& baseInTcp,
               const std::vector<Eigen::Matrix4d>& boardInCamera) {
    if (baseInTcp.size() != boardInCamera.size() || baseInTcp.size() < 3)
        throw std::invalid_argument("Li requires at least three matching pose pairs.");
    const auto n = static_cast<Eigen::Index>(baseInTcp.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12 * n, 24);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(12 * n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto& robotPose = baseInTcp[i];
        const auto& cameraPose = boardInCamera[i];
        const auto Ra = robotPose.topLeftCorner<3, 3>();
        const auto Rb = cameraPose.topLeftCorner<3, 3>();
        const auto tb = cameraPose.topRightCorner<3, 1>();
        const auto row = 12 * i;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c)
                A.block<3, 3>(row + 3 * r, 3 * c).diagonal().setConstant(Ra(r, c));
            A.block<3, 3>(row + 3 * r, 9 + 3 * r) = -Rb.transpose();
            A.block<1, 3>(row + 9 + r, 9 + 3 * r) = tb.transpose();
        }
        A.block<3, 3>(row + 9, 18) = -Ra;
        A.block<3, 3>(row + 9, 21).setIdentity();
        b.segment<3>(row + 9) = robotPose.topRightCorner<3, 1>();
    }

    const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    using RowMajorMatrix3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    Calibration result{Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity()};
    result.boardInBase.topLeftCorner<3, 3>() = closestRotation(Eigen::Map<const RowMajorMatrix3>(x.data()));
    result.cameraInTcp.topLeftCorner<3, 3>() = closestRotation(Eigen::Map<const RowMajorMatrix3>(x.data() + 9));
    result.boardInBase.topRightCorner<3, 1>() = x.segment<3>(18);
    result.cameraInTcp.topRightCorner<3, 1>() = x.segment<3>(21);
    return result;
}

static Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S << 0, -w.z(), w.y(),
        w.z(), 0, -w.x(),
        -w.y(), w.x(), 0;
    return S;
}

Eigen::Matrix4d invertSE3(const Eigen::Matrix4d& T) {
    Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();
    Ti.topLeftCorner<3, 3>() = R.transpose();
    Ti.topRightCorner<3, 1>() = -R.transpose() * t;
    return Ti;
}

Matrix6 adjointSE3(const Eigen::Matrix4d& T) {
    Matrix6 Ad; Ad.setZero();
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();
    Ad.topLeftCorner<3, 3>() = R;
    Ad.bottomRightCorner<3, 3>() = R;
    Ad.bottomLeftCorner<3, 3>() = skewMatrix(t) * R;
    return Ad;
}

Eigen::Matrix4d se3Exp(const Vector6& xi) {
    const Eigen::Vector3d w = xi.head<3>();
    const Eigen::Vector3d v = xi.tail<3>();
    const double th = w.norm();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
    if (th >= 1e-10) {
        const Eigen::Matrix3d w_hat = skewMatrix(w);
        R = Eigen::AngleAxisd(th, w / th).toRotationMatrix();
        const double A = std::sin(th) / th;
        const double B = (1.0 - std::cos(th)) / (th * th);
        const double C = (1.0 - A) / (th * th);
        V = Eigen::Matrix3d::Identity() + B * w_hat + C * (w_hat * w_hat);
    }
    const Eigen::Vector3d t = V * v;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;
    return T;
}

Vector6 se3Log(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();

    double c = (R.trace() - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    const double th = std::acos(c);

    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity();

    if (std::abs(th) < 1e-10) {
        // w ~ 0, V_inv ~ I
    }
    else {
        const Eigen::Matrix3d w_hat = (R - R.transpose()) * (th / (2.0 * std::sin(th)));
        w << w_hat(2, 1), w_hat(0, 2), w_hat(1, 0);
        const double A = std::sin(th) / th;
        const double B = (1.0 - std::cos(th)) / (th * th);
        V_inv = Eigen::Matrix3d::Identity()
            - 0.5 * w_hat
            + (1.0 / (th * th)) * (1.0 - A / (2.0 * B)) * (w_hat * w_hat);
    }
    const Eigen::Vector3d v = V_inv * t;
    Vector6 xi;
    xi.head<3>() = w;
    xi.tail<3>() = v;
    return xi;
}

Matrix6 rightJacobianInverseSE3(const Vector6& xi) {
    const Eigen::Vector3d w = xi.head<3>();
    const Eigen::Vector3d v = xi.tail<3>();
    const Eigen::Matrix3d W = skewMatrix(w);
    const Eigen::Matrix3d V = skewMatrix(v);
    Matrix6 ad = Matrix6::Zero();
    ad.topLeftCorner<3, 3>() = W;
    ad.topRightCorner<3, 3>().setZero();
    ad.bottomLeftCorner<3, 3>() = V;
    ad.bottomRightCorner<3, 3>() = W;
    return Matrix6::Identity() + 0.5 * ad + (1.0 / 12.0) * (ad * ad);
}

Eigen::MatrixXd projectPose(const Eigen::Matrix4d& T,
    const Eigen::MatrixXd& homogeneousWorldPoints,
    const CameraMatrix& cameraMatrix) {
    // 3xM
    Eigen::MatrixXd Pc = cameraMatrix * (T * homogeneousWorldPoints);

    Eigen::ArrayXd z = Pc.row(2).array();
    Pc.row(0).array() /= z;
    Pc.row(1).array() /= z;
    // Mx2
    Eigen::MatrixXd proj(Pc.cols(), 2);
    proj.col(0) = Pc.row(0).transpose();
    proj.col(1) = Pc.row(1).transpose();
    return proj;
}

void pixelNormalEquations(const Eigen::Matrix4d& T,
    const Eigen::MatrixXd& imagePoints, // Mx2
    const Eigen::MatrixXd& homogeneousWorldPoints,// 4xM
    const CameraMatrix& cameraMatrix,
    Matrix6& JtJ,
    Vector6& Jtr,
    double& cost)
{
    const int M = static_cast<int>(imagePoints.rows());
    if (M == 0) {
        JtJ.setZero();
        Jtr.setZero();
        cost = 0.0;
        return;
    }
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();
    const Eigen::MatrixXd X = homogeneousWorldPoints.topRows(3);            // 3xM
    const Eigen::MatrixXd P = R * X + t.replicate(1, M);     // 3xM

    const Eigen::VectorXd x = P.row(0).transpose();
    const Eigen::VectorXd y = P.row(1).transpose();
    const Eigen::VectorXd z = P.row(2).transpose();

    const double fx = cameraMatrix(0, 0), fy = cameraMatrix(1, 1), cx = cameraMatrix(0, 2), cy = cameraMatrix(1, 2);

    const Eigen::VectorXd u = fx * (x.array() / z.array()).matrix() + Eigen::VectorXd::Constant(M, cx);
    const Eigen::VectorXd v = fy * (y.array() / z.array()).matrix() + Eigen::VectorXd::Constant(M, cy);

    const Eigen::VectorXd resx = u - imagePoints.col(0);
    const Eigen::VectorXd resy = v - imagePoints.col(1);

    const Eigen::VectorXd a = fx * (1.0 / z.array()).matrix();
    const Eigen::VectorXd b = -fx * (x.array() / (z.array() * z.array())).matrix();
    const Eigen::VectorXd c = fy * (1.0 / z.array()).matrix();
    const Eigen::VectorXd d = -fy * (y.array() / (z.array() * z.array())).matrix();

    const Eigen::MatrixXd RX = R * X;                   // 3xM
    const Eigen::VectorXd uRX = RX.row(0).transpose();
    const Eigen::VectorXd vRX = RX.row(1).transpose();
    const Eigen::VectorXd wRX = RX.row(2).transpose();

    Eigen::MatrixXd G1(M, 3);
    G1.col(0) = b.array() * vRX.array();                       // b*v
    G1.col(1) = a.array() * wRX.array() - b.array() * uRX.array(); // a*w - b*u
    G1.col(2) = -a.array() * vRX.array();

    Eigen::MatrixXd G2(M, 3);
    G2.col(0) = -c.array() * wRX.array() + d.array() * vRX.array();
    G2.col(1) = -d.array() * uRX.array();
    G2.col(2) = c.array() * uRX.array();

    Eigen::MatrixXd H1(M, 3);
    H1 = a.replicate(1, 3).array() * R.row(0).replicate(M, 1).array()
        + b.replicate(1, 3).array() * R.row(2).replicate(M, 1).array();

    Eigen::MatrixXd H2(M, 3);
    H2 = c.replicate(1, 3).array() * R.row(1).replicate(M, 1).array()
        + d.replicate(1, 3).array() * R.row(2).replicate(M, 1).array();

    const Eigen::Matrix3d GtG = G1.transpose() * G1 + G2.transpose() * G2;
    const Eigen::Matrix3d GtH = G1.transpose() * H1 + G2.transpose() * H2;
    const Eigen::Matrix3d HtH = H1.transpose() * H1 + H2.transpose() * H2;

    const Eigen::Vector3d Gtr = G1.transpose() * resx + G2.transpose() * resy;
    const Eigen::Vector3d Htr = H1.transpose() * resx + H2.transpose() * resy;

    JtJ.topLeftCorner<3, 3>() = GtG;
    JtJ.topRightCorner<3, 3>() = GtH;
    JtJ.bottomLeftCorner<3, 3>() = GtH.transpose();
    JtJ.bottomRightCorner<3, 3>() = HtH;

    Jtr.head<3>() = Gtr;
    Jtr.tail<3>() = Htr;

    cost = 0.5 * (resx.squaredNorm() + resy.squaredNorm());
}

} // namespace vdf_vdc
