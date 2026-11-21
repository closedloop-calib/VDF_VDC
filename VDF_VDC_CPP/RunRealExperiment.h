#pragma once

#include "VDF_VDC.h"

#include <array>
#include <ostream>

namespace vdf_vdc {

struct BenchmarkResult {
    double timeMeanMs = 0.0;
    double timeStdMs = 0.0;
    double peakMemoryIncreaseMB = 0.0;
    double peakProcessMemoryMB = 0.0;
    bool hasMemoryMeasurement = false;
    Calibration calibration{};
    SolverStats solverStats;
};

constexpr std::array<SolverMode, 4> benchmarkModes{
    SolverMode::DenseFull, SolverMode::SparseFull,
    SolverMode::DenseSchur, SolverMode::SparseSchur
};

constexpr int warmupRuns = 1;
constexpr int timedRuns = 100;

BenchmarkResult benchmarkCalibration(const CalibrationData& data, SolverMode mode);

void printComputationalCostComparison(
    std::ostream& output, const std::array<BenchmarkResult, benchmarkModes.size()>& results);

} // namespace vdf_vdc
