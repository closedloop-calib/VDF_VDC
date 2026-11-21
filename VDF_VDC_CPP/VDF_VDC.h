#pragma once

#include "CalibrationData.h"

namespace vdf_vdc {

enum class SolverMode { DenseFull, SparseFull, DenseSchur, SparseSchur };

struct OptimizationSettings {
    int outerLoops = 3;
    int lmMaxIterations = 10;
    double lmTolerance = 1e-3;
};

struct SolverStats {
    int lmIterations = 0;
    int acceptedSteps = 0;
    double finalCost = 0.0;
    double consistencyRatio = 0.0;
};

// Pass the Li result as initial. SparseSchur uses the per-view block structure.
Calibration VDF_VDC(const CalibrationData& data, const Calibration& initial,
                    SolverMode mode = SolverMode::SparseSchur, SolverStats* stats = nullptr,
                    const OptimizationSettings& settings = {});

} // namespace vdf_vdc
