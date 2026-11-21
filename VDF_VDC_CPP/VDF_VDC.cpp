#include "VDF_VDC.h"
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace vdf_vdc {
namespace {
struct VDFState {
    Eigen::Matrix4d cameraInTcp;
    Eigen::Matrix4d boardInBase;
    std::vector<Eigen::Matrix4d> impliedPoses;
};

struct CostBlocks {
    Eigen::VectorXd pose_cost_vec;  // 6n x 1
    std::vector<Eigen::Matrix<double, 6, 12>> Jg_pose;
    std::vector<Matrix6>  Jc_pose;
    std::vector<Matrix6>  Hcc_pix;
    std::vector<Vector6>  gc_pix;
    std::vector<double> pix_cost;
};

double sumCosts(const std::vector<double>& v) {
    return std::accumulate(v.begin(), v.end(), 0.0);
}

Eigen::Vector3d estimateResidualScales(
    const Eigen::MatrixXd& rotationResiduals,
    const Eigen::MatrixXd& translationResiduals,
    const Eigen::MatrixXd& pixelResiduals) {
    auto compute_std = [](const Eigen::MatrixXd& M) -> double {
            if (M.size() == 0) return 1e-12;
            const double mean = M.array().mean();
            const double var =
                (M.array() - mean).square().sum() /
                std::max(1.0, static_cast<double>(M.size() - 1));
            return std::sqrt(var);
        };

    const double sig_r = compute_std(rotationResiduals);
    const double sig_t = compute_std(translationResiduals);
    const double sig_px = compute_std(pixelResiduals);

    return Eigen::Vector3d(sig_r, sig_t, sig_px);
}

void collectResidualBlocks(
    const std::vector<Eigen::Matrix4d>& baseInTcp,
    const VDFState& state,
    const CameraMatrix& cameraMatrix,
    const std::vector<Eigen::MatrixXd>& imagePoints,
    const Eigen::MatrixXd& homogeneousWorldPoints,
    Eigen::MatrixXd& rotationResiduals,
    Eigen::MatrixXd& translationResiduals,
    Eigen::MatrixXd& pixelResiduals) {
    const int n = static_cast<int>(baseInTcp.size());
    if (n == 0) throw std::runtime_error("baseInTcp is empty.");
    const int Np = static_cast<int>(imagePoints[0].rows());

    rotationResiduals.resize(n, 3);
    translationResiduals.resize(n, 3);
    pixelResiduals.resize(n * Np, 2);

    int row = 0;
    for (int i = 0; i < n; ++i) {
        const Eigen::Matrix4d& Ai = baseInTcp[i];
        const Eigen::Matrix4d& Bi = state.impliedPoses[i];

        const Eigen::Matrix4d T_pred = state.cameraInTcp * Bi * state.boardInBase.inverse();
        const Eigen::Matrix4d T_err = Ai.inverse() * T_pred;
        const Vector6 r_pose = se3Log(T_err);

        rotationResiduals.row(i) = r_pose.head<3>().transpose();
        translationResiduals.row(i) = r_pose.tail<3>().transpose();

        const Eigen::MatrixXd proj = projectPose(Bi, homogeneousWorldPoints, cameraMatrix);
        const Eigen::MatrixXd r_pix = imagePoints[i] - proj;

        pixelResiduals.block(row, 0, Np, 2) = r_pix;
        row += Np;
    }
}

double updatePixelScale(
    const std::vector<Eigen::Matrix4d>& baseInTcp,
    const VDFState& state,
    const CameraMatrix& cameraMatrix,
    const std::vector<Eigen::MatrixXd>& imagePoints,
    const Eigen::MatrixXd& homogeneousWorldPoints,
    double& pixelScale) {
    const int n = static_cast<int>(baseInTcp.size());
    if (n == 0) throw std::runtime_error("baseInTcp is empty.");
    const int M = static_cast<int>(imagePoints[0].rows());

    Eigen::MatrixXd err1(M, n);
    Eigen::MatrixXd err2(M, n);

    for (int tt = 0; tt < n; ++tt) {
        const Eigen::MatrixXd& imgP = imagePoints[tt];

        const Eigen::Matrix4d& Bi1 = state.impliedPoses[tt];
        const Eigen::MatrixXd proj1 = projectPose(Bi1, homogeneousWorldPoints, cameraMatrix);
        err1.col(tt) = (imgP - proj1).rowwise().norm();

        const Eigen::Matrix4d Bi2 = state.cameraInTcp.inverse() * baseInTcp[tt] * state.boardInBase;
        const Eigen::MatrixXd proj2 = projectPose(Bi2, homogeneousWorldPoints, cameraMatrix);
        err2.col(tt) = (imgP - proj2).rowwise().norm();
    }

    const double mean1 = err1.mean();
    const double mean2 = err2.mean();
    const double consistencyRatio = mean1 / mean2;

    double omega = 1.0;
    if (consistencyRatio < 0.2) {
        omega = 5.0;
    } else if (consistencyRatio <= 0.35) {
        omega = 1.0 / consistencyRatio;
    }

    pixelScale *= omega;
    return consistencyRatio;
}

void applyRightUpdate(
    const VDFState& state,
    const Eigen::Matrix<double, 12, 1>& d_g,
    const Eigen::VectorXd& d_c,
    VDFState& state_new) {
    const int n = static_cast<int>(state.impliedPoses.size());

    state_new = state;

    const Vector6 deltaCameraInTcp = d_g.segment<6>(0);
    const Vector6 deltaBoardInBase = d_g.segment<6>(6);

    state_new.cameraInTcp = se3Exp(se3Log(state.cameraInTcp * se3Exp(deltaCameraInTcp)));
    state_new.boardInBase = se3Exp(se3Log(state.boardInBase * se3Exp(deltaBoardInBase)));

    for (int i = 0; i < n; ++i) {
        const Vector6 deltaImpliedPose = d_c.segment<6>(6 * i);
        state_new.impliedPoses[i] = se3Exp(se3Log(state.impliedPoses[i] * se3Exp(deltaImpliedPose)));
    }
}

// Build the weighted pose and pixel blocks shared by all modes.
void buildNormalEquations(
    const std::vector<Eigen::Matrix4d>& baseInTcp,
    const VDFState& state,
    const CameraMatrix& cameraMatrix,
    const std::vector<Eigen::MatrixXd>& imagePoints,
    const Eigen::MatrixXd& homogeneousWorldPoints,
    const Eigen::Vector3d& sig,
    double pixelScale,
    CostBlocks& blocks) {
    const int n = static_cast<int>(baseInTcp.size());

    const double sigma_r = (sig(0) == 0.0 ? 1e-12 : sig(0));
    const double sigma_t = (sig(1) == 0.0 ? 1e-12 : sig(1));
    const double sigma_px = (sig(2) == 0.0 ? 1e-12 : sig(2));

    Matrix6 Wr = Matrix6::Zero();
    Wr.diagonal() << 1.0 / sigma_r, 1.0 / sigma_r, 1.0 / sigma_r,
        1.0 / sigma_t, 1.0 / sigma_t, 1.0 / sigma_t;

    const double s_pix = (1.0 / sigma_px) * pixelScale;

    blocks.pose_cost_vec.resize(6 * n);
    blocks.Jg_pose.resize(n);
    blocks.Jc_pose.resize(n);
    blocks.Hcc_pix.resize(n);
    blocks.gc_pix.resize(n);
    blocks.pix_cost.resize(n);

    const Matrix6 AdX = adjointSE3(state.boardInBase);

    for (int i = 0; i < n; ++i) {
        const Eigen::Matrix4d& Ai = baseInTcp[i];
        const Eigen::Matrix4d& Bi = state.impliedPoses[i];

        // ---------------- pose block ----------------
        const Eigen::Matrix4d E = invertSE3(Ai) * state.cameraInTcp * Bi * invertSE3(state.boardInBase);
        const Vector6 ei = se3Log(E);
        const Matrix6 Jr_inv = rightJacobianInverseSE3(ei);
        const Matrix6 AdB_inv = adjointSE3(invertSE3(Bi));

        const Matrix6 Jx = Jr_inv * (-AdX);
        const Matrix6 Jz = Jr_inv * (AdX * AdB_inv);
        const Matrix6 Jb = Jr_inv * (AdX);

        const Vector6 ei_w = Wr * ei;
        const Matrix6 Jx_w = Wr * Jx;
        const Matrix6 Jz_w = Wr * Jz;
        const Matrix6 Jb_w = Wr * Jb;

        Eigen::Matrix<double, 6, 12> Jg;
        Jg.leftCols<6>() = Jz_w;
        Jg.rightCols<6>() = Jx_w;

        blocks.Jg_pose[i] = Jg;
        blocks.Jc_pose[i] = Jb_w;
        blocks.pose_cost_vec.segment<6>(6 * i) = ei_w;

        // ---------------- pixel block ----------------
        Matrix6 JtJ_raw;
        Vector6 Jtr_raw;
        double pix_cost_raw = 0.0;

        pixelNormalEquations(Bi, imagePoints[i], homogeneousWorldPoints, cameraMatrix,
            JtJ_raw, Jtr_raw, pix_cost_raw);

        blocks.Hcc_pix[i] = (s_pix * s_pix) * JtJ_raw;
        blocks.gc_pix[i] = (s_pix * s_pix) * Jtr_raw;
        blocks.pix_cost[i] = (s_pix * s_pix) * pix_cost_raw;
    }
}

double costFromBlocks(const CostBlocks& blocks) {
    return 0.5 * blocks.pose_cost_vec.squaredNorm() + sumCosts(blocks.pix_cost);
}

void solveSchurLM(
    const std::vector<Eigen::Matrix4d>& baseInTcp,
    VDFState& state,
    const CameraMatrix& cameraMatrix,
    const std::vector<Eigen::MatrixXd>& imagePoints,
    const Eigen::MatrixXd& homogeneousWorldPoints,
    const Eigen::Vector3d& sig,
    double pixelScale,
    int maxIter,
    double tol,
    SolverMode mode,
    SolverStats* stats) {
    const int n = static_cast<int>(baseInTcp.size());
    const int ng = 12;

    double lambda = 1e-3;
    const double nu_up = 7.0;
    const double nu_down = 1.0 / 7.0;

    CostBlocks curBlocks;
    buildNormalEquations(baseInTcp, state, cameraMatrix, imagePoints, homogeneousWorldPoints, sig, pixelScale, curBlocks);
    double cost = costFromBlocks(curBlocks);

    for (int it = 0; it < maxIter; ++it) {
        Eigen::Matrix<double, ng, ng> Hgg = Eigen::Matrix<double, ng, ng>::Zero();
        Eigen::Matrix<double, ng, 1>  gg = Eigen::Matrix<double, ng, 1>::Zero();

        std::vector<Matrix6> Hcc_diag(n);
        std::vector<Vector6> gc_cells(n);
        std::vector<Eigen::Matrix<double, 6, ng>> Hcg_cells(n);

        for (int i = 0; i < n; ++i) {
            const Eigen::Matrix<double, 6, ng>& Jg = curBlocks.Jg_pose[i];
            const Matrix6& Jc = curBlocks.Jc_pose[i];
            const Vector6 rp = curBlocks.pose_cost_vec.segment<6>(6 * i);

            Matrix6 Hcc_i = Jc.transpose() * Jc;
            Vector6 gc_i = Jc.transpose() * rp;
            Eigen::Matrix<double, 6, ng> Hcg = Jc.transpose() * Jg;

            Hgg += Jg.transpose() * Jg;
            gg += Jg.transpose() * rp;

            Hcc_i += curBlocks.Hcc_pix[i];
            gc_i += curBlocks.gc_pix[i];

            Hcc_diag[i] = Hcc_i;
            gc_cells[i] = gc_i;
            Hcg_cells[i] = Hcg;
        }

        // ---------------- LM damping ----------------
        Eigen::Matrix<double, ng, 1> Dgg = Hgg.diagonal();
        Eigen::Matrix<double, ng, ng> Dgg_mat = Eigen::Matrix<double, ng, ng>::Zero();
        Dgg_mat.diagonal() = Dgg.array() + 1e-15;

        Eigen::Matrix<double, ng, ng> Hgg_d = Hgg + lambda * Dgg_mat;

        std::vector<Matrix6> Hcc_inv;
        if (mode == SolverMode::SparseSchur) Hcc_inv.resize(n);
        std::vector<Matrix6> Hcc_damped(n);
        for (int i = 0; i < n; ++i) {
            Vector6 Dcc = Hcc_diag[i].diagonal();
            Matrix6 Dcc_mat = Matrix6::Zero();
            Dcc_mat.diagonal() = Dcc.array() + 1e-15;

            Hcc_damped[i] = Hcc_diag[i] + lambda * Dcc_mat;
            // Use the MATLAB local Cholesky regularization in every mode.
            Hcc_damped[i].diagonal().array() += 1e-12;
            if (mode == SolverMode::SparseSchur)
                Hcc_inv[i] = Hcc_damped[i].llt().solve(Matrix6::Identity());
        }

        Eigen::Matrix<double, ng, 1> d_g;
        Eigen::VectorXd d_c(6 * n);
        if (mode == SolverMode::SparseSchur) {
            // Assemble the reduced system directly from per-view blocks.
            Eigen::Matrix<double, ng, ng> S = Hgg_d;
            Eigen::Matrix<double, ng, 1> rhs = gg;
            for (int i = 0; i < n; ++i) {
                S.noalias() -= Hcg_cells[i].transpose() * Hcc_inv[i] * Hcg_cells[i];
                rhs.noalias() -= Hcg_cells[i].transpose() * Hcc_inv[i] * gc_cells[i];
            }
            d_g = S.ldlt().solve(-rhs);
            for (int i = 0; i < n; ++i)
                d_c.segment<6>(6 * i) =
                    -Hcc_inv[i] * (gc_cells[i] + Hcg_cells[i] * d_g);
        } else {
            const int dim = ng + 6 * n;
            Eigen::VectorXd g = Eigen::VectorXd::Zero(dim);
            g.head<ng>() = gg;
            for (int i = 0; i < n; ++i)
                g.segment<6>(ng + 6 * i) = gc_cells[i];

            if (mode == SolverMode::SparseFull) {
                typedef Eigen::Triplet<double> Triplet;
                std::vector<Triplet> entries;
                entries.reserve(ng * ng + n * (36 + 2 * 6 * ng));
                for (int r = 0; r < ng; ++r)
                    for (int c = 0; c < ng; ++c)
                        entries.emplace_back(r, c, Hgg_d(r, c));
                for (int i = 0; i < n; ++i) {
                    const int off = ng + 6 * i;
                    for (int r = 0; r < 6; ++r) {
                        for (int c = 0; c < 6; ++c)
                            entries.emplace_back(off + r, off + c, Hcc_damped[i](r, c));
                        for (int c = 0; c < ng; ++c) {
                            entries.emplace_back(off + r, c, Hcg_cells[i](r, c));
                            entries.emplace_back(c, off + r, Hcg_cells[i](r, c));
                        }
                    }
                }
                Eigen::SparseMatrix<double> H(dim, dim);
                H.setFromTriplets(entries.begin(), entries.end());
                H.makeCompressed();
                Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor;
                factor.compute(H);
                if (factor.info() != Eigen::Success)
                    throw std::runtime_error("SparseFull factorization failed");
                const Eigen::VectorXd d = factor.solve(-g);
                if (factor.info() != Eigen::Success || !d.allFinite())
                    throw std::runtime_error("SparseFull solve failed");
                d_g = d.head<ng>();
                d_c = d.tail(6 * n);
            }
            else {
                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
                H.topLeftCorner<ng, ng>() = Hgg_d;
                for (int i = 0; i < n; ++i) {
                    const int off = ng + 6 * i;
                    H.block<6, 6>(off, off) = Hcc_damped[i];
                    H.block(off, 0, 6, ng) = Hcg_cells[i];
                    H.block(0, off, ng, 6) = Hcg_cells[i].transpose();
                }
                if (mode == SolverMode::DenseFull) {
                    Eigen::LDLT<Eigen::MatrixXd> factor(H);
                    if (factor.info() != Eigen::Success)
                        throw std::runtime_error("DenseFull factorization failed");
                    const Eigen::VectorXd d = factor.solve(-g);
                    if (!d.allFinite()) throw std::runtime_error("DenseFull solve failed");
                    d_g = d.head<ng>();
                    d_c = d.tail(6 * n);
                }
                else {
                    // DenseSchur: factor the complete dense local partition.
                    // This intentionally does not exploit its block-diagonal storage.
                    const Eigen::MatrixXd Hll = H.bottomRightCorner(6 * n, 6 * n);
                    const Eigen::MatrixXd Hlg = H.bottomLeftCorner(6 * n, ng);
                    Eigen::LDLT<Eigen::MatrixXd> local(Hll);
                    if (local.info() != Eigen::Success)
                        throw std::runtime_error("DenseSchur local factorization failed");
                    const Eigen::MatrixXd Lg = local.solve(Hlg);
                    const Eigen::VectorXd Lb = local.solve(g.tail(6 * n));
                    const Eigen::Matrix<double, ng, ng> S =
                        Hgg_d - Hlg.transpose() * Lg;
                    const Eigen::Matrix<double, ng, 1> rhs =
                        gg - Hlg.transpose() * Lb;
                    d_g = S.ldlt().solve(-rhs);
                    d_c = -local.solve(g.tail(6 * n) + Hlg * d_g);
                    if (!d_c.allFinite())
                        throw std::runtime_error("DenseSchur back-substitution failed");
                }
            }
        }
        if (!d_g.allFinite() || !d_c.allFinite())
            throw std::runtime_error("Nonfinite LM step");

        VDFState candState;
        applyRightUpdate(state, d_g, d_c, candState);

        CostBlocks candBlocks;
        buildNormalEquations(baseInTcp, candState, cameraMatrix, imagePoints, homogeneousWorldPoints, sig, pixelScale, candBlocks);
        const double cost_new = costFromBlocks(candBlocks);

        // ---------------- predicted reduction ----------------
        double pred = 0.0;

        pred += 0.5 * (
            d_g.transpose() * (lambda * Dgg.asDiagonal()) * d_g
            - gg.transpose() * d_g
            )(0, 0);

        for (int i = 0; i < n; ++i) {
            Vector6 Dcc = Hcc_diag[i].diagonal();
            Vector6 dc = d_c.segment<6>(6 * i);

            pred += 0.5 * (
                dc.transpose() * (lambda * Dcc.asDiagonal()) * dc
                - gc_cells[i].transpose() * dc
                )(0, 0);
        }

        const double act = cost - cost_new;
        const double rho = (pred > 0.0) ? (act / std::max(pred, std::numeric_limits<double>::epsilon())) : -1.0;

        if (stats) ++stats->lmIterations;
        if (rho > 0.0) {
            if (stats) ++stats->acceptedSteps;
            state = candState;
            curBlocks = std::move(candBlocks);
            cost = cost_new;

            lambda = std::max(
                lambda * std::max(nu_down, 1.0 - std::pow(2.0 * rho - 1.0, 3.0)),
                1e-15
            );

            if (std::abs(act) < tol) {
                break;
            }
        } else {
            lambda = std::min(lambda * nu_up, 1e12);
            if (lambda > 1e11) {
                break;
            }
        }
    }
}
} // namespace

Calibration VDF_VDC(const CalibrationData& data, const Calibration& initial,
                    SolverMode mode, SolverStats* stats,
                    const OptimizationSettings& settings) {
    validateCalibrationData(data);
    if (settings.outerLoops < 1 || settings.lmMaxIterations < 1 ||
        !std::isfinite(settings.lmTolerance) || settings.lmTolerance <= 0.0)
        throw std::invalid_argument("Invalid optimization settings.");
    if (!initial.boardInBase.allFinite() || !initial.cameraInTcp.allFinite())
        throw std::invalid_argument("Initial transforms must be finite.");
    if (stats) *stats = {};

    const auto& baseInTcp = data.baseInTcp;
    const auto& imagePoints = data.imagePoints;
    const auto& cameraMatrix = data.cameraMatrix;
    const auto Np = data.worldPoints.rows();
    Eigen::MatrixXd homogeneousWorldPoints = Eigen::MatrixXd::Zero(4, Np);
    homogeneousWorldPoints.topRows(2) = data.worldPoints.transpose();
    homogeneousWorldPoints.row(3).setOnes();

    VDFState state{se3Exp(se3Log(initial.cameraInTcp)), se3Exp(se3Log(initial.boardInBase)), {}};
    state.impliedPoses.reserve(data.boardInCamera.size());
    for (const auto& pose : data.boardInCamera)
        state.impliedPoses.push_back(se3Exp(se3Log(pose)));

    Eigen::MatrixXd rotationResiduals, translationResiduals, pixelResiduals;
    collectResidualBlocks(baseInTcp, state, cameraMatrix, imagePoints, homogeneousWorldPoints,
                          rotationResiduals, translationResiduals, pixelResiduals);
    Eigen::Vector3d sig(0.1, 1e-3, 0.5);
    const auto initialScales = estimateResidualScales(rotationResiduals, translationResiduals, pixelResiduals);
    if (initialScales.allFinite()) sig = initialScales;

    double pixelScale = std::sqrt(3.0 / static_cast<double>(Np));
    double consistencyRatio = std::numeric_limits<double>::quiet_NaN();
    for (int outer = 0; outer < settings.outerLoops; ++outer) {
        solveSchurLM(baseInTcp, state, cameraMatrix, imagePoints, homogeneousWorldPoints,
                     sig, pixelScale, settings.lmMaxIterations, settings.lmTolerance, mode, stats);
        if (outer == 0)
            consistencyRatio = updatePixelScale(baseInTcp, state, cameraMatrix, imagePoints,
                                                homogeneousWorldPoints, pixelScale);
        collectResidualBlocks(baseInTcp, state, cameraMatrix, imagePoints, homogeneousWorldPoints,
                              rotationResiduals, translationResiduals, pixelResiduals);
        const auto updatedScales = estimateResidualScales(rotationResiduals, translationResiduals, pixelResiduals);
        if (updatedScales.allFinite()) sig = sig.cwiseMin(updatedScales);
    }

    if (stats) {
        CostBlocks finalBlocks;
        buildNormalEquations(baseInTcp, state, cameraMatrix, imagePoints, homogeneousWorldPoints,
                             sig, pixelScale, finalBlocks);
        stats->finalCost = costFromBlocks(finalBlocks);
        stats->consistencyRatio = consistencyRatio;
    }
    return {state.boardInBase, state.cameraInTcp};
}

} // namespace vdf_vdc
