% SOLVESCHURLM Block-Schur Levenberg-Marquardt with right SE(3) updates.
function x = solveSchurLM(cF, x, maxIter, tol)
    if nargin < 3 || isempty(maxIter)
        maxIter = 50;
    end
    if nargin < 4 || isempty(tol)
        tol     = 1e-6;
    end
    lambda   = 1e-3;
    nu_up    = 7.0;
    nu_down  = 1 / 7.0;
    [pose_cost_vec, Jg_pose, Jc_pose, Hcc_pix, gc_pix, pix_cost] = cF(x);
    cost = 0.5 * (pose_cost_vec.' * pose_cost_vec) + sumCell(pix_cost);
    n  = numel(Jg_pose);
    ng = 12;
    for it = 1:maxIter
        Hgg = zeros(ng, ng);
        gg  = zeros(ng, 1);
        Hcc_diag = cell(n, 1);
        gc_cells = cell(n, 1);
        Hcg_cells = cell(n, 1);
        for i = 1:n
            Jg = Jg_pose{i};
            Jc = Jc_pose{i};
            rp = pose_cost_vec(6 * i - 5:6 * i, 1);
            Hcc_i = Jc.' * Jc;
            gc_i  = Jc.' * rp;
            Hgg   = Hgg + Jg.' * Jg;
            Hcg   = Jc.' * Jg;
            gg    = gg  + Jg.' * rp;
            if ~isempty(Hcc_pix{i})
                Hcc_i = Hcc_i + Hcc_pix{i};
                gc_i  = gc_i  + gc_pix{i};
            end
            Hcc_diag{i} = Hcc_i;
            gc_cells{i} = gc_i;
            Hcg_cells{i} = Hcg;
        end
        Dgg = diag(Hgg);
        Hgg_d = Hgg + lambda * diag(Dgg + 1e-15);
        Hcc_d_inv = cell(n, 1);
        for i = 1:n
            Dcc = diag(Hcc_diag{i});
            Hcc_d = Hcc_diag{i} + lambda * diag(Dcc + 1e-15);
            Hcc_d_inv{i} = invertPositiveDefinite(Hcc_d);
        end
        S   = Hgg_d;
        rhs = gg;
        for i = 1:n
            Hcg = Hcg_cells{i};
            Hcc_inv = Hcc_d_inv{i};
            S   = S   - Hcg.' * (Hcc_inv * Hcg);
            rhs = rhs - Hcg.' * (Hcc_inv * gc_cells{i});
        end
        rhs = -rhs;
        d_g = S \ rhs;
        d_c = zeros(6 * n, 1);
        for i = 1:n
            Hcg = Hcg_cells{i};
            Hcc_inv = Hcc_d_inv{i};
            d_c(6 * i - 5:6 * i, 1) = -Hcc_inv * (gc_cells{i} + Hcg * d_g);
        end
        x_cand = applyRightUpdate(x, d_g, d_c);
        [pose_cost_vec2, Jg2, Jc2, Hcc_pix2, gc_pix2, pix_cost2] = cF(x_cand);
        cost_new = 0.5 * (pose_cost_vec2.' * pose_cost_vec2) + sumCell(pix_cost2);
        pred = 0.5 * (d_g.' * (lambda * diag(Dgg)) * d_g - gg.' * d_g);
        for i = 1:n
            Dcc = diag(Hcc_diag{i});
            dc  = d_c(6 * i - 5:6 * i, 1);
            gc  = gc_cells{i};
            pred = pred + 0.5 * (dc.' * (lambda * diag(Dcc)) * dc - gc.' * dc);
        end
        act = cost - cost_new;
        rho = (pred > 0) * (act / max(pred, eps));
        if rho > 0
            x = x_cand;
            cost = cost_new;
            Jg_pose = Jg2;
            Jc_pose = Jc2;
            Hcc_pix = Hcc_pix2;
            gc_pix = gc_pix2;
            pose_cost_vec = pose_cost_vec2;
                lambda = max(lambda * max(nu_down, 1 - (2 * rho - 1)^3), 1e-15);
            if abs(act) < tol
                break
            end
        else
            lambda = min(lambda * nu_up, 1e12);
            if lambda > 1e11
                break
            end
        end
    end
end

function s = sumCell(C)
    s = 0;
    for i = 1:numel(C)
        if ~isempty(C{i})
            s = s + C{i};
        end
    end
end

function x_new = applyRightUpdate(x, d_g, d_c)
    n  = (numel(x) - 12) / 6;
    deltaCameraInTcp = d_g(1:6);
    deltaBoardInBase = d_g(7:12);
    cameraInTcp = se3Exp(x(1:6));
    boardInBase = se3Exp(x(7:12));
    cameraInTcp = cameraInTcp * se3Exp(deltaCameraInTcp);
    boardInBase = boardInBase * se3Exp(deltaBoardInBase);
    x_new = [se3Log(cameraInTcp).', se3Log(boardInBase).'];
    for i = 1:n
        bi = x(12 + (i - 1) * 6 + (1:6)).';
        Bi = se3Exp(bi);
        db = d_c(6 * i - 5:6 * i);
        Bi = Bi * se3Exp(db);
        x_new = [x_new, se3Log(Bi).'];
    end
    x_new = x_new(:).';
end

function Ainv = invertPositiveDefinite(A)
    jit = 1e-12;
    for k = 1:5
        [L, p] = chol(A + jit * eye(size(A)));
        if p == 0
            Ainv = (L \ (L' \ eye(size(A))));
            return
        end
        jit = jit * 10;
    end
    Ainv = inv(A);
end
