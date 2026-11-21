function [boardInBase, cameraInTcp, elapsedSeconds, consistencyRatio] = ...
    VDF_VDC(baseInTcp, boardInCamera, initialCameraInTcp, initialBoardInBase, ...
              cameraMatrix, imagePoints, worldPoints)
    % CALIBRATE VDF-VDC calibration for the supplied real-world experiment.
    % Inputs: baseInTcp A_i and measured boardInCamera B_i are 4-by-4-by-N.
    % Model: A_i * boardInBase = cameraInTcp * B_i.
    % cameraMatrix is [K, zeros(3,1)]; imagePoints are undistorted pixels.
    % worldPoints are planar XY coordinates in metres. Translations are metres.
    % Internal state: [log(cameraInTcp), log(boardInBase), log(D_1), ..., log(D_N)].
    % The numerical settings are retained from the supplied VDA_WDC.m.

    tic;
    n = size(baseInTcp, 3);
    Np = size(imagePoints, 1);
    Wpts = [worldPoints, zeros(size(worldPoints, 1), 1), ones(size(worldPoints, 1), 1)]';
    x = [se3Log(initialCameraInTcp)', se3Log(initialBoardInBase)'];
    for t = 1:n
        b = se3Log(boardInCamera(:, :, t));
        x = [x, b']; %#ok<AGROW>
    end

    % Initial fallback scales: rotation (rad), translation (m), image (px).
    sig = [0.1, 1e-3, 0.5];
    [rotationResiduals, translationResiduals, pixelResiduals] = collectResidualBlocks(baseInTcp, x, cameraMatrix, imagePoints, Wpts);
    estimatedScales = estimateResidualScales(rotationResiduals, translationResiduals, pixelResiduals);
    if all(isfinite(estimatedScales))
        sig = estimatedScales;
    end

    outerLoops = 3;
    balancing.pixelScale = sqrt(3 / Np);
    for outer = 1:outerLoops
        costFunction = @(est) buildNormalEquations(baseInTcp, est, cameraMatrix, imagePoints, Wpts, sig, balancing);
        x = solveSchurLM(costFunction, x, 10, 1e-3);
        if outer == 1
            numPoints = size(worldPoints, 1);
            imageError = zeros(numPoints, n);
            closureError = zeros(numPoints, n);
            for frame = 1:n
                measuredPixels = imagePoints(:, :, frame);
                impliedPose = se3Exp(x(12 + (frame - 1) * 6 + (1:6)));
                imageProjection = projectPose(impliedPose, Wpts, cameraMatrix);
                imageError(:, frame) = sqrt(sum((measuredPixels - imageProjection).^2, 2));
                closurePose = inv(se3Exp(x(1:6))) * baseInTcp(:, :, frame) * se3Exp(x(7:12));
                closureProjection = projectPose(closurePose, Wpts, cameraMatrix);
                closureError(:, frame) = sqrt(sum((measuredPixels - closureProjection).^2, 2));
            end
            consistencyRatio = mean(imageError(:)) / mean(closureError(:));
            omega = 1;
            if consistencyRatio < 0.2
                omega = 5;
            elseif consistencyRatio <= 0.35
                omega = 1 / consistencyRatio;
            end
            balancing.pixelScale = balancing.pixelScale * omega;
        end
        % Standard-deviation estimates, with the original non-increasing update.
        [rotationResiduals, translationResiduals, pixelResiduals] = collectResidualBlocks(baseInTcp, x, cameraMatrix, imagePoints, Wpts);
        updatedScales = estimateResidualScales(rotationResiduals, translationResiduals, pixelResiduals);
        sig = min(updatedScales, sig);
    end
    cameraInTcp = se3Exp(x(1:6));
    boardInBase = se3Exp(x(7:12));
    elapsedSeconds = toc;
end

function proj = projectPose(T, Wpts, cameraMatrix)
    selectRow12 = @(x) x(1:2, :);
    normbyZ = @(ptProj) selectRow12(bsxfun(@rdivide, ptProj, ptProj(3, :)));
    proj = normbyZ(cameraMatrix * (T * Wpts))';
end

function [rotationResiduals, translationResiduals, pixelResiduals] = collectResidualBlocks(baseInTcp, x, cameraMatrix, imagePoints, Wpts)
    n  = size(baseInTcp, 3);
    Np = size(imagePoints, 1);
    cameraInTcp = se3Exp(x(1:6));
    boardInBase = se3Exp(x(7:12));
    rotationResiduals = zeros(n, 3);
    translationResiduals = zeros(n, 3);
    pixelResiduals = zeros(n * Np, 2);
    row = 1;
    for i = 1:n
        A = baseInTcp(:, :, i);
        B = se3Exp(x(i * 6 + 7:i * 6 + 12));
        T_pred = cameraInTcp * B / boardInBase;
        T_err  = inv(A) * T_pred;
        r_pose = se3Log(T_err);
        rotationResiduals(i, :) = r_pose(1:3);
        translationResiduals(i, :) = r_pose(4:6);
        imgP = imagePoints(:, :, i);
        HT   = B;
        proj = projectWorldPoints(cameraMatrix, HT, Wpts);
        r_pix = imgP - proj;
        pixelResiduals(row:row + Np - 1, :) = r_pix;
        row = row + Np;
    end
end

function estimatedScales = estimateResidualScales(rotationResiduals, translationResiduals, pixelResiduals)
    sig_r  = std(rotationResiduals(:));
    sig_t  = std(translationResiduals(:));
    sig_px = std(pixelResiduals(:));
    estimatedScales = [sig_r, sig_t, sig_px];
end

function proj = projectWorldPoints(cameraMatrix, HT, Wpts4)
    Pc = cameraMatrix * (HT * Wpts4);
    Pc = Pc ./ Pc(3, :);
    proj = Pc(1:2, :)';
end

function [pose_cost_vec, Jg_pose, Jc_pose, ...
          Hcc_pix, gc_pix, pix_cost] = ...
    buildNormalEquations(baseInTcp, x, cameraMatrix, imagePoints, Wpts, sig, balancing)
    n  = size(baseInTcp, 3);
    cameraInTcp = se3Exp(x(1:6));
    boardInBase = se3Exp(x(7:12));
    sigma_r  = sig(1);
    sigma_t  = sig(2);
    sigma_px = sig(3);
    Wr = diag([ones(1, 3) / sigma_r, ones(1, 3) / sigma_t]);
    pix_scale = balancing.pixelScale;
    s_pix = (1 / sigma_px) * pix_scale;
    Jg_pose   = cell(n, 1);
    Jc_pose   = cell(n, 1);
    Hcc_pix   = cell(n, 1);
    gc_pix    = cell(n, 1);
    pix_cost  = cell(n, 1);
    pose_cost_vec = zeros(6 * n, 1);
    AdX = adjointSE3(boardInBase);
    for i = 1:n
        Ai = baseInTcp(:, :, i);
        bi = x(12 + (i - 1) * 6 + (1:6));
        Bi = se3Exp(bi);
        E  = invertSE3(Ai) * cameraInTcp * Bi * invertSE3(boardInBase);
        ei = se3Log(E);
        Jr_inv = rightJacobianInverseSE3(ei);
        AdB_inv = adjointSE3(invertSE3(Bi));
        Jx = Jr_inv * (-AdX);
        Jz = Jr_inv * (AdX * AdB_inv);
        Jb = Jr_inv * (AdX);
        ei_w = Wr * ei;
        Jx_w = Wr * Jx;
        Jz_w = Wr * Jz;
        Jb_w = Wr * Jb;
        Jg_pose{i} = [Jz_w, Jx_w];
        Jc_pose{i} = Jb_w;
        pose_cost_vec(6 * i - 5:6 * i, 1) = ei_w;
        imgP = imagePoints(:, :, i);
        [JtJ_raw, Jtr_raw, pix_cost_raw] = pixelNormalEquations(Bi, imgP, Wpts, cameraMatrix);
        Hcc_pix{i}  = (s_pix^2) * JtJ_raw;
        gc_pix{i}   = (s_pix^2)   * Jtr_raw;
        pix_cost{i} = (s_pix^2) * pix_cost_raw;
    end
end

function Tinv = invertSE3(T)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    Tinv = eye(4);
    Tinv(1:3, 1:3) = R.';
    Tinv(1:3, 4) = -R.' * t;
end

function Ad = adjointSE3(T)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    Ad = [R,               zeros(3, 3)
          skewMatrix(t) * R,       R];
end

function Jr_inv = rightJacobianInverseSE3(xi)
    w = xi(1:3);
    v = xi(4:6);
    W = skewMatrix(w);
    V = skewMatrix(v);
    ad = [W, zeros(3, 3); V, W];
    Jr_inv = eye(6) + 0.5 * ad + (1 / 12) * (ad * ad);
end

function [JtJ, Jtr, cost] = pixelNormalEquations(T, imgP, Wpts, cameraMatrix)
    M = size(imgP, 1);
    if M == 0
        JtJ = zeros(6, 6);
        Jtr = zeros(6, 1);
        cost = 0;
        return
    end
    R  = T(1:3, 1:3);
    t  = T(1:3, 4);
    points3D  = Wpts(1:3, :);
    P  = R * points3D + t;
    x  = P(1, :).';
    y = P(2, :).';
    z = P(3, :).';
    fx = cameraMatrix(1, 1);
    fy = cameraMatrix(2, 2);
    cx = cameraMatrix(1, 3);
    cy = cameraMatrix(2, 3);
    u  = fx .* (x ./ z) + cx;
    v  = fy .* (y ./ z) + cy;
    resx = u - imgP(:, 1);
    resy = v - imgP(:, 2);
    a = fx ./ z;
    b = -fx .* x ./ (z.^2);
    c =  fy ./ z;
    d = -fy .* y ./ (z.^2);
    RX = R * points3D;
    uRX = RX(1, :).';
    vRX = RX(2, :).';
    wRX = RX(3, :).';
    G1 = [b .* vRX,              a .* wRX - b .* uRX,   -a .* vRX];
    G2 = [-c .* wRX + d .* vRX,    -d .* uRX,            c .* uRX];
    H1 = bsxfun(@times, a, R(1, :)) + bsxfun(@times, b, R(3, :));
    H2 = bsxfun(@times, c, R(2, :)) + bsxfun(@times, d, R(3, :));
    GtG = G1.' * G1 + G2.' * G2;
    GtH = G1.' * H1 + G2.' * H2;
    HtH = H1.' * H1 + H2.' * H2;
    Gtr = G1.' * resx + G2.' * resy;
    Htr = H1.' * resx + H2.' * resy;
    JtJ = [GtG, GtH; GtH.', HtH];
    Jtr = [Gtr; Htr];
    cost = 0.5 * (resx.' * resx + resy.' * resy);
end
