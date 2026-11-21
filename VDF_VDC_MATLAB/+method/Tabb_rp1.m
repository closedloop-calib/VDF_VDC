function [boardInBase, cameraInTcp, elapsedSeconds] = Tabb_rp1(baseInTcp, initial, cameraMatrix, imagePoints, worldPoints)
    % CALIBRATETABBRP1 Tabb rp1 baseline adapted from RWHE-Calib/AXZBreproj.m.
    % Only the Levenberg-Marquardt branch used in the supplied experiment remains.
    % See THIRD_PARTY_NOTICES.md for source and citation.
    tic;
    boardPoints4D = [worldPoints, zeros(size(worldPoints, 1), 1), ones(size(worldPoints, 1), 1)]';
    options.Algorithm = 'levenberg-marquardt';
    options.ScaleProblem = 'jacobian';
    options.Display = 'off';
    objective = @(est) reprojectionResidual(imagePoints, boardPoints4D, cameraMatrix, baseInTcp, est);
    parameters = lsqnonlin(objective, initial, [], [], options);
    boardRotation = quat2rotm(parameters(1:4));
    boardTranslation = parameters(5:7);
    cameraRotation = quat2rotm(parameters(8:11));
    cameraTranslation = parameters(12:14);
    boardInBase = createHT(boardRotation, boardTranslation);
    cameraInTcp = createHT(cameraRotation, cameraTranslation);
    elapsedSeconds = toc;
end

function [err] = reprojectionResidual(imagePoints, boardPoints4D, cameraMatrix, baseInTcp, x)
    firstTwoRows = @(x) x(1:2, :);
    normalizeProjection = @(ptProj) firstTwoRows(bsxfun(@rdivide, ptProj, ptProj(3, :)));
    for viewIndex = 1:length(baseInTcp)
        robotPose = baseInTcp(:, :, viewIndex);
        observedPixels = imagePoints(:, :, viewIndex);
        cameraPoseFromParameters = @(val)   (inv([[quat2rotm(val(8:11)) val(12:14)']; [0 0 0 1]]) * robotPose *  ([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1]]));
        projectedPixels = (normalizeProjection(cameraMatrix * (cameraPoseFromParameters(x)) * boardPoints4D))';
        err(:, :, viewIndex) = observedPixels - projectedPixels;
    end
    err = reshape(permute(err, [2, 1, 3]), size(err, 2), [])';
end
