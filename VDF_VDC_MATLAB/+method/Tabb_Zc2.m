function [boardInBase, cameraInTcp, elapsedSeconds] = Tabb_Zc2(baseInTcp, boardInCamera, initial)
    % CALIBRATETABBZC2 Tabb Zc2 baseline adapted from RWHE-Calib/AXZB.m.
    % Only cost 2 and the Levenberg-Marquardt branch are retained.
    % See THIRD_PARTY_NOTICES.md for source and citation.
    tic;
    options.Algorithm = 'levenberg-marquardt';
    options.ScaleProblem = 'jacobian';
    options.Display = 'off';
    objective = @(est) poseConsistencyResidual(baseInTcp, boardInCamera, est);
    parameters = lsqnonlin(objective, initial, [], [], options);
    boardRotation = quat2rotm(parameters(1:4));
    boardTranslation = parameters(5:7);
    cameraRotation = quat2rotm(parameters(8:11));
    cameraTranslation = parameters(12:14);
    boardInBase = createHT(boardRotation, boardTranslation);
    cameraInTcp = createHT(cameraRotation, cameraTranslation);
    elapsedSeconds = toc;
end

function [err] = poseConsistencyResidual(baseInTcp, boardInCamera, x)
    for viewIndex = 1:length(baseInTcp)
        robotPose = baseInTcp(:, :, viewIndex);
        observedCameraPose = boardInCamera(:, :, viewIndex);
        residualNorm = @(val) sum([quatnorm(val(1:4)) norm(val(5:7))]);
        rotationTranslationVector = @(val) [rotm2quat(val(1:3, 1:3)) val(1:3, 4)'];
        poseDifference = [robotPose - ([[quat2rotm([x(8) x(9) x(10) x(11)]) x(12:14)']; [0 0 0 1]] * observedCameraPose * inv([[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1]]))];
        err(viewIndex, :) = residualNorm(rotationTranslationVector(poseDifference));
    end

end
