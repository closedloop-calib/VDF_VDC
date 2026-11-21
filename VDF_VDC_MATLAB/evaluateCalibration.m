function metrics = evaluateCalibration( ...
    baseInTcp, boardInCamera, boardInBase, cameraInTcp, cameraMatrix, imagePoints, worldPoints)
    % EVALUATECALIBRATION AX=ZB surrogate metrics using measured camera poses.
    % Adapted from the user-supplied computeErrors.m (RWHE-Calib derivative).
    % Returns [mean rotation angle (deg), mean translation norm (m), RMS pixels].
    % The original inverse operations are retained to preserve the evaluation of
    % rounded pose matrices; no orthogonalization or data rounding is applied.
    numFrames = size(imagePoints, 3);
    points4D = [worldPoints, zeros(size(worldPoints, 1), 1), ones(size(worldPoints, 1), 1)]';
    rotationErrors = zeros(numFrames, 1);
    translationErrors = zeros(numFrames, 3);
    squaredPixelErrors = zeros(size(imagePoints, 1), numFrames);
    for frame = 1:numFrames
        % Match the original entry script followed by computeErrors.m.
        tcpToBase = invertRigidTransform(baseInTcp(:, :, frame));
        evaluatedRobotPose = inv(tcpToBase);
        rotationResidual = inv(evaluatedRobotPose(1:3, 1:3)) * cameraInTcp(1:3, 1:3) * ...
            boardInCamera(1:3, 1:3, frame) * inv(boardInBase(1:3, 1:3));
        cosine = (trace(rotationResidual) - 1) / 2;
        cosine = max(min(cosine, 1), -1);
        rotationErrors(frame) = acos(cosine) * (180 / pi);
        poseResidual = inv(evaluatedRobotPose) * cameraInTcp * boardInCamera(:, :, frame) * inv(boardInBase);
        translationErrors(frame, :) = poseResidual(1:3, 4)';
        predictedCameraPose = inv(cameraInTcp) * evaluatedRobotPose * boardInBase;
        homogeneousPixels = cameraMatrix * (predictedCameraPose * points4D);
        normalizedPixels = bsxfun(@rdivide, homogeneousPixels, homogeneousPixels(3, :));
        projectedPixels = normalizedPixels(1:2, :)';
        squaredPixelErrors(:, frame) = sum((imagePoints(:, :, frame) - projectedPixels).^2, 2);
    end
    metrics = [mean(rotationErrors), ...
               mean(sqrt(sum(translationErrors.^2, 2))), sqrt(mean(squaredPixelErrors(:)))];
end
