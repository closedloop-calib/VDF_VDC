% RUN_REAL_EXPERIMENT Run the four MATLAB methods on all 80 real-world views.
% Open this file and click Run. Paths are resolved relative to this script.
clc;
clear;
close all;
projectRoot = fileparts(mfilename('fullpath'));
originalPath = path;
pathCleanup = onCleanup(@() path(originalPath));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'auxiliaries'));
requiredFunctions = {'lsqnonlin', 'optimoptions', 'rotm2quat', 'quat2rotm', 'quatnorm'};
for functionIndex = 1:numel(requiredFunctions)
    assert(~isempty(which(requiredFunctions{functionIndex})), ...
           'Missing dependency: %s. See README.md.', requiredFunctions{functionIndex});
end
inputFile = fullfile(fileparts(projectRoot), 'hand_eye_calibration_input.txt');
data = readCalibrationData(inputFile);
numFrames = size(data.baseInTcp, 3);
assert(numFrames >= 3, 'At least three matching frames are required.');
assert(size(data.baseInTcp, 3) == numFrames && ...
       size(data.boardInCamera, 3) == numFrames && ...
       size(data.imagePoints, 3) == numFrames, 'Robot, camera and image frame counts must match.');
assert(isequal(size(data.cameraMatrix), [3, 4]), 'Expected a 3-by-4 camera matrix.');
assert(size(data.imagePoints, 1) == size(data.worldPoints, 1), 'Corner counts must match.');
assert(all(isfinite(data.imagePoints(:))) && all(isfinite(data.baseInTcp(:))) && ...
       all(isfinite(data.boardInCamera(:))) && all(isfinite(data.cameraMatrix(:))) && ...
       all(isfinite(data.worldPoints(:))), 'Input contains non-finite values.');
baseInTcp = data.baseInTcp;
boardInCamera = data.boardInCamera;
methodNames = {'Li'; 'Tabb rp1'; 'Tabb Zc2'; 'VDF-VDC'};
boardInBase = zeros(4, 4, 4);
cameraInTcp = zeros(4, 4, 4);
elapsedSeconds = zeros(4, 1);
[boardInBase(:, :, 1), cameraInTcp(:, :, 1), elapsedSeconds(1)] = method.Li( ...
    reshape(baseInTcp, 4, []), reshape(boardInCamera, 4, []));
initial = [rotm2quat(boardInBase(1:3, 1:3, 1)), boardInBase(1:3, 4, 1)', ...
           rotm2quat(cameraInTcp(1:3, 1:3, 1)), cameraInTcp(1:3, 4, 1)'];
[boardInBase(:, :, 2), cameraInTcp(:, :, 2), elapsedSeconds(2)] = method.Tabb_rp1( ...
    baseInTcp, initial, data.cameraMatrix, data.imagePoints, data.worldPoints);
[boardInBase(:, :, 3), cameraInTcp(:, :, 3), elapsedSeconds(3)] = method.Tabb_Zc2( ...
    baseInTcp, boardInCamera, initial);
[boardInBase(:, :, 4), cameraInTcp(:, :, 4), elapsedSeconds(4), consistencyRatio] = method.VDF_VDC( ...
    baseInTcp, boardInCamera, cameraInTcp(:, :, 1), boardInBase(:, :, 1), ...
    data.cameraMatrix, data.imagePoints, data.worldPoints);
metrics = zeros(4, 3);
for methodIndex = 1:4
    metrics(methodIndex, :) = evaluateCalibration(baseInTcp, boardInCamera, ...
        boardInBase(:, :, methodIndex), cameraInTcp(:, :, methodIndex), ...
        data.cameraMatrix, data.imagePoints, data.worldPoints);
end
results = table(elapsedSeconds, metrics(:, 1), 1000 * metrics(:, 2), metrics(:, 3), ...
                'VariableNames', {'Time_sec', 'Rerr_deg', 'terr_mm', 'reproj_px'}, 'RowNames', methodNames);
fprintf('Closed-loop consistency ratio q = %.8f\n', consistencyRatio);
disp(results);
% Calibration matrices, metrics and q remain available in the workspace.
clear pathCleanup;
