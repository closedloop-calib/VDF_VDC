function [boardInBase, cameraInTcp, elapsedSeconds] = Li(stackedRobotPoses, stackedCameraPoses)
    % CALIBRATELI Li baseline: A_i * boardInBase = cameraInTcp * B_i.
    % Original solver description follows.
    % Solves the problem AX=YB
    % using the formulation of
    %
    % Simultaneous robot-world and hand-eye calibration
    % using dual-quaternions and Kronecker product
    %
    % Aiguo Li, Lin Wang and Defeng Wu
    %
    % Mili Shah
    % July 2014

    tic;
    [~, n] = size(stackedRobotPoses);
    n = n / 4;

    A = zeros(12 * n, 24);
    b = zeros(12 * n, 1);
    for i = 1:n
        Ra = stackedRobotPoses(1:3, 4 * i - 3:4 * i - 1);
        Rb = stackedCameraPoses(1:3, 4 * i - 3:4 * i - 1);
        ta = stackedRobotPoses(1:3, 4 * i);
        tb = stackedCameraPoses(1:3, 4 * i);
        A(12 * i - 11:12 * i - 3, 1:18) = [kron(Ra, eye(3)) kron(-eye(3), Rb')];
        A(12 * i - 2:12 * i, 10:24) = [kron(eye(3), tb') -Ra eye(3)];
        b(12 * i - 2:12 * i) = ta;
    end
    x = A \ b;

    boardInBase = reshape(x(1:9), 3, 3)';
    [u, ~, v] = svd(boardInBase);
    boardInBase = u * v';
    if det(boardInBase) < 0
        boardInBase = u * diag([1 1 -1]) * v';
    end
    boardInBase = [boardInBase x(19:21); [0 0 0 1]];
    cameraInTcp = reshape(x(10:18), 3, 3)';
    [u, ~, v] = svd(cameraInTcp);
    cameraInTcp = u * v';
    if det(cameraInTcp) < 0
        cameraInTcp = u * diag([1 1 -1]) * v';
    end
    cameraInTcp = [cameraInTcp x(22:24); [0 0 0 1]];
    elapsedSeconds = toc;
end
