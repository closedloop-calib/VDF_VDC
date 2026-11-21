function xi = se3Log(T)
    % SE3LOG Logarithm map, returning [rotation; translation] in rad and m.
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    c = (trace(R) - 1) / 2;
    c = max(min(c, 1), -1);
    theta = acos(c);
    if abs(theta) < 1e-10
        omega = [0; 0; 0];
        V_inv = eye(3);
    else
        omega_hat = (R - R') * (theta / (2 * sin(theta)));
        omega = [omega_hat(3, 2); omega_hat(1, 3); omega_hat(2, 1)];
        A = sin(theta) / theta;
        B = (1 - cos(theta)) / (theta^2);
        V_inv = eye(3) - 0.5 * omega_hat + (1 / (theta^2)) * (1 - A / (2 * B)) * (omega_hat^2);
    end
    v = V_inv * t(:);
    xi = [omega(:); v(:)];
end
