function T = se3Exp(xi)
    % SE3EXP Exponential map for xi = [rotation; translation], in rad and m.
    omega = xi(1:3);
    v     = xi(4:6);
    theta = norm(omega);
    if theta < 1e-10
        R = eye(3);
        V = eye(3);
    else
        omega_hat = skewMatrix(omega);
        R = expm(omega_hat);
        A = sin(theta) / theta;
        B = (1 - cos(theta)) / (theta^2);
        C = (1 - A) / (theta^2);
        V = eye(3) + B * omega_hat + C * (omega_hat^2);
    end
    t = V * v(:);
    T = [R, t; 0 0 0 1];
end
