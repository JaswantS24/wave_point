function X_dot = quad_dynamics(t, X, U, params)
    %% Unpack the variables
    phi = X(1);
    theta = X(2);
    psi = X(3);
    I = params('Inertia');
    I_xx = I(1,1);
    I_yy = I(2,2);
    I_zz = I(3,3);

    p = X(4);
    q = X(5);
    r = X(6);
    bod_ang_vel = [p; q; r];
    %%calculate euler ang dot 
    ang_dot = rot_mat(phi, theta, psi, 1) * bod_ang_vel;
    %% Calculate thrust coefficients
    l = params('length');
    k_t = params('k_t');
    k_d = params('k_d');
    tau_l = k_t * l * (U(1)*abs(U(1)) - U(2)*abs(U(2)) - U(3)*abs(U(3)) + U(4)*abs(U(4)));
    tau_m = k_t * l * (U(1)*abs(U(1)) + U(2)*abs(U(2)) - U(3)*abs(U(3)) - U(4)*abs(U(4)));
    tau_n = k_d * l * (U(1)*abs(U(1)) - U(2)*abs(U(2)) + U(3)*abs(U(3)) - U(4)*abs(U(4)));
    %% calculate X_dot
    p_dot = ((I_yy - I_zz)*q*r / I_xx) + tau_l/I_xx;
    q_dot = ((I_zz - I_xx)*p*r / I_yy) + tau_m/I_yy;
    r_dot = ((I_xx - I_yy)*p*q / I_zz) + tau_n/I_zz;
    body_ang_vel_dot = [p_dot; q_dot; r_dot];

    X_dot = [ang_dot;body_ang_vel_dot];

    