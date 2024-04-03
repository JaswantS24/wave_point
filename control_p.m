function u = control_p(t,X,XI, X_des,kp_rpy,ki_rpy,kd_rpy)
    %% calculating errors
    p_error = X_des(1:3)-X(1:3);
    d_error = X_des(4:6)-X(4:6);
    i_error = XI;
    rpy = [0,0,0];
    %%calculating the new values of rpy
    for i = 1:3
        rpy(i) = p_error(i)*kp_rpy(i) + i_error(i)*ki_rpy(i) + d_error(i)*kd_rpy(i);
    end

    %motor mixing algorithm
    m(1) = rpy(2) + rpy(1) + rpy(3);
    m(2) = rpy(2) - rpy(1) - rpy(3);
    m(3) = -rpy(2) - rpy(1) + rpy(3);
    m(4) = -rpy(2) + rpy(1) - rpy(3);
    u = [m(1); m(2); m(3); m(4)];
