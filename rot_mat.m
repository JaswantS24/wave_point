function r = rot_mat(phi, theta, psi, option)
    pqr2ang_dot = [
        1 sin(phi)*tan(theta) cos(phi)*tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi)/cos(theta) cos(phi)/cos(theta)
    ];
    if option == 1
        r = pqr2ang_dot;
    end
end