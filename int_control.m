function u = int_control(X, X_des)
    %% Feed forward input
    u_ff = 1;
    %% Simple PID Controller
    %PD controller
    % kp_z = 300;
    % kd_z = 300;
    kp_rpy = [13; 10; 5];
    kd_rpy = [150; 150; 6];
    %current errors
    ang_error = X_des(4:6) - X(4:6);
    ang_rate_error = X_des(10:12) - X(10:12);
    % z_error = X_des(3) - X(3);
    % z_rate_error = X_des(9) - X(9);

    % z_cnt = kp_z*z_error + kd_z*z_rate_error;
    r_cnt = kp_rpy(1)*ang_error(1) + kd_rpy(1) * ang_rate_error(1);
    p_cnt = kp_rpy(2)*ang_error(2) + kd_rpy(2) * ang_rate_error(2);
    y_cnt = kp_rpy(3)*ang_error(3) + kd_rpy(3) * ang_rate_error(3);

    %% Motor Mixing Algorithm
    m(1) = p_cnt + r_cnt + y_cnt + u_ff;
    m(2) = p_cnt - r_cnt - y_cnt + u_ff;
    m(3) = -p_cnt - r_cnt + y_cnt + u_ff;
    m(4) = -p_cnt + r_cnt - y_cnt + u_ff;
    u = [m(1); m(2); m(3); m(4)];
end