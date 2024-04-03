clc; clear
close all

parameters;
%% Define Simulation time
Ns = 2000;
t_end = 80;
t_span = linspace(0,t_end,Ns);
dt = t_end / Ns;
%% Desired State
X_des = [0; 0; 2; 10*d2r; 10*d2r; 10*d2r; 0; 0; 0; 0; 0; 0];
%% Initialize U_matrix
U = zeros(4,Ns);

%% Define Initial State
X0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]*d2r; % [x, y, z, phi, theta, psi, u, v, w, p, q, r]
X(:,1) = X0;

for i = 1:Ns-1
    U(:,i) = int_control(X(:, i), X_des);
    X(:, i+1) = X(:, i) + dt * Quad_6dof(X(:,i),U(:,i));
end


%% desired state vectors
roll_des = ones(Ns,1)*X_des(4);
pitch_des = ones(Ns,1)*X_des(5);
yaw_des = ones(Ns,1)*X_des(6);

%% Plots Generation
figure
subplot(4,1,1)
plot(t_span, roll_des*r2d, LineStyle="--", Color='black', LineWidth=2)
hold on
plot(t_span, X(4,:)*r2d, LineWidth=2)
title('Roll')
xlabel('time (s)')
ylabel('phi (deg)')

subplot(4,1,2)
plot(t_span, pitch_des*r2d, LineStyle="--", Color='black', LineWidth=2)
hold on
plot(t_span, X(5,:)*r2d, LineWidth=2)
title('Pitch')
xlabel('time (s)')
ylabel('theta (deg)')

subplot(4,1,3)
plot(t_span, yaw_des*r2d, LineStyle="--", Color='black', LineWidth=2)
hold on
plot(t_span, X(6,:)*r2d, LineWidth=2)
title('Yaw')
xlabel('time (s)')
ylabel('psi (deg)')

subplot(4,1,4)
%plot(t_span, yaw_des*r2d, LineStyle="--", Color='black', LineWidth=2)
hold on
plot(t_span, X(3,:), LineWidth=2)
title('Z')
xlabel('time (s)')
ylabel('Z')