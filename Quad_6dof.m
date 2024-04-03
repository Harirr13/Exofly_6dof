function X_dot = Quad_6dof(X, U)

parameters; 

%% state 
% Inertial frame
x = X(1);
y = X(2);
z = X(3);
% Vehicle, Vehicle 1, Vehicle 2 frames
phi   = X(4);
theta = X(5);
psi   = X(6);
% Body frame 
u = X(7);
v = X(8);
w = X(9);
% Body frame
p = X(10);
q = X(11);
r = X(12);


%% Kinematics 

% Translation 
Pos_dot = RotMat(phi, theta, psi, 5)*[u; v; w]; % Inertial frame

% Rotational 
Eul_dot = RotMat(phi, theta, psi, 7)*[p; q; r]; % Diff Vehicle frames

%% Dynamics 

% Translation 

T = km * (U(1)*abs(U(1)) + U(2)*abs(U(2)) + U(3)*abs(U(3)) + U(4)*abs(U(4)));
L = km * l * (U(1)*abs(U(1)) - U(2)*abs(U(2)) - U(3)*abs(U(3)) + U(4)*abs(U(4)));
M = km * l * (U(1)*abs(U(1)) + U(2)*abs(U(2)) - U(3)*abs(U(3)) - U(4)*abs(U(4)));
N = b * l * (U(1)*abs(U(1)) - U(2)*abs(U(2)) + U(3)*abs(U(3)) - U(4)*abs(U(4)));

Vel_dot   =[0; 0; -T/m] + RotMat(phi, theta, psi, 4)*[0; 0; g] + [r*v-q*w; p*w-r*u; q*u-p*v];

% Rotation Dynamics
Att_dot = [ L/(Ixx) + ((Iyy - Izz)/(Ixx))*q*r;
            M/(Iyy) + ((Izz - Ixx)/(Iyy))*p*r;
            N/(Izz) + ((Ixx - Iyy)/(Izz))*p*q; ]; % Body frame
             
X_dot = [ Pos_dot
          Eul_dot
          Vel_dot
          Att_dot];
