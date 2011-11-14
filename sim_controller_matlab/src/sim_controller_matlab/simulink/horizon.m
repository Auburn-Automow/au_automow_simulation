%% Velocity Limits
% Sizes for Kinematics (Measured)
wheel_radius = 0.159;
wheel_base = 0.5461;

% Maximum wheel velocity
vw_min = -6;
vw_max = 6;

% Maximum linear and angular velocities
v_min = -1;
v_max = 2;
w_min = -1;
w_max = 1;

F_w = wheel_radius * [1/2,              1/2; 
                      1/wheel_base,     -1/wheel_base];
                  
% Compact form (4.31)

P = zeros(8,2);
P(1:2,:) = F_w^-1;
P(3:4,:) = -F_w^-1;
P(5:6,:) = [0, 1; 0, -1];
P(7:8,:) = [1, 0; -1, 0];

q = zeros(8,1);
q(1:2) = vw_max * ones(2,1);
q(3:4) = -vw_min * ones(2,1);
q(5:8) = [w_max, -w_min, v_max, -v_min];

%% Simulation Params
L = 1000;
T = 0.1;
path_points = [0 , 0;
               10 , 0;
               2,  5];
initial_pose = [1,1,0];

lambda = 0.0001;
N = 50;
I_q = eye((N+1)*2);
v_des = 0.5;

%% N step predictor

[F, G_phi, G_r] = infhorizon(v_des, T, N);
d = 1;
phi_1 = 0;
theta_hat = 0;

K = (lambda + G_phi' * I_q * G_phi);
L_z = K\(G_phi' * I_q * F);
L_r = K\(G_phi' * I_q * (G_r - eye(2*(N+1))));

Phi = -L_z * [d;theta_hat] - L_r * repmat([0;phi_1],N+1,1);

%% Velocity Scaling
Gamma = ones(N,1);
for n = 1:N,
    P_prime = P * [1; Phi(n)] * v_des;
    minimize = q./P_prime;
    for i = 1:8,
        if minimize(i) > 0,
            Gamma(n) = min([gamma(n),minimize(i)]);
        end
    end
end

%% Velocity Selection
gamma = Gamma(1);
phi = Phi(1);
v = gamma * v_des;
omega = v * phi;





