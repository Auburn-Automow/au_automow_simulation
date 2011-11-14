clear; clc

%%
L = 500;
T = 0.05;
path_points = [0 , 0;
               10 , 0;
               0, 10];
initial_pose = [-3,-3,0.01];

lambda = 0.00001;
N = 20;
Q = [1,0.02];
I_q = diag(repmat(Q,1,N));
v_des = 2;

%% Variable initialization
pose = zeros(L,3);
pose(1,:) = initial_pose;
u_w = zeros(L,2);             % Wheel Velocity input (v_r, v_l)
u = zeros(L,2);
v = zeros(L,1);
w = zeros(L,1);
z = zeros(L,2);             % State Vector z

%% Velocity Limits
% Sizes for Kinematics (Measured)
wheel_radius = 0.159;
wheel_base = 0.5461;

% Maximum wheel velocity
vw_min = 2;
vw_max = 2;

% Maximum linear and angular velocities
v_min = 1;
v_max = 1;
w_min = 2*pi/10;
w_max = 2*pi/10;

F_w = [1/2,              1/2; 
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
                  
%% Simulation 
path_iter = 1;
k = 1;

figure(1); clf;
nframes = L;
M = moviein(nframes);

for k = 1:L,  
    path_vector = path_points(path_iter+1,:) - path_points(path_iter,:);
    path_vector_n = path_points(path_iter+2,:) - path_points(path_iter+1,:);
    phi_1 = atan2(path_vector(2), path_vector(1));
    phi_2 = atan2(path_vector_n(2),path_vector_n(1));
    
    d = norm(det([path_vector; ...
                 pose(k,1:2) - path_points(path_iter,:)]))/...
                 norm(path_vector);
    perp = [-path_vector(2),path_vector(1)];
    d = d * sign(dot(pose(k,1:2) - path_points(path_iter,:), perp));
    
    theta_hat = wrapToPi(pose(k,3) - phi_1);

    z(k,:) = [d; theta_hat];
    R = repmat([0;phi_1],N,1);
    
    [u(k,:), gamma, phi] = LinearRecedingHorizon(R,z(k,:)',P,q,v_des,T, N, lambda, I_q);
    
    prpose = drawPlan(pose(k,:),gamma,phi,v_des,T);
    
    figure(1); clf;
    plot(path_points(1:2,1), path_points(1:2,2)'); hold on;
    scatter(prpose(:,1),prpose(:,2)); hold on;
    axis([-5, 10, -5, 5]);
    
    pose(k+1,1) = pose(k, 1) + T * u(k,1) * cos(pose(k,3) + 0.5 * T * u(k,2));
    pose(k+1,2) = pose(k, 2) + T * u(k,1) * sin(pose(k,3) + 0.5 * T * u(k,2));
    pose(k+1,3) = pose(k, 3) + T * u(k,2);
    
    plot(pose(1:k,1),pose(1:k,2));
    M(:,k) = getframe;
end