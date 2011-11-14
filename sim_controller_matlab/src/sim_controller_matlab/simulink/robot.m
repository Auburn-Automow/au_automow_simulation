clear; clc

%%
L = 5000;
T = 0.05;
path_points = [ 1.0 , 1.0 ;  1.0 , 5.0 ; 5.0, 5.0; 5.0, 1.0; 1.0, 1.0; 1.0, 5.0];
initial_pose = [1.01,1,pi/2];

lambda = 0.0001;
N = 10;

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

F_w = wheel_radius *[1/2,              1/2; 
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

start = true;

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
    if start,
        Ref = repmat([0;phi_1],N,1);
    end
    
    [u(k,:), gamma, phi] = LinearRecedingHorizon(Ref,z(k,:)',P,q,v_des,T, N, lambda, I_q);
    
    prpose = drawPlan(pose(k,:),gamma,phi,v_des,T);
    
    figure(1); clf;
    plot(path_points(:,1), path_points(:,2)'); hold on;
    scatter(prpose(:,1),prpose(:,2)); hold on;
    axis([-1, 11, -1, 11]);
    
    pose(k+1,1) = pose(k, 1) + T * u(k,1) * cos(pose(k,3) + 0.5 * T * -u(k,2));
    pose(k+1,2) = pose(k, 2) + T * u(k,1) * sin(pose(k,3) + 0.5 * T * -u(k,2));
    pose(k+1,3) = wrapToPi(pose(k, 3) + T * u(k,2));
    
    n_hat = nstepIntersect(pose(k,:), gamma, phi, path_points, path_iter, v_des);
    if n_hat > -1,
        start = false;
        iters = length(Ref)/2;
        if n_hat < iters,
            Ref(n_hat*2+1:end) = repmat([0;phi_2],iters-n_hat,1);
        end
        if n_hat < 2,
            path_iter = path_iter +1;
            start = true;
        end
    end

    plot(pose(1:k,1),pose(1:k,2),'r');
    M(:,k) = getframe;
end