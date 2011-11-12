
L = 100;
T = 0.1;
path_points = [0 , 0;
               5 , 5;
               2,  5];
initial_pose = [1,1,1];

lambda = 0.0001;
N = 10;
I_q = eye((N+1)*2);

%% Variable initialization
pose = zeros(L,3);
pose(1,:) = initial_pose;
u_w = zeros(L,2);             % Wheel Velocity input (v_r, v_l)
u = zeros(L,2);
v = zeros(L,1);
w = zeros(L,1);

z = zeros(L,2);             % State Vector z

wheel_radius = 0.1;
wheel_base = 0.5;

F_w = wheel_radius * [1/2,              1/2; 
                      1/wheel_base,     -1/wheel_base];

                  
%% Simulation 
path_iter = 1;
for k = 2:length(pose),
    % Orthogonal position of point on current path
    path_vector = path_points(path_iter+1,:) - path_points(path_iter,:);
    path_vector_next = path_points(path_iter+2,:) - path_points(path_iter+1,:);
    phi_1 = atan2(path_vector(2),path_vector(1));
    phi_2 = atan2(path_vector_next(2),path_vector_next(1));
    alpha = (phi_2 - phi_1)/2;
    
    
    C = path_points(path_iter+1,:)';
    R = pose(k,1:2) - path_points(path_iter,:);
    P = (R' .* path_vector')/(path_vector.^2)' * path_vector';
    % Scale back to original coordinates
    R = pose(k,1:2)';
    P = P + path_points(path_iter,:)';
    
    [F, G_phi, G_r] = infhorizon(1, T, N);
    
    K = (lambda + G_phi' * I_q * G_phi);
    L_z = K\(G_phi' * I_q * F);
    L_r = K\(G_phi' * I_q * (G_r - eye(2*(N+1))));
    
    pose(k,1) = pose(k-1, 1) + u(k,1) * cos(pose(k-1,3));
    pose(k,2) = pose(k-1, 2) + u(k,1) * sin(pose(k-1,3));
    pose(k,3) = pose(k-1, 3) + u(k,2);
end
    