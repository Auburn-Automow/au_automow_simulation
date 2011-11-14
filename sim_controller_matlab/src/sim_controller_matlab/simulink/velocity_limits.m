clear; clc;
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

figure(1);hold on;
plot([w_max+1, w_min-1],[0,0],'k');
plot([0,0],[v_min-1,v_max+1],'k');
plot([w_max, w_min],[v_max, v_max],'k--'); 
plot([w_max, w_min],[v_min, v_min],'k--');
xlim([w_min- 1, w_max + 1]); xlabel 'Angular Velocity'
ylim([v_min-1, v_max + 1]); ylabel 'Linear Velocity'
plot([w_min, w_min],[v_min, v_max],'k--');
plot([w_max, w_max],[v_min, v_max],'k--');