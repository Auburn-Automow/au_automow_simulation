function [ n ] = nstepIntersect( pose, Gamma, Phi, path_points, path_iter, v_des)
%NSTEPINTERSECT Summary of this function goes here
%   Detailed explanation goes here

T = 0.05;
v = Gamma * v_des;
w = v .* Phi;

prpose = zeros(length(Gamma),3);
prpose(1,:) = pose;

path_vector = path_points(path_iter+1,:) - path_points(path_iter,:);
path_vector_n = path_points(path_iter+2,:) - path_points(path_iter+1,:);
phi_1 = atan2(path_vector(2), path_vector(1));
phi_2 = atan2(path_vector_n(2),path_vector_n(1));

for k = 1:length(Gamma),
    prpose(k+1,1) = prpose(k, 1) + T * v(k) * cos(prpose(k,3) + 0.5 * T * w(k));
    prpose(k+1,2) = prpose(k, 2) + T * v(k) * sin(prpose(k,3) + 0.5 * T * w(k));
    prpose(k+1,3) = prpose(k, 3) + T * w(k);
    
    R = [prpose(k,1),prpose(k,2)];
    P = point_to_line(path_points(path_iter+1,:),path_points(path_iter,:),R);
    C = path_points(path_iter+1,:);
    d = norm(det([path_vector; ...
                 prpose(k,1:2) - path_points(path_iter,:)]))/...
                 norm(path_vector);
    perp = [-path_vector(2),path_vector(1)];
    d(k) = d * sign(dot(prpose(k,1:2) - path_points(path_iter,:), perp));
    
    s(k) = sqrt((P(1)-C(1))^2 + (P(2)-C(2))^2);
end

n = -1;

for i=1:length(s),
    if s(i) < 0.1,
        n = i;
        break;
    end
end


end

