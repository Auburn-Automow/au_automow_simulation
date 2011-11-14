function [ prpose ] = drawPlan( pose, Gamma, Phi, v_des,T )

v = Gamma * v_des;
w = v .* Phi;

prpose = zeros(length(Gamma),3);
prpose(1,:) = pose;

for k = 1:length(Gamma),
    prpose(k+1,1) = prpose(k, 1) + T * v(k) * cos(prpose(k,3) + 0.5 * T * w(k));
    prpose(k+1,2) = prpose(k, 2) + T * v(k) * sin(prpose(k,3) + 0.5 * T * w(k));
    prpose(k+1,3) = prpose(k, 3) + T * w(k);
end

end

