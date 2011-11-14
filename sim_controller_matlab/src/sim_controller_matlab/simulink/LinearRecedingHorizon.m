function [ u, Gamma, Phi ] = LinearRecedingHorizon( R, z, P,q, v_des,T, N, lambda, I_q )

    [F, G_phi, G_r] = infhorizon(v_des, T, N);
     
    K = (lambda + G_phi' * I_q * G_phi);
    L_z = K\G_phi' * I_q * F;
    L_r = K\G_phi' * I_q * (G_r - eye(N*2));
   
    Phi = -L_z *[sin(2*z(2))/(2*z(2)),0;0,1] * z - L_r * R;
    
    Gamma = ones(N,1);
    for n = 1:N,
        P_prime = P * [1; Phi(n)] * v_des;
        minimize = q./P_prime;
        for i = 1:8,
            if minimize(i) > 0,
                Gamma(n) = min([Gamma(n),minimize(i)]);
            end
        end
    end
    
    gamma = Gamma(1);
    phi = Phi(1);
    v = gamma * v_des;
    omega = v * phi;
    
    u = [v, omega];
end

