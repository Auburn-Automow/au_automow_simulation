function [ F, G_phi ] = controller( z, phi, r, u, T, N )

    A =     [ 1,    T * u(1);
              0,    1];
    B_phi = [ T^2 * u(1)^2 / 2;
              T * u(1)];
    B_r =   [ 0, - T * u(1);
              0, 0];

    %Tatsy preallocation
    F = zeros(2*(N+1),2);
    G_phi = zeros(2*(N+1),N+1);
    G_r = zeros(2*(N+1),2*(N+1));

    % Populate F
    F(1:2,:) = eye(2);
    for i = 2:N+1,
        F(i*2-1:i*2,:) = A^(i-1);
    end
    
    % Populate G_phi
    col = F * B_phi;
    for i=1:N+1,
       G_phi(1+2*i:end,i) = col(1:end-2*i); 
    end
    


end

