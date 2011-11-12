function [ F, G_phi, G_r ] = infhorizon( v, T, N )

    A =     [ 1,    T * v;
              0,    1];
    B_phi = [ T^2 * v^2 / 2;
              T * v];
    B_r =   [ 0, - T * v;
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
    
    % Populate G_r
    col = F * B_r;
    for i=1:N;
        G_r(1+2*i:end,(2*i-1):2*i) = col(1:end-2*i,:);
    end


end

