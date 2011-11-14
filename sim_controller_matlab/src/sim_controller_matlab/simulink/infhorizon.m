function [ F, G_phi, G_r ] = infhorizon( v, T, N )

    A =     [ 1,    T * v;
              0,    1];
    B_phi = [ T^2 * v^2 / 2;
              T * v];
    B_r =   [ 0, - T * v;
              0, 0];

    %Tatsy preallocation
    F = zeros(2*(N),2);
    G_phi = zeros(2*(N),N);
    G_r = zeros(2*(N),2*(N));

    % Populate F
    for i = 1:2:2*N,
        F(i:i+1,:) = A^((i+1)/2);
    end
    
    % Populate G_phi
    col = F * B_phi;
    for i=1:N,
       G_phi(i*2-1:end,i) = col(1:end-2*(i-1)); 
    end
    
    % Populate G_r
    col = F * B_r;
    for i=1:N;
        G_r(i*2-1:end,2*i-1:2*i) = col(1:end-2*(i-1),:);
    end


end

