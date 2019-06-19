function [ Y ] = SE3_ad( X )
%ad computes the ad: \mathfrak{g} \to \mathrm{Aut}\left\mathfrak{g}\right)
% operator. X = [omega; vx; vy] \in \mathfrak{g}, which is represented by
% a vector in \mathbb{R}^3. h is a scale parameter.
    
    omega = X(1:3);
    v = X(4:6);
    hat_omega = SO3_hat(omega);
    hat_v = SO3_hat(v);
    Y = [[hat_omega, zeros(3,3)];
         [    hat_v,  hat_omega]];
end