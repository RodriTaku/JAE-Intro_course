function [ Y ] = SO3_ad( X )
%ad computes the ad: \mathfrak{g} \to \mathrm{Aut}\left\mathfrak{g}\right)
% operator. X = [omega; vx; vy] \in \mathfrak{g}, which is represented by
% a vector in \mathbb{R}^3. h is a scale parameter.
    
    Y = SO3_hat(X);
end