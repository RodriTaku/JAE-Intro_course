function [ Y ] = SE3_Adj( X )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    
    R = X(1:3,1:3);
    t = X(1:3,4);
    Z = SO3_hat( t );
    Y = [[   R, zeros(3,3)];
         [ Z*R,          R]];
end