function [ Y ] = SE3_hat( X )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    Y = [[ SO3_hat(X(1:3)), X(4:6)];
                         zeros(1,4)];
end