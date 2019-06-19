function [ Y ] = SE3_inv( X )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    R = X(1:3,1:3);
    t = X(1:3,4);
    Y = [[        R.', -R.'*t];
         [ zeros(1,3),      1]];
end