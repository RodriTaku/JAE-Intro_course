function [ Y ] = SO3_exp( X, varargin )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
	nvarargs = length(varargin);
    if nvarargs == 0
        theta = sqrt(X(1:3).'*X(1:3));
    elseif nvarargs == 1
        theta = varargin{1};
    else
        error('Unexpected number of arguments')
    end

    hat_omega = SO3_hat(X);
    hat_omega2 = hat_omega*hat_omega;
    Y = eye(3) + sin(theta)/theta*hat_omega + (1-cos(theta))/theta^2*hat_omega2;
end