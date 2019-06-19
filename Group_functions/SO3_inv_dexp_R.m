function [ Y ] = SO3_inv_dexp_R( X, varargin )
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
    Y = eye(3) - 1/2*hat_omega + (theta*sin(theta)/2 + cos(theta) - 1)/((cos(theta)-1)*theta^2)*hat_omega2;
end