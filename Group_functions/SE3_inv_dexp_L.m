function [ Y ] = SE3_inv_dexp_L( X, varargin )
	nvarargs = length(varargin);
    if nvarargs == 0
        theta = sqrt(X(1:3).'*X(1:3));
    elseif nvarargs == 1
        theta = varargin{1};
    else
        error('Unexpected number of arguments')
    end
    omega = X(1:3);
    v = X(4:6);
    hat_omega = SO3_hat(omega);
    hat_omega2 = hat_omega*hat_omega;
    hat_v = SO3_hat(v);
    A = eye(3) + 1/2*hat_omega + (theta*sin(theta)/2 + cos(theta) - 1)/((cos(theta)-1)*theta^2)*hat_omega2;
    B = 1/2*hat_v ...
        + (theta*sin(theta)/2 + cos(theta) - 1)/((cos(theta)-1)*theta^2)*(hat_omega*hat_v+hat_v*hat_omega) ...
        + (theta^2/4 + theta*sin(theta)/4 + cos(theta) - 1)/((cos(theta)-1)*theta^4)*(hat_omega2*hat_v*hat_omega+hat_omega*hat_v*hat_omega2);
    Y = [[ A, zeros(3,3)];
         [ B,          A]];
end