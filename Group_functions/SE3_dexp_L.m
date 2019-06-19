function [ Y ] = SE3_dexp_L( X, varargin )
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
    A = eye(3) - (1-cos(theta))/theta^2*hat_omega + (theta-sin(theta))/theta^3*hat_omega2;
    B = - 1/2*hat_v ...
        + (theta-sin(theta))/theta^3*(hat_omega*hat_v+hat_v*hat_omega) ...
        - (cos(theta)+theta^2/2-1)/theta^4*(hat_omega2*hat_v+hat_v*hat_omega2) ...
        - (1-cos(theta)-theta*sin(theta)/2)/theta^4*hat_omega*hat_v*hat_omega ...
        + (theta-3*sin(theta)/2+theta*cos(theta)/2)/theta^5*(hat_omega2*hat_v*hat_omega+hat_omega*hat_v*hat_omega2) ...
        - (theta^2/2+theta*sin(theta)/2+2*cos(theta)-2)/theta^6*hat_omega2*hat_v*hat_omega2;
    Y = [[ A, zeros(3,3)];
         [ B,          A]];
end