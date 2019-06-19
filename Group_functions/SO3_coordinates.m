function [ Y ] = SO3_coordinates( X, varargin )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
% Euler angles: Algorithm by Gregory G. Slabaugh
    nvarargs = length(varargin);
    if nvarargs == 0
        is_inverted = 0;
    elseif nvarargs == 1
        is_inverted = varargin{1};
    else
        error('Unexpected number of arguments')
    end

    if (X(3,1) ~= 1) && (X(3,1) ~= -1)
        theta = -asin(X(3,1));
        if is_inverted
            theta = pi-theta;
        end
        psi = atan2(X(3,2)/cos(theta),X(3,3)/cos(theta));
        phi = atan2(X(2,1)/cos(theta),X(1,1)/cos(theta));
    else
        phi = 0;
        if X(3,1) == -1
            theta = pi/2;
            psi = phi + atan2(X(1,2),X(1,3));
        else
            theta = -pi/2;
            psi = -phi + atan2(-X(1,2),-X(1,3));
        end
    end
    Y = [   psi;
          theta;
            phi];
end