function [ Y ] = SO3_element( X )
% Rotation matrix:
% X = [psi; theta; phi];
% Rx = [[ 1,        0,         0];
%       [ 0, cos(psi), -sin(psi)];
%       [ 0, sin(psi),  cos(psi)]];
% Ry = [[ cos(theta), 0, sin(theta)];
%       [          0, 1,          0];
%       [-sin(theta), 0, cos(theta)]];
% Rz = [[ cos(phi), -sin(phi), 0];
%       [ sin(phi),  cos(phi), 0];
%       [        0,         0, 1]];
% Y = Rz(phi)*Ry(theta)*Rx(psi);

    psi = X(1);
    theta = X(2);
    phi = X(3);
    Y = [[ cos(phi)*cos(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)];
         [ cos(theta)*sin(phi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi)];
         [         -sin(theta),                              cos(theta)*sin(psi),                              cos(psi)*cos(theta)]];
end