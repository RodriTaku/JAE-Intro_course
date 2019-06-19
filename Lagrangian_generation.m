filepath = fileparts(which(mfilename));
addpath(fullfile(filepath,'\Group_functions'));

%Create 2-link robot

%Parameters:
%   l1 - Length of link 1
%   l2 - Length of link 2
%   r1 - Distance from joint 0 to center of mass of link 1
%   r2 - Distance from joint 1 to center of mass of link 2
%   (Ix1, Iy1, Iz1)
%      - Principal moments of inertia of link 1
%   (Ix2, Iy2, Iz2)
%      - Principal moments of inertia of link 2
%   m1 - Mass of link 1
%   m2 - Mass of link 2
%   g  - Value of gravity
syms l1 l2 r1 r2
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 m1 m2 g

%Generalized inertia matrices
M1 = diag([Ix1, Iy1, Iz1, ones(1,3)*m1]);
M2 = diag([Ix2, Iy2, Iz2, ones(1,3)*m2]);

%Main variables:
%   theta1  - Joint angle 1 / Angle of link 1 w.r.t. ref 0
%   theta2  - Joint angle 2 / Angle of link 2 w.r.t. ref 1
%   vtheta1 - Joint angular velocity 1
%   vtheta2 - Joint angular velocity 2
syms theta1 theta2 vtheta1 vtheta2

%Unit vectors:
v1 = sym([1;0;0]);
v2 = sym([0;1;0]);
v3 = sym([0;0;1]);

%Gravity vector
g_vec = -g*v2;

%Joint twists:
xi1 = [v3;-cross(v3,v1*0)];
xi2 = [v3;-cross(v3,v1*l1)];

%Position and velocity computations
G01 = simplify(SE3_exp( theta1*xi1, theta1 ));
G12 = simplify(SE3_exp( theta2*xi2, theta2 ));

g01_0 = SE3_element( [zeros(3,1); v1*r1] );
g02_0 = SE3_element( [zeros(3,1); v1*(l1+r2)] );

g01 = simplify(G01*g01_0);
g02 = simplify(G01*G12*g02_0);

dg01 = simplify(reshape(jacobian(g01(:),[theta1;theta2])*[vtheta1;vtheta2],size(g01)));
dg02 = simplify(reshape(jacobian(g02(:),[theta1;theta2])*[vtheta1;vtheta2],size(g02)));

v01 = simplify(SE3_vee(SE3_inv(g01)*dg01));
v02 = simplify(SE3_vee(SE3_inv(g02)*dg02));

x1 = g01(1:3,4);
x2 = g02(1:3,4);

%Kinetic and potential energies, Lagrangian
T = simplify((v01.'*M1*v01 + v02.'*M2*v02)/2);
U = simplify(- m1*g_vec.'*x1 - m2*g_vec.'*x2);
L = T - U;

%Torques
syms tau1 tau2
F = [tau1; tau2];

%Derivatives of the Lagrangian and Euler-Lagrange equations
dLdv = simplify(gradient(L,[vtheta1,vtheta2]));
dLdq = simplify(gradient(L,[theta1,theta2]));
A = simplify(jacobian(dLdv,[vtheta1,vtheta2])\(dLdq + F - jacobian(dLdv,[theta1,theta2])*[vtheta1;vtheta2]));
%A = simplify(subs(A,[r1,r2,Iz1,Iz2],[l1/2,l2/2,m1*l1^2/12,m2*l2^2/12]));