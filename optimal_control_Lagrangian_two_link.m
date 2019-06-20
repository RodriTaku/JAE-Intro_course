%Parameters:
%   l1 - Length of link 1
%   l2 - Length of link 2
%   m1 - Mass of link 1
%   m2 - Mass of link 2
%   g  - Value of gravity
syms m1 m2 l1 l2 g
param = [m1 m2 l1 l2 g];

%Main variables:
%   thetai  - Joint angle i / Angle of link 1 w.r.t. ref i-1
%   vthetai - Joint angular velocity i
%   muthetai  - Costate of joint angle i
%   muvthetai - Costate of joint angular velocity i
%   dthetai  - Differential of joint angle i
%   dvthetai - differential of joint angular velocity i
%   dmuthetai  - Differential of costate of joint angle i
%   dmuvthetai - Differential of costate of joint angular velocity i
%   ui - Control torque on joint i
syms theta1 theta2 vtheta1 vtheta2
syms theta1 theta2 vtheta1 vtheta2
syms dtheta1 dtheta2 dvtheta1 dvtheta2
syms mutheta1 mutheta2 muvtheta1 muvtheta2
syms dmutheta1 dmutheta2 dmuvtheta1 dmuvtheta2
syms u1 u2

Y = [theta1; theta2; vtheta1; vtheta2];
dY = [dtheta1; dtheta2; dvtheta1; dvtheta2];
Z = [mutheta1; mutheta2; muvtheta1; muvtheta2];
dZ = [dmutheta1; dmutheta2; dmuvtheta1; dmuvtheta2];
U = [u1; u2];

F = two_link_robot_vec_field(0,Y,{@(t,y) u1; @(t,y) u2},param);

%Constraint
N = Z.'*(dY-F);
%Cost function
C = U.'*U/2;
%Lagrangian
L = C + N;

%Variational equations
dLdY = gradient(L,[Y;Z;U]);
dLddY = [gradient(L,dY);gradient(L,dZ);zeros(2,1)];
ddLddY = jacobian(dLddY,[Y;Z])*[dY;dZ];
EQS = simplify(ddLddY - dLdY);