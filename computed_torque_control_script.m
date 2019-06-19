% Parameters:
%   l1  - Length of link 1
%   l2  - Length of link 2
%   m1  - Mass of link 1
%   m2  - Mass of link 2
%   g   - Value of gravity
%   Kq1 - Position gain joint 1
%   Kq2 - Position gain joint 2
%   Kv1 - Velocity gain joint 1
%   Kv2 - Velocity gain joint 2
param = [1,1,1,1,1,1,2,1,2];

%Tend0 - Total time for the reference trajectory to reach destination
Tend0 = 4;
% Reference trajectory (position, velocity and acceleration necessary)
theta1 = @(t) (t < Tend0)*pi/4.*(cos(pi*t/Tend0)+1);
theta2 = @(t) ((t < Tend0)*(-pi/4).*(3*cos(pi*t/Tend0)+1) + (t >= Tend0).*pi/2);
vtheta1 = @(t) (t < Tend0)*(-pi^2).*sin(pi*t/Tend0)/(4*Tend0);
vtheta2 = @(t) (t < Tend0)*3*pi^2.*sin(pi*t/Tend0)/(4*Tend0);
atheta1 = @(t) (t < Tend0)*(-pi^3).*cos(pi*t/Tend0)/(4*Tend0^2);
atheta2 = @(t) (t < Tend0)*(3*pi^3).*cos(pi*t/Tend0)/(4*Tend0^2);

%Tend  - Total simulation time
Tend = 6;

Yref = {theta1; theta2; vtheta1; vtheta2; atheta1; atheta2};
Y0 = [pi/2; -pi; 0; 0] + rand(4,1)/10;
nsteps = Tend*100;

[T, Y] = ode45(@(T,Y) computed_torque_control_two_link_robot_vec_field(T, Y, Yref, param), linspace(0,Tend,nsteps+1), Y0);
plot_two_link_robot(T, Y, param, 0.01)