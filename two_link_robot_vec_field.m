function [dY] = two_link_robot_vec_field(T, Y, F, param)
%TWO_LINK_ROBOT_VEC_FIELD evaluates the vector field of a planar two-link
%robot.
%
%     It is assumed that the center of mass of each link is at its
%     geometric center. It is also assumed that the principal moment of
%     inertia about its local z axis (perpendicular to the plane of motion)
%     is m/12*l^2
% 
%     Arguments:
%       T - Time (scalar or vector)
%       Y - State vector [theta1; theta2; vtheta1; vtheta2]
%       F - Torques {tau1(T,Y), tau2(T,Y)} (cell array of function handles)
%       param - Parameters of the model [l1, l2, m1, m2, g] (vector)
% 
%     Parameters:
%       l1 - Length of link 1
%       l2 - Length of link 2
%       m1 - Mass of link 1
%       m2 - Mass of link 2
%       g  - Value of gravity
%
%     Example (using ode45):
%       param = [1,1,1,1,1];
%       F = {@(t,y) 0; @(t,y) cos(t)}
%       t0 = 0;
%       t1 = 100;
%       nsteps = 1000;
%       q0 = [0; 0];
%       v0 = [0; 0];
%       [T,Y] = ode45(@(T,Y) two_link_robot_vec(T,Y,param), linspace(t0,t1,nsteps+1), [q0; v0]);

    m1 = param(1);
    m2 = param(2);
    l1 = param(3);
    l2 = param(4);
    g = param(5);

    tau1 = F{1}(T,Y);
    tau2 = F{2}(T,Y);

    dY = ...
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   Y(3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        Y(4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                  (24*l2*tau1 - 24*l2*tau2 - 36*l1*tau2*cos(Y(2)) + 9*g*l1*l2*m2*cos(Y(1) + 2*Y(2)) + 9*l1^2*l2*m2*Y(3)^2*sin(2*Y(2)) + 12*l1*l2^2*m2*Y(3)^2*sin(Y(2)) + 12*l1*l2^2*m2*Y(4)^2*sin(Y(2)) - 12*g*l1*l2*m1*cos(Y(1)) - 15*g*l1*l2*m2*cos(Y(1)) + 24*l1*l2^2*m2*Y(3)*Y(4)*sin(Y(2)))/(l1^2*l2*(8*m1 + 15*m2 - 9*m2*cos(2*Y(2))));
     -(24*l2^2*m2*tau1 - 72*l1^2*m2*tau2 - 24*l1^2*m1*tau2 - 24*l2^2*m2*tau2 + 12*l1*l2^3*m2^2*Y(3)^2*sin(Y(2)) + 36*l1^3*l2*m2^2*Y(3)^2*sin(Y(2)) + 12*l1*l2^3*m2^2*Y(4)^2*sin(Y(2)) + 36*g*l1^2*l2*m2^2*cos(Y(1) + Y(2)) - 24*g*l1*l2^2*m2^2*cos(Y(1)) + 18*l1^2*l2^2*m2^2*Y(3)^2*sin(2*Y(2)) + 9*l1^2*l2^2*m2^2*Y(4)^2*sin(2*Y(2)) + 36*l1*l2*m2*tau1*cos(Y(2)) - 72*l1*l2*m2*tau2*cos(Y(2)) + 12*l1^3*l2*m1*m2*Y(3)^2*sin(Y(2)) + 24*l1*l2^3*m2^2*Y(3)*Y(4)*sin(Y(2)) + 18*g*l1*l2^2*m2^2*cos(Y(1) + Y(2))*cos(Y(2)) - 36*g*l1^2*l2*m2^2*cos(Y(1))*cos(Y(2)) + 12*g*l1^2*l2*m1*m2*cos(Y(1) + Y(2)) - 12*g*l1*l2^2*m1*m2*cos(Y(1)) + 18*l1^2*l2^2*m2^2*Y(3)*Y(4)*sin(2*Y(2)) - 18*g*l1^2*l2*m1*m2*cos(Y(1))*cos(Y(2)))/(2*l1^2*l2^2*m2*(4*m1 + 12*m2 - 9*m2*cos(Y(2))^2))];
end

