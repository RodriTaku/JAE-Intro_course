function [dY] = computed_torque_control_two_link_robot_vec_field(T, Y, Yref, param)
%COMPUTED_TORQUE_CONTROL_TWO_LINK_ROBOT_VEC_FIELD evaluates the vector
%field of a planar two-link robot being controlled using the 'computed
%torque control' method.
%
%     It is assumed that the center of mass of each link is at its
%     geometric center. It is also assumed that the principal moment of
%     inertia about its local z axis (perpendicular to the plane of motion)
%     is m/12*l^2. For the 'computed torque control' we assume diagonal
%     gain matrices.
% 
%     Arguments:
%       T - Time (scalar or vector)
%       Y - State vector [theta1; theta2; vtheta1; vtheta2]
%       Yref  - Reference trajectory
%               {theta1(T,Tend), theta2(T,Tend), ...
%                vtheta1(T,Tend), vtheta2(T,Tend), ...
%                atheta1(T,Tend), atheta2(T,Tend)}
%               (cell array of function handles)
%       param - Parameters of the model
%               [l1, l2, m1, m2, g, Kq1, Kq2, Kv1, Kv2] (vector)
% 
%     Parameters:
%       l1  - Length of link 1
%       l2  - Length of link 2
%       m1  - Mass of link 1
%       m2  - Mass of link 2
%       g   - Value of gravity
%       Kq1 - Position gain joint 1
%       Kq2 - Position gain joint 2
%       Kv1 - Velocity gain joint 1
%       Kv2 - Velocity gain joint 2

    m1 = param(1);
    m2 = param(2);
    l1 = param(3);
    l2 = param(4);
    g = param(5);
    Kq1 = param(6);
    Kq2 = param(7);
    Kv1 = param(8);
    Kv2 = param(9);

    X = [Yref{1}(T);
         Yref{2}(T);
         Yref{3}(T);
         Yref{4}(T);
         Yref{5}(T);
         Yref{6}(T)];

    Z = ...
    [ l1^2*m2*(X(5) + Kq1*(X(1) - Y(1)) + Kv1*(X(3) - Y(3))) + l2^2*m2*(X(5)/3 + X(6)/3 + (Kq1*(X(1) - Y(1)))/3 + (Kq2*(X(2) - Y(2)))/3 + (Kv1*(X(3) - Y(3)))/3 + (Kv2*(X(4) - Y(4)))/3) + l1^2*m1*(X(5)/3 + (Kq1*(X(1) - Y(1)))/3 + (Kv1*(X(3) - Y(3)))/3) + (g*l2*m2*cos(Y(1) + Y(2)))/2 + l1*l2*m2*(cos(Y(2))*(X(5) + Kq1*(X(1) - Y(1)) + Kv1*(X(3) - Y(3))) + (cos(Y(2))*(X(6) + Kq2*(X(2) - Y(2)) + Kv2*(X(4) - Y(4))))/2 - Y(4)*(Y(3)*sin(Y(2)) + (Y(4)*sin(Y(2)))/2)) + (g*l1*m1*cos(Y(1)))/2 + g*l1*m2*cos(Y(1));
                                                                                                                                                                                                                                                               l2^2*m2*(X(5)/3 + X(6)/3 + (Kq1*(X(1) - Y(1)))/3 + (Kq2*(X(2) - Y(2)))/3 + (Kv1*(X(3) - Y(3)))/3 + (Kv2*(X(4) - Y(4)))/3) + (g*l2*m2*cos(Y(1) + Y(2)))/2 + l1*l2*m2*((cos(Y(2))*(X(5) + Kq1*(X(1) - Y(1)) + Kv1*(X(3) - Y(3))))/2 + (Y(3)^2*sin(Y(2)))/2)];

    dY = ...
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       Y(3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            Y(4);
                                                                                                                                                                                                                                                                                                                                                                                (l2*(12*Z(1) - 12*Z(2)) - 18*Z(2)*l1*cos(Y(2)) + l1*l2^2*m2*(6*sin(Y(2))*Y(3)^2 + 12*sin(Y(2))*Y(3)*Y(4) + 6*sin(Y(2))*Y(4)^2) - l1*l2*m2*(12*g*cos(Y(1)) - 9*g*cos(Y(1) + Y(2))*cos(Y(2))) - 6*g*l1*l2*m1*cos(Y(1)) + 9*Y(3)^2*l1^2*l2*m2*cos(Y(2))*sin(Y(2)))/(4*l1^2*l2*m1 - l1^2*l2*m2*(9*cos(Y(2))^2 - 12));
     (l2^2*m2*(12*Z(1) - 12*Z(2)) - 12*Z(2)*l1^2*m1 - 36*Z(2)*l1^2*m2 + l1^2*l2^2*m2^2*(18*cos(Y(2))*sin(Y(2))*Y(3)^2 + 18*cos(Y(2))*sin(Y(2))*Y(3)*Y(4) + 9*cos(Y(2))*sin(Y(2))*Y(4)^2) - l1*l2^2*m2^2*(12*g*cos(Y(1)) - 9*g*cos(Y(1) + Y(2))*cos(Y(2))) + l1^2*l2*m2^2*(18*g*cos(Y(1) + Y(2)) - 18*g*cos(Y(1))*cos(Y(2))) + l1*l2^3*m2^2*(6*sin(Y(2))*Y(3)^2 + 12*sin(Y(2))*Y(3)*Y(4) + 6*sin(Y(2))*Y(4)^2) + l1*l2*m2*(18*Z(1)*cos(Y(2)) - 36*Z(2)*cos(Y(2))) + l1^2*l2*m1*m2*(6*g*cos(Y(1) + Y(2)) - 9*g*cos(Y(1))*cos(Y(2))) + 18*Y(3)^2*l1^3*l2*m2^2*sin(Y(2)) - 6*g*l1*l2^2*m1*m2*cos(Y(1)) + 6*Y(3)^2*l1^3*l2*m1*m2*sin(Y(2)))/((9*cos(Y(2))^2 - 12)*l1^2*l2^2*m2^2 - 4*m1*l1^2*l2^2*m2)];
end