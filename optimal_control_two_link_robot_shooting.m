function [ T, Y, X ] = optimal_control_two_link_robot_shooting( X0, Tend, param, varargin )
%OPTIMAL_CONTROL_TWO_LINK_ROBOT_SHOOTING tries to solve the boundary value
%problem associated to the optimal control of a planar two-link robot
%moving from pose Y0 to pose Y1 via the shooting method.
%
%     It is assumed that the center of mass of each link is at its
%     geometric center. It is also assumed that the principal moment of
%     inertia about its local z axis (perpendicular to the plane of motion)
%     is m/12*l^2
% 
%     Arguments:
%       X0 - Initial guess
%            [ mutheta1_0; mutheta2_0; muvtheta1_0; muvtheta2_0]
%       Tend   - Total time to reach opposite boundary
%       param  - Parameters of the model [l1, l2, m1, m2, g] (vector)
%       Y0 - (Optional) Initial state vector
%            [ theta1_0; theta2_0; vtheta1_0; vtheta2_0]
%       Y1 - (Optional) Final state vector
%            [ theta1_1; theta2_1; vtheta1_1; vtheta2_1]
%       nsteps - (Optional) Number of steps in final numerical solution
% 
%     Parameters:
%       l1 - Length of link 1
%       l2 - Length of link 2
%       m1 - Mass of link 1
%       m2 - Mass of link 2
%       g  - Value of gravity

	nvarargs = length(varargin);
    if nvarargs == 0
        Y0 = [pi/2; -pi; 0; 0];
        Y1 = [0; pi/2; 0; 0];
        nsteps = Tend*100;
    elseif nvarargs == 2
        Y0 = varargin{1};
        Y1 = varargin{2};
        nsteps = Tend*100;
    elseif nvarargs == 3
        Y0 = varargin{1};
        Y1 = varargin{2};
        nsteps = varargin{3};
    else
        error('Unexpected number of arguments')
    end
    
    X = fsolve(@(X) objective_fun( X, Tend, Y0, Y1, param ), X0);
    [T, Y] = ode45(@(T, Y) optimal_control_two_link_robot_vec_field(T, Y, param), linspace(0, Tend, nsteps+1), [Y0; X]);
end

function [ F ] = objective_fun( X0, Tend, Y0, Y1, param )
%OBJECTIVE_FUN Auxiliary function that evaluates the function for the
%boundary value problem.
    [~, Y] = ode45(@(T, Y) optimal_control_two_link_robot_vec_field(T, Y, param), [0, Tend], [Y0; X0]);
    F = acos(cos(Y(end,1:4).'- Y1));
end