function [] = plot_two_link_robot( T, Y, param, varargin )
%PLOT_TWO_LINK_ROBOT plots a simple model of the planar two-link robot.
%
%     It is assumed that the center of mass of each link is at its
%     geometric center. It is also assumed that the principal moment of
%     inertia about its local z axis (perpendicular to the plane of motion)
%     is m/12*l^2
% 
%     Arguments:
%       T - Time vector
%       Y - State matrix [theta1(:), theta2(:), vtheta1(:), vtheta2(:)]
%       param - Parameters of the model [l1, l2, m1, m2, g] (vector)
% 
%     Parameters:
%       l1 - Length of link 1
%       l2 - Length of link 2
%       m1 - Mass of link 1
%       m2 - Mass of link 2
%       g  - Value of gravity

    nsteps = length(T)-1;
	nvarargs = length(varargin);
    bool_write = false;
    if nvarargs == 1
        pause_time = varargin{1};
    elseif nvarargs == 2
        bool_write = true;
        pause_time = varargin{1};
        filename = varargin{2};
    else
        error('Unexpected number of arguments')
    end

    l1 = param(3);
    l2 = param(4);

    for i = 1:nsteps+1
        if i == 1
            p_rod1 = plot([0, l1*cos(Y(i,1))],[0, l1*sin(Y(i,1))],'g', 'LineWidth',1);
            hold on
            p_rod2 = plot([l1*cos(Y(i,1)), l1*cos(Y(i,1))+l2*cos(Y(i,1)+Y(i,2))],[l1*sin(Y(i,1)),l1*sin(Y(i,1))+l2*sin(Y(i,1)+Y(i,2))],'g', 'LineWidth',1);
            hold off
            axis(1.25*[-(l1+l2),(l1+l2),-(l1+l2),(l1+l2)])
            daspect([1,1,1])
            if bool_write
                frame = getframe(F);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                %Write to the GIF File
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0);
            end
        else
            set(p_rod1,'XData',[0, l1*cos(Y(i,1))],'YData',[0, l1*sin(Y(i,1))])
            set(p_rod2,'XData',[l1*cos(Y(i,1)), l1*cos(Y(i,1))+l2*cos(Y(i,1)+Y(i,2))],'YData',[l1*sin(Y(i,1)),l1*sin(Y(i,1))+l2*sin(Y(i,1)+Y(i,2))])
            drawnow
            if bool_write
                frame = getframe(F);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0);
            end
        end
        pause(pause_time)
    end
end

