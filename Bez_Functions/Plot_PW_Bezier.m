function Plot_PW_Bezier(waypoints,time_wpts, varargin)
% Plot_PW_Bezier.m plots the ownship top level trajectory information.
% This includes 3D position trajectory curve and associated PW Bernstein
% polynomial velocity and acceleration profiles.  This file is used to
% visual a given PW Bernstein polynomial trajectory.

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 05.19.2025, APP: Function
% 9.29.2023, MJA: Initial version of script.  Creates and plots ownship 
% PW Bernstein polynomial trajectory information 

% *************************************************************************
% Plots current trajectory
%close all

% First varargin is a scalar (1-4) of which figures to plot
if nargin > 2
    figs_2_plot = varargin{1}; % input array of figures to plot
end

% Specify the plot symbol arguments
if nargin > 3
    plot_sym = varargin{2};
else 
    plot_sym = [];
end

% Specify what to plot: 0 = curves and times, 1 = curves only, 2 = times only
% for the first plot only
if nargin > 4
    plot_type = varargin{3};
else 
    plot_type = 0;
end

% *************************************************************************
p_int = 0.01;
t_st = time_wpts{1}(1);
t_end = time_wpts{1}(end);

if isempty(t_st)
    t1 = 0;
else 
    t1 = t_st;
end
if isempty(t_end)
    t2 =  time_wpts{1}(end);
else
    t2 = t_end;
end
time = linspace(t1,t2, t2/p_int);
pwcurve_plot = genPWCurve(waypoints,time_wpts);

% Plot positions with time
values = evalPWCurve(pwcurve_plot,time,0);
%figure
if any(figs_2_plot==1)
    figure(1);
    % if nargin > 3
    if plot_type==0 || plot_type ==1
        plot3(values(:,1), values(:,2), -values(:,3), plot_sym);
    end
    % else
    %     plot3(values(:,1), values(:,2), -values(:,3));
    % end
    title('3D Desired trajectory');
    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;

    if plot_type==0 || plot_type ==2
        values2 = evalPWCurve(pwcurve_plot,time_wpts{1} ,0);
        %plot3(values2(:,1), values2(:,2), values2(:,3),'rx');
        
        % Add trajectory time at beginning of each PW segment position
        for loop = 1:length(time_wpts{1})
            if time_wpts{1}(loop)>=t1 && time_wpts{1}(loop)<=t2
                plot3(values2(loop,1), values2(loop,2), -values2(loop,3),'rx');
                text(values2(loop,1), values2(loop,2), -values2(loop,3),sprintf('%0.1f',round(time_wpts{1}(loop))));
            end
        end
        hold on
    end
    % Add segment number at beginning of each PW segment position
    % for loop = 0:length(time_wpts{1})-1
    %     text(values2(loop+1,1), values2(loop+1,2), -values2(loop+1,3),sprintf('%0.1f',loop));
    % end
end

if any(figs_2_plot==2)
    %figure;
    figure(2);
    plot(time, values);
    title('Positions');
    xlabel('Time (sec)');
    ylabel('Postion (ft)');
    grid on;
    legend('X', 'Y', 'Z');
    hold on
end

if any(figs_2_plot==3)
    %Plot velocities with time
    values = evalPWCurve(pwcurve_plot,time,1);
    %figure;
    figure(3);
    plot(time, values)
    title('Velocities')
    xlabel('Time (sec)');
    ylabel('Velocity (ft/sec)');
    grid on;
    legend('X','Y','Z')
    disp('')
    hold on;
end

if any(figs_2_plot==4)
    %Plot accelerations with time
    values = evalPWCurve(pwcurve_plot,time,2);
    %figure;
    figure(4);
    plot(time, values)
    title('Accelerations')
    xlabel('Time (sec)');
    ylabel('Acc (ft/sec^2)');
    grid on;
    legend('X','Y','Z')
    hold on
end

end % end of function