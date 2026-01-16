function [out_struct,c_struct] = defaultBezBball()

initial_time = 0;
% wptsX = [0 3 0; 
%     120 3 0]; % within row = pos vel acc, rows are waypoints
wptsX = [60+3*sqrt(3)*20/2 -3*sqrt(3)/2 0; 60-3*sqrt(3)*20/2 -3*sqrt(3)/2 0];
time_wptsX = [initial_time 40];
% wptsY = [0 0 0; 
%     0 0 0]; % within row = pos vel acc, rows are waypoints
wptsY = [+3*sqrt(3)*20/2 -3*sqrt(3)/2 0; -3*sqrt(3)*20/2 -3*sqrt(3)/2 0];
time_wptsY = [initial_time 40];
wptsZ = [0 0 0 ; 
    -60 -3 0 ]; % within row = pos vel acc, rows are waypoints, NOTE: NED frame -z is up...
time_wptsZ = [initial_time 40];

out_struct.waypointsX = wptsX;
out_struct.waypointsY = wptsY;
out_struct.waypointsZ = wptsZ;

out_struct.time_wptsX = time_wptsX;
out_struct.time_wptsY = time_wptsY;
out_struct.time_wptsZ = time_wptsZ;


[pos_i, vel_i, acc_i, chi, chi_d, ~] =... 
    evalSegments(wptsX,...
    wptsY,...
    wptsZ,...
    time_wptsX,...
    time_wptsY,...
    time_wptsZ, initial_time);
c_struct.initial_output.pos_i = pos_i;
c_struct.initial_output.vel_i = vel_i;
c_struct.initial_output.acc_i = acc_i;
c_struct.initial_output.chi = chi;
c_struct.initial_output.chi_d = chi_d;
end


