function out_struct = setupBernstein()

wptsX = [0 3 0; 120 3 0]; % within row = pos vel acc, rows are waypoints
time_wptsX = [0 40];
wptsY = [0 0 0; 0 0 0]; % within row = pos vel acc, rows are waypoints
time_wptsY = [0 40];
wptsZ = [0 0 0 ; -60 -3 0 ]; % within row = pos vel acc, rows are waypoints, NOTE: NED frame -z is up...
time_wptsZ = [0 40];

out_struct.RefInputs.Bezier.waypointsX = wptsX;
out_struct.RefInputs.Bezier.waypointsY = wptsY;
out_struct.RefInputs.Bezier.waypointsZ = wptsZ;

out_struct.RefInputs.Bezier.time_wptsX = time_wptsX;
out_struct.RefInputs.Bezier.time_wptsY = time_wptsY;
out_struct.RefInputs.Bezier.time_wptsZ = time_wptsZ;

% Vectorize the Bernstein polynomial desired trajectory
[wptVect, timeVect, parseVect] = encodeBezierArray({wptsX, wptsY,wptsZ}, ...
    {time_wptsX, time_wptsY, time_wptsZ}); 
out_struct.RefInputs.trajectory.wptVect      = wptVect;
out_struct.RefInputs.trajectory.timeVect     = timeVect;
out_struct.RefInputs.trajectory.parseVect    = parseVect;