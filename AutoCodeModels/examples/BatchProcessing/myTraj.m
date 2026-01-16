function [wptVect, timeVect, parseVect] = myTraj(randVector)
%   MYTRAJ [wptVect, timeVect, parseVect] = myTraj(randVector)
%   randVector = [x, y, z, xd, yd, zd] of dimension [1,6]
    initial_time = 0;
    wptsX = [0 3 0; 
        120+randVector(1) 3+randVector(4) 0];
    time_wptsX = [initial_time 40];
    wptsY = [0 0 0; 
        0+randVector(2) 0+randVector(5) 0];
    time_wptsY = [initial_time 40];
    wptsZ = [0 0 0 ; 
        -60+randVector(3) -3+randVector(6) 0 ];
    time_wptsZ = [initial_time 40];
    [wptVect, timeVect, parseVect] = encodeBezierArray({wptsX wptsY wptsZ}, {time_wptsX time_wptsY time_wptsZ});
end
