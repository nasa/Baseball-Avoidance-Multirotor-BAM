function [wptVect, timeVect, parseVect] = zerosTraj()

    initial_time = 0;
    wptsX = [0 0 0; 
            0 0 0];
    time_wptsX = [initial_time 40];
    wptsY = [0 0 0; 
            0 0 0];
    time_wptsY = [initial_time 40];
    wptsZ = [0 0 0 ; 
            0 0 0];
    time_wptsZ = [initial_time 40];
    [wptVect, timeVect, parseVect] = encodeBezierArray({wptsX wptsY wptsZ}, {time_wptsX time_wptsY time_wptsZ});
end
