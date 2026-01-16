function SimIn = setupTrajBezBball(SimIn,userStruct)

    temp = userStruct.simulation_defaults.RefInputsBball;
    
    [wptVect, timeVect, parseVect] = ...
        encodeBezierArray({temp.waypointsX, temp.waypointsY,temp.waypointsZ}, ...
        {temp.time_wptsX, temp.time_wptsY, temp.time_wptsZ}); 

    SimIn.RefInputs.trajectoryBball.wptVect      = wptVect;
    SimIn.RefInputs.trajectoryBball.timeVect     = timeVect;
    SimIn.RefInputs.trajectoryBball.parseVect    = parseVect;
end


