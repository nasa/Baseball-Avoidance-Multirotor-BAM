function SimIn = setupTrajBez(SimIn,userStruct)

    temp = userStruct.simulation_defaults.RefInputs;
    
    [wptVect, timeVect, parseVect] = ...
        encodeBezierArray({temp.waypointsX, temp.waypointsY,temp.waypointsZ}, ...
        {temp.time_wptsX, temp.time_wptsY, temp.time_wptsZ}); 

    SimIn.RefInputs.trajectory.wptVect      = wptVect;
    SimIn.RefInputs.trajectory.timeVect     = timeVect;
    SimIn.RefInputs.trajectory.parseVect    = parseVect;
end


