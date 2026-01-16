function SimIn = setupFCS(SimIn,userStruct)
    % Load controller parameters
    in_struct.MP = SimIn.vehicle.mass;
    in_struct.prop = SimIn.vehicle.propellers;
    in_struct.Units = SimIn.Units;
    FCS = initFCS(in_struct); % initFCS(SimIn,Setup.type)
    SimIn.FCS = FCS;
end
