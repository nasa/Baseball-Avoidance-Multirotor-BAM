function SimIn = setupVehicle(SimIn,userStruct)
    Q = IMPACT_props2(SimIn);
    
    % Below is no longer needed as used with OG aero-propulsive model
    SimIn.vehicle.constants = Q.c;
    SimIn.vehicle.aero = Q.aero;
    SimIn.vehicle.mass = Q.MP;
    SimIn.vehicle.propellers = Q.prop;
end