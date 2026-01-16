function SimIn = setupRefInputs(SimIn,userStruct)
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            SimIn = setupTrajBez(SimIn,userStruct);
        case  RefInputEnum.TIMESERIES
        otherwise
            error('Not a valid input or no setup defined.');
    end

end