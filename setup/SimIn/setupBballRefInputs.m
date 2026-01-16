function SimIn = setupBballRefInputs(SimIn,userStruct)
    switch userStruct.variants.refInputTypeBball
        case  Bball_RefInputEnum.BEZIER_BBALL
            SimIn = setupTrajBezBball(SimIn,userStruct);
        case  Bball_RefInputEnum.NONE_BBALL
            disp('')
        otherwise
            error('Not a valid input or no setup defined.');
    end

end