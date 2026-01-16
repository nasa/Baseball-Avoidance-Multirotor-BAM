function [SimIn, SimPar] = setupParams(SimIn,userStruct)
    SimIn.model_params.stop_time = userStruct.model_params.stop_time;
    SimPar = setupParameters(SimIn);
end
