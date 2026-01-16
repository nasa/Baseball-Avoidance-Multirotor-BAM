function [data] = convertArrayToStruct(matFile, busName)
    arguments
      matFile char;
      busName (1, :) char =  'BUS_USER_SIM_OUT'
    end

    if ~exist(matFile,'file')
        error(sprintf('matFile %s does not exist', matFile));
    end

    matData = load(matFile);

    % convert the loaded sim output from a vector into a structure of
    % timeseries objects based on the BUS_USER_SIM_OUT definition
    initialOffset = 0;
    [data, ~] = createTimeseries(busName, initialOffset,...
                                 matData.rt_yout.signals(1),...
                                 matData.rt_yout.time);

end
