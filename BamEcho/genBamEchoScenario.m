function genBamEchoScenario(SimInData, SimOutData, framerate, tagString, doBaseball)
arguments 
  SimInData struct {mustBeNonempty};
  SimOutData struct {mustBeNonempty};
  framerate double {mustBePositive} = 24;
  tagString {mustBeText} = '';
  doBaseball logical = 1;
end

  recordedTimeSlice = SimInData.Environment.Turbulence.dT;
  timeSlice = 1/framerate;

  % additional safety check if using baseball, make sure the time codes match so we have 1 to 1
  % Dev Note: This should pretty much never happen.  It's really a catch all for some
  %           catastrophic configuration or simulation failure on the two datasets.  -DRH
  if doBaseball
    rotorTimeInfo = SimOutData.EOM.InertialData.Pos_bii.TimeInfo;
    bbTimeInfo = SimOutData.RefInputs.pos_i.TimeInfo;;
    if not(rotorTimeInfo.Start == bbTimeInfo.Start) || not(rotorTimeInfo.End==bbTimeInfo.End)
      error("Time codes between multirotor and baseball must match");
    end
  end

  if isempty(tagString)
    % generate a default scenario tag for the filenames since this was left
    % empty in the arguments.
    timestamp = datetime('Now','Format','yyyyMMdd_hhmm');
    tagString = sprintf('BamScn_%s', timestamp);
  end
  
  % examine time meta data and make sure the times match up (they should, but this is just to
  % be extra careful)


  times = SimOutData.EOM.InertialData.Pos_bii.Time;
  timeInfo = SimOutData.EOM.InertialData.Pos_bii.TimeInfo;
  posNED = SimOutData.EOM.InertialData.Pos_bii.Data;
  roll = SimOutData.EOM.WorldRelativeData.Euler.phi.Data;
  pitch= SimOutData.EOM.WorldRelativeData.Euler.theta.Data;
  yaw  = SimOutData.EOM.WorldRelativeData.Euler.psi.Data;

  % scale down
  duration = timeInfo.End-timeInfo.Start;
  sampleTimes = linspace(timeInfo.Start, timeInfo.End, duration*framerate);
  %times  = interp1(times, times, sampleTimes);
  posNED = interp1(times, posNED, sampleTimes);
  roll   = interp1(times, roll, sampleTimes)';  % transpose
  pitch  = interp1(times, pitch, sampleTimes)'; % transpose
  yaw    = interp1(times, yaw, sampleTimes)';   % transpose

  % ownship data gen
  % TimeSec, PosNed_1, PosNed_2, PosNed_3, rolldeg, pitchdeg, yawdeg
  filename = [tagString, '_multirotor.csv'];
  varNames = {'timeSec', 'posNED','rollRad', 'pitchRad', 'yawRad'};
  tableOut = table(sampleTimes', posNED, roll, pitch, yaw, 'VariableNames',varNames);
  writetable(tableOut, filename);
  clear tableOut;

  % I could probably make this more elegant and put it in a loop, but I'm going for quick and
  % honestly we aren't expecting this to augment beyond 2 things right now. -DRH 
  %      PS - Famous last words.  I'll see you back here, Future Dan.
  if doBaseball

    times = SimOutData.Bball_Traj_Iner.pos_i.Time;
    timeInfo = SimOutData.Bball_Traj_Iner.pos_i.TimeInfo;
    posNED = SimOutData.Bball_Traj_Iner.pos_i.Data;
    track  = SimOutData.Bball_Traj_Iner.chi_des.Data;

    duration = timeInfo.End-timeInfo.Start;
    sampleTimes = linspace(timeInfo.Start, timeInfo.End, duration*framerate);
    posNED = interp1(times, posNED, sampleTimes);
    track   = interp1(times, track, sampleTimes)';
  
    % ownship data gen
    filename = [tagString, '_baseball.csv'];
    varNames = {'timeSec', 'posNED','trackRad'};
    tableOut = table(sampleTimes', posNED, track, 'VariableNames',varNames);
    writetable(tableOut, filename);
    clear tableOut;
  end
end

%========================================================================
% BAM Note: I borrowed this from SPT's Util class because I knew it could
%           be useful here, but SPT itself isn't packaged with BAM
%========================================================================
% helper function for determining a subset (row-wise)of data based on a certain step size between
% data.  It defaults to guaranteeing that the end data point will be part of the data, but
% this can be disabled.
% Dev note: This operates row-wise; aka treats rows for indices of records.  Column wise won't work.] 
% Dev Note: I threw this together because I regularly was needing to figure how to downsize
%           or rather how big something would be if I downsized a set of data for SPT runs.
%           But I noticed the default logic would drop the last value at times, and this
%           was unwanted.  Since the could and checks became bulky being used in mulitple
%           places, i just figured to centralized it and place several outputs depending on
%           which I needed to use at the time (sometimes just the rec count)
%
%========================================================================
function [dataOut, idxOut, recCount] = getSubset(dataIn, stepSize, forceEnd)
arguments
  dataIn;
  stepSize {mustBeGreaterThanOrEqual(stepSize, 1)};
  forceEnd logical = true;
end
  num = size(dataIn,1);
  indices = 1:stepSize:num;

  % ensure the final value is part of the set regardless of step size
  if forceEnd && (indices(end) ~= num)
    indices(end+1) = num;
  end

  dataOut = dataIn(indices,:);
  idxOut = indices;
  recCount = length(idxOut);
end