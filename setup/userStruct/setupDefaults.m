%% Default model_params
if ~isfield(userStruct,'model_params')
    userStruct.model_params = [];
end
if ~(isfield(userStruct.model_params,'stop_time'))
    userStruct.model_params.stop_time = 40;
end
if ~(isfield(userStruct.model_params,'sim_rate'))
    userStruct.model_params.sim_rate = 1/200;
end
if ~(isfield(userStruct.model_params,'rt_pace'))
    % This parameters set the autocode wall clock pacing if the
    % Autocode_RT_Pacing variant subsytem is selected via
    % userStruct.variants.rt_pacing = PaceEnum.RT_PACE;
    userStruct.model_params.rt_pace = 1; % This parameters set the autocode wall clock pacing 
end

%% Default trajectory
if ~isfield(userStruct,'simulation_defaults')
    userStruct.simulation_defaults = [];
end
if ~isfield(userStruct.simulation_defaults,'RefInputs')
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            [out1,out2] = defaultBez();  
            userStruct.simulation_defaults.RefInputs = out1;
            compStruct.simulation_defaults.RefInputs = out2;
            clear out1 out2;

        otherwise
            error('Not a valid input or no setup defined.');
    end
end

%% Baseball Default trajectory
if ~isfield(userStruct,'simulation_defaults')
    userStruct.simulation_defaults = [];
end
if ~isfield(userStruct.simulation_defaults,'RefInputsBball')
    switch userStruct.variants.refInputTypeBball
        case Bball_RefInputEnum.NONE_BBALL
            disp('')
        case Bball_RefInputEnum.BEZIER_BBALL
            [out1,out2] = defaultBezBball();  
            userStruct.simulation_defaults.RefInputsBball = out1;
            compStruct.simulation_defaults.RefInputsBball = out2;
        otherwise
            error('Not a valid input or no setup defined.');
    end
end
%% Default trim
if ~isfield(userStruct.simulation_defaults,'trim')
    userStruct.simulation_defaults.trim = [];
end
if ~(isfield(userStruct.simulation_defaults.trim,'pos_i'))
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            wptsX = userStruct.simulation_defaults.RefInputs.waypointsX;
            wptsY = userStruct.simulation_defaults.RefInputs.waypointsY;
            wptsZ = userStruct.simulation_defaults.RefInputs.waypointsZ;
            time_wptsX = userStruct.simulation_defaults.RefInputs.time_wptsX;
            time_wptsY = userStruct.simulation_defaults.RefInputs.time_wptsY;
            time_wptsZ = userStruct.simulation_defaults.RefInputs.time_wptsZ;            
            initial_time = time_wptsX(1);
            [pos_i, vel_i, acc_i, chi, chi_d, ~] =... 
            evalSegments(wptsX,...
            wptsY,...
            wptsZ,...
            time_wptsX,...
            time_wptsY,...
            time_wptsZ, initial_time);
            userStruct.simulation_defaults.trim.pos_i = ...
                pos_i;
        otherwise
            userStruct.simulation_defaults.trim.pos_i = zeros(3,1);
    end
    
end
if ~(isfield(userStruct.simulation_defaults.trim,'vel_bIi'))
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            userStruct.simulation_defaults.trim.vel_bIi = vel_i;
        otherwise
            userStruct.simulation_defaults.trim.vel_bIi = zeros(3,1);
    end
    
end
if ~(isfield(userStruct.simulation_defaults.trim,'acc_i'))
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            userStruct.simulation_defaults.trim.acc_i = acc_i;
        otherwise
            userStruct.simulation_defaults.trim.acc_i = zeros(3,1);
    end
    
end
if ~(isfield(userStruct.simulation_defaults.trim,'psi'))
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            userStruct.simulation_defaults.trim.psi = chi;
        otherwise
            userStruct.simulation_defaults.trim.psi = 0;
    end
    
end
if ~(isfield(userStruct.simulation_defaults.trim,'psi_d'))
    switch userStruct.variants.refInputType
        case  RefInputEnum.BEZIER
            userStruct.simulation_defaults.trim.psi_d = chi_d;
        otherwise
            userStruct.simulation_defaults.trim.chi_d = 0;
    end
    
end

%% Default winds
if ~isfield(userStruct.simulation_defaults,'Environment')
    userStruct.simulation_defaults.Environment = [];
end
if ~isfield(userStruct.simulation_defaults.Environment,'Turbulence')
    userStruct.simulation_defaults.Environment.Turbulence = [];
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'RandomSeedLong'))
    userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLong=3366;
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'RandomSeedLat'))
    userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLat=23;
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'RandomSeedVert'))
    userStruct.simulation_defaults.Environment.Turbulence.RandomSeedVert=1369;
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'intensity'))
    userStruct.simulation_defaults.Environment.Turbulence.intensity=1;
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'WindAt5kft'))
    userStruct.simulation_defaults.Environment.Turbulence.WindAt5kft=0;
end
if ~(isfield(userStruct.simulation_defaults.Environment.Turbulence,'WindDirectionAt5kft'))
    userStruct.simulation_defaults.Environment.Turbulence.WindDirectionAt5kft=60;
end
