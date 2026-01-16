% Atmosphere Model
if ~isfield(userStruct.variants,'atmosType')
    userStruct.variants.atmosType = AtmosphereEnum.US_STD_ATMOS_76;
end

% % Turbulence Model 
% Note there is only one environmental model: US_STD_ATMOS_76, and the
% turbulence model is always "on" although the winds and turbulence
% default values are set to zeros.   So the turbType  and enumeration below is not used.
% Users who want to change the winds/turbulence values should use the
% userStruct.simulation_defaults.Environment.Turbulence fields.
% if ~isfield(userStruct.variants,'turbType')
%     userStruct.variants.turbType = TurbulenceEnum.None;
% end

if ~isfield(userStruct.variants,'dataOutType')
  userStruct.variants.dataOutType = DataOutEnum.NORMAL;
end

% Reference Type
if ~isfield(userStruct.variants,'refInputType')
    userStruct.variants.refInputType = RefInputEnum.BEZIER;
end

% Baseball Reference Type
if ~isfield(userStruct.variants,'refInputTypeBball')
    userStruct.variants.refInputTypeBball = Bball_RefInputEnum.NONE_BBALL;
end

% EOM Model
if ~isfield(userStruct.variants,'eomType')
    userStruct.variants.eomType = EOMEnum.STARS;
end

% ROS2 Publishing Model
if ~isfield(userStruct.variants,'pubType')
    userStruct.variants.pubType = PubEnum.NONE;
end

% Bball ROS2 Publishing Model
if ~isfield(userStruct.variants,'pubTypeBball')
    userStruct.variants.pubTypeBball = PubEnum.NONE;
end

% ROS2 Subscribing Model
if ~isfield(userStruct.variants,'subType')
    userStruct.variants.subType = SubEnum.NONE_SUB;
end

% Autocode Realtime Pacing
if ~isfield(userStruct.variants,'rt_pacing')
    userStruct.variants.rt_pacing = PaceEnum.RT_NONE;
end
