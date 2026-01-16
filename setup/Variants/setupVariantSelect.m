function SimIn = setupVariantSelect(SimIn,userStruct)
% setup common variant conditions


m = enumeration('AtmosphereEnum');
SimIn.variants.atmosType = m(userStruct.variants.atmosType);
m = enumeration('DataOutEnum');
SimIn.variants.dataOutType = m(userStruct.variants.dataOutType); % select data output model
m = enumeration('RefInputEnum');
SimIn.variants.refInputType = m(userStruct.variants.refInputType);
m=enumeration('Bball_RefInputEnum');
SimIn.variants.refInputTypeBball = m(userStruct.variants.refInputTypeBball);
m = enumeration('EOMEnum');
SimIn.variants.eomType = m(userStruct.variants.eomType);
m = enumeration('PubEnum'); % Set up own-ship publishing node
SimIn.variants.pubType = m(userStruct.variants.pubType);
m = enumeration('PubEnum'); % Set up Bball publishing node
SimIn.variants.pubTypeBball = m(userStruct.variants.pubTypeBball);
m = enumeration('SubEnum'); % Set up ROS2 subscribing node
SimIn.variants.subType = m(userStruct.variants.subType);
m = enumeration('PaceEnum'); % Set up Real Time Pacing s-function
SimIn.variants.rt_pacing = m(userStruct.variants.rt_pacing);
end
