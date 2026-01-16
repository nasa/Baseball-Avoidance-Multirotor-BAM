%% Check if there is a userStruct input
% get userStruct state
bUserExist = exist('userStruct','var');
%% Check top level fields
% variants and types
if bUserExist
  bTypesExist = isfield(userStruct, 'types');
else
  bTypesExist = false;
end
if bUserExist
  bVariantExist = isfield(userStruct, 'variants');
else
  bVariantExist = false;
end

% modify userStruct top level
if ~bUserExist
    userStruct = struct();
end
if ~bVariantExist
    userStruct.variants = [];
end
if ~bTypesExist
    userStruct.types = [];
end

% setup Computation Struct
compStruct = [];
% documentation and cleanup
userStruct.info.bVariantExist = bVariantExist;
userStruct.info.bSwitchExist = bTypesExist;
userStruct.info.bUserExist = bUserExist;
clearvars('bSwitchExist','bVariantExist','bUserExist','bTypesExist')
