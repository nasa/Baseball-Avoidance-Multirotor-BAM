function grt_make_rtw_hook(hookMethod,modelName,rtwroot, ...
                           templateMakefile,buildOpts,buildArgs,buildInfo)
% function grt_make_rtw_hook(hookMethod,modelName,rtwroot, ...
%                            templateMakefile,buildOpts,buildArgs,buildInfo)
%
% Allows customization of the code generation process.
%
% AUTHORS:
%   Thomas C. Britton
%     Science and Technology Corporation (STC), RSES Contract,
%     NASA Langley Research Center
% 
% Revisions:
%   tbritton 20250605 - Initial version

switch hookMethod
  case 'entry'
    rtw.targetNeedsCodeGen('set', true);

%  case 'before_tlc'

  case 'after_tlc'
   % Called just after to invoking TLC Compiler (actual code generation.)
   % Valid arguments at this stage are hookMethod, Name, and
   % buildArgs, buildInfo
   %
   setTargetProvidesMain(buildInfo, true);

%  case 'before_make'

%  case 'after_make'

%  case 'exit'

%  case 'error'

end

end

