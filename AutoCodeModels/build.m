function []=build(buildDiagram, modelFile, exeFlag, targetLangC)
% function [] = build(buildDiagram, modelFile, exeFlag, targetLangC)
%
% Build function used to perform code-generation on the specified model
% file.  Based on functionality originally developed for the Generalized
% Vehicle Simulation (GVS).
% 
% Inputs:
%   buildDiagram - character array that the specified model file is copied
%                  to, on which code generation is performed
%   modelFile - character array containing the name of the model file
%   exeFlag - boolean flag controlling whether an executable or static
%             library is built
%   targetLangC - boolean flag controlling whether code is generated as C
%                 or C++
%
% AUTHORS:
%   Thomas C. Britton
%     Science and Technology Corporation (STC), RSES Contract,
%     NASA Langley Research Center
% 
% Revisions:
%   tbritton 20250320 - Initial version
%   tbritton 20250606 - Modified to remove calls to parse functions due to
%                       implementation of customized build hook function
%   TNZ      20250702 - Disable SIMD for Apple Silicon
arguments
  buildDiagram char;
  modelFile char;
  exeFlag logical = false;
  targetLangC logical = true;
end

if ~exist(modelFile,'file'), error('Must supply model filename'); end

SimIn = evalin('base','SimIn');

% set data output type for code generation
SimIn.dataOutType = DataOutEnum.CODEGEN;

rootDir = getenv("ActiveRootDir");
buildFolder = sprintf('%s/AutoCode/%s', rootDir, buildDiagram);
rtw_buildfolder = sprintf('%s/%s_grt_rtw',buildFolder,buildDiagram);
slprj_folder = sprintf('%s/slprj',buildFolder);

Simulink.fileGenControl('set',...
    'CacheFolder', buildFolder,...
    'CodeGenFolder', buildFolder,...
    'createDir', true)

cfg = Simulink.fileGenControl('getConfig')

% Remove old build directories if they exist
if exist(rtw_buildfolder,'dir'), rmdir(rtw_buildfolder,'s'); end
if exist(slprj_folder,'dir'), rmdir(slprj_folder,'s'); end

% Clear existing system, terminate if unable to close
try
  bdclose(buildDiagram);
catch
  eval(sprintf('%s([],[],[],''term'');',buildDiagram));
  bdclose(buildDiagram);
end

% Load system into backgroud, model name must be consistent with calling routines
%load_system(buildDiagram);

% Load system into background, with normalized modelname
new_system(buildDiagram,'FromFile',modelFile);

% cs = getActiveConfigSet('SimParModel');
% get_param(cs,'ObjectParameters');

% Set important code-generation options

%
% Code Generation
%
% Target selection
set_param(buildDiagram,'SystemTargetFile','grt.tlc');
if (targetLangC == true)
  set_param(buildDiagram,'TargetLang','C');
else
  set_param(buildDiagram,'TargetLang','C++');
end

% Build process
set_param(buildDiagram,'GenCodeOnly','on'); % generate code only to allow modification of makefiles, etc

% set the toolchain value to 'automatically...' in the model, then it is based
% on the selected compiler
toolchainStr = 'Automatically locate an installed toolchain';
set_param(buildDiagram, 'Toolchain', toolchainStr);   % Toolchain

set_param(buildDiagram, 'PackageGeneratedCodeAndArtifacts', 'on');

% build configuration 
set_param(buildDiagram,'BuildConfiguration','Faster Runs');

% Optimization
set_param(buildDiagram,'OptimizeBlockIOStorage','off');
set_param(buildDiagram,'ExpressionFolding','off');
set_param(buildDiagram,'LocalBlockOutputs','off');
set_param(buildDiagram,'DefaultParameterBehavior','Tunable');

% Disable single instruction, multiple data (SIMD) flag which fails for Apple
% Silicon (vectorization is done with a different process). Unfortunately,
% simulink coder does not automatically deal with this
if ismac
    set_param(buildDiagram,'InstructionSetExtensions','None');
end

% Report
set_param(buildDiagram,'GenerateReport','on');

% Code interface
set_param(buildDiagram,'CodeInterfacePackaging','Reusable'); %
%set_param(buildDiagram,'CodeInterfacePackaging','C++ class');

if (targetLangC == true)
  fileExt = '.c';
else
  fileExt = '.cpp';
end
customMain = sprintf('%s_rt_malloc_main%s', buildDiagram, fileExt);

if (exeFlag == true)
  set_param(buildDiagram,'MatFileLogging','on');

  % specify any user supplied or 3rd party lib files to be used
  % OS-specific static lib extension
  if ispc
    libext = 'lib';
    osDir = 'win64';
    directives = '';
  elseif ismac
    libext = 'a';
    osDir = 'mac';
    directives = '';
  elseif isunix
    libext = 'a';
    osDir = 'linux';
    directives = '';
  end

  libfiles = '';
    
%  set_param(buildDiagram,'CustomSourceCode', '');
  set_param(buildDiagram,'CustomHeaderCode', '#include "readPfile.h"');
  set_param(buildDiagram,'CustomInitializer', 'readPFile("pFile.bin");');
%  set_param(buildDiagram,'CustomTerminator', '');

  includedirs = '';

  % create folder for custom source files, not revision controlled
  customSrcDir = 'AutoCodeModels/custom_src';
  if exist(customSrcDir,'dir') == 0
    mkdir('./AutoCodeModels', 'custom_src');
  end

  % copy custom main to one reflecting the build name in the custom src folder
  copyfile(['AutoCodeModels/src/rt_malloc_main' fileExt], ...
           [customSrcDir filesep customMain], 'f');

  % copy readPfile to custom src folder
  pfilename = sprintf('readPfile%s', fileExt);
  copyfile('AutoCodeModels/src/readPfile.c', ...
           [customSrcDir filesep pfilename], 'f')
  copyfile('AutoCodeModels/src/readPfile.h', customSrcDir, 'f')

  % set custom src files
  srcfiles = sprintf('%s%s%s\n%s%s%s\n', customSrcDir, filesep, pfilename, customSrcDir, filesep, customMain);

  set_param(buildDiagram,'CustomInclude', includedirs);
  set_param(buildDiagram,'CustomSource', srcfiles);
  set_param(buildDiagram,'CustomLibrary', libfiles);
  set_param(buildDiagram,'CustomDefine', directives);

else
  set_param(buildDiagram,'MatFileLogging','off');

  includedirs = '';
  set_param(buildDiagram,'CustomInclude', includedirs);
end

%
% Solver
%
% Simulation timee
%set_param(buildDiagram,'StartTime','0.0');
%set_param(buildDiagram,'StopTime', num2str(stopTime));

% Solver selection
set_param(buildDiagram,'SolverType','Fixed-step');
%set_param(buildDiagram,'Solver','ode3');
set_param(buildDiagram,'Solver','FixedStepAuto');
% Solver details
%set_param(buildDiagram,'FixedStep','1/200'); % for BAM, controlled via a SimPar value

set_param(buildDiagram,'EnableMultiTasking','off');

set_param(buildDiagram,'ParameterTunabilityLossMsg','warning');

% Data Import/Export
% Save to workspace or file
if (exeFlag == true)
  set_param(buildDiagram,'SaveFormat','StructureWithTime');
  set_param(buildDiagram,'SaveOutput','on');
  set_param(buildDiagram,'Decimation','1'); % @@TCB: variable not currently defined in SimIn
%  set_param(buildDiagram,'Decimation',int2str(SimIn.decimation));
end

% Generate source code
if (exeFlag == true)
  slbuild(buildDiagram, 'BuildOutputType' ,'Executable');
else
  slbuild(buildDiagram, 'BuildOutputType' ,'StaticLibrary');
end

% build command
if ispc
  osBuildCmd = sprintf('%s.bat', buildDiagram);
elseif isunix
  osBuildCmd = sprintf('make -f %s.mk', buildDiagram);
end
buildCmd = sprintf('cd %s/AutoCode/%s/%s_grt_rtw/ && %s && cd %s', ...
                   rootDir, buildDiagram, buildDiagram, osBuildCmd, rootDir);
system(buildCmd);

% Copy report folder to top level model build folder
copyfile(sprintf('%s/html',rtw_buildfolder), ...
         sprintf('%s/report_html',buildFolder), 'f');

% Pack into flat file
load(sprintf('%s/buildInfo',rtw_buildfolder));
packNGo(buildInfo,...
    'packType','flat',...
    'FileName',buildDiagram,...
    'includeReport',false);

zipFile = sprintf('%s/%s',buildFolder,buildDiagram);

% Remove build directories if they exist
%if exist(rtw_buildfolder,'dir'), rmdir(rtw_buildfolder,'s'); end
if exist(slprj_folder,'dir'), rmdir(slprj_folder,'s'); end
buildInfo_file = sprintf('%s/buildInfo.mat',buildFolder);
if exist(buildInfo_file,'file'), delete(buildInfo_file); end

% Close hidden system
bdclose(buildDiagram);

Simulink.fileGenControl('reset');
