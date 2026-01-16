#==================================================================================================
# BamEcho
# A Visualization tool for BAM trajectory data to Airsim Unreal Environments.
#
# Description: This tool is designed to automate the process of taking a pregenerated BAM trajectory
#              data file, transmit the trajectories contained (one for the ownship drone and one for
#              the baseball), and then generate a video from the scene created.  Default values and 
#              camera configuration are done via the BamEcho.json file packaged with this tool.
#
# Usage: python BamEcho.py <DroneTrajectory> <BaseballTrajectory> [options]
#
# Options: '-s' - Silent Mode.  Disables almost all text output.
#          '-v' - Verbose Mode.  The most detailed console output available.
#          '-i' - UI input mode.  !!!!!!Currently not implemented!!!!!!!
#          '-t' - Tag name.  Allows user to set a custom string tag for the generated output.  Requires
#                 the user to input a string after the option flag on the command line.
#                 Usage: "python BamEcho.py <file1> <file2> -t <StringTag>"
#         '-dt' - Debug Trajectory mode.  This allows testing of the trajectory visualization while
#                 disabling the time intensive image collection and video export routines.'
#       '-noBB' - Disables all activities with the baseball actor.
#         '-rm' - Enable directory cleanup at the end; removes all the generated frame images, leaving
#                 just the the videos. 
#
# Performance Note: This tool's speed is heavily dependant on two connected items: the size of the camera
#           resolution that is being captured (defaults are Front Scene and Front Depth), and the number
#           of cameras being captured.  The biggest hog call is 'Client.simGetImages()', and unfortunately
#           it can't be spun off in a thread because it would create a race condition between what image
#           is being captured and the scene being actively updated by the main transmission logic.
#           Tips to speed up performance:
#           - Reduce resolution of target cameras
#           - Reduce the number of camera (hardcode changes or dynamic camera system(future release))
#           - Headless mode on the environment (I haven't tested this, but Airsim claims it works in Linux)
#
# Dev Note: This script is critically dependent on several compatibility assumptions with BAM and the 
#           chosen Unreal Airsim environment.  A lack of these items could result Assumptions:
#           - Bam Trajectories are given in CSV files with established format.
#           - Unreal Airsim environment must be running.
#           - Unreal environment has an asset that is spawnable named 'baseball_3d_model'
#           - Unreal environment was configured with a JSON file that:
#             -- Sets the camera resolution (like using CameraDefaultSettings)
#             -- Instantiates at least one Airsim Multirotor vehicle named 'Drone'
#             -- Disables the Airsim physics by setting: "PhysicsEngineName": "ExternalPhysicsEngine" 
#
#
# Author: Dan Hill
# Version History:
#   0.9 - Initial Beta creation.  Handles basic CSV parsing, transmission of Drone and baseball
#         positions to Airsim, screen grabs of Scene and Depth cameras onboard the Drone vehicle,
#         and exporting video to AVI via ffmpeg
#   0.95- Updated the file format reads for the formal ownship trajectory updates for BAM.
#         The baseball format file is still pending, and will need updates for that.  Added option
#         to mute baseball operations. Added Threaded image recording for a significant speed boost, 
#         but the actualy capture command still holds things up a bit.  Added option to automatically
#         delete generated Frames after videos have been created (disk saver)
#   0.99- Added JSON configuration file and dynamic camera view system. Allows the user more up front
#         visibility to the options available in Echo, and a way to setup the desired camera views that
#         will be exported to video.  Defaults are set as the current Dev preferences.  The camera views
#         beyond Scene and Depth are nominall available, but have not been tested.  There may need to be 
#         additional handling and processing logic implemented to handle the more complex views Airsim 
#         provides (Done as needs arise.).
#   1.00- Scenario Data validation. Tied to baseball data format from simulation.
#==================================================================================================

import setup_path          # critical to be first for all Airsim Python API related scripting.
import airsim              # Airsim interface calls
from enum import Enum
from math import pi
from sys import argv
import msgpackrpc
import numpy as np         # image data processing
import bamCsvLoad          # generic CSV loading
import time                # for timestamping and sleeping
import datetime            # time stamping
import sys                 # command line times primarily
import os                  # directory processing
import ffmpeg              # turn generated image files into a video in the end.
import atexit              # emergency shutdown cleanup
import shutil              # for image frame directory cleanup
import threading           # speed up image capture process
import json                # config file (particularly dynamic camera system)                 


#==================================================================================================
# TYPE DEFS 
#==================================================================================================

class Options():
    def __init__(self):
        self.logLevel = 1;  # 0 - silent, 1 - normal, 2 - verbose
        self.interactiveMode = False;
        self.doImageCapture = True;
        self.doVideoExport = True;
        self.skipBaseball = False;
        self.cleanupFrames = True;
        self.compressFrames = True;

#==================================================================================================
# Globals
# Dev note: Generally not a globals guy, but the alternatives here apply needless overhead. So I'm 
#           avoiding it in a few small cases like program options. -DRH 20250319
#==================================================================================================
version = 1.00;
options = Options();
startCWD = '';
KnownVehicleActors = ['Drone'];

LOG_LEVEL_SILENT = 0;
LOG_LEVEL_NORMAL = 1;
LOG_LEVEL_VERBOSE = 2;

# Supposedly the sim is outputing SI units, which is unlike our other sims.  So i'm disabling
# use of this.  However I'm retaining it as a reminder in case, for whatever reason, things go 
# vac
# FT_TO_METER = 0.3048;

#===============================================================================================
#===============================================================================================
def log(logStr, logLevel=1, endStr=None):
    if options.logLevel == LOG_LEVEL_SILENT:   # silent mode catch all
        return;
    if logLevel <= options.logLevel:
        if not(endStr==None):
            print(logStr, end=endStr);
        else:
            print(logStr);

#==================================================================================================
# Emergency Close cleanup
# As I'm using some directory changing in this tool, I want to make sure if something bad happens the
# user is returned to the proper CWD.  So I'm going to register this function almost immediately 
# even thought the directory stuff will be near the end of the workflow.
#                         -- DRH 20250319
#==================================================================================================
def on_exit():
    if os.getcwd() != startCWD:
        if options.logLevel > 1:
            log("Returning to original working directory.");
        os.chdir(startCWD);

    if options.logLevel > 0:
        log("Program is closing.") 

#==================================================================================================
#==================================================================================================
def ImageType2Str(imageType):
    match(imageType):
        
        case airsim.ImageType.Scene:
            return 'Scene';
        case airsim.ImageType.DepthPlanar:
            return 'DepthPlanar';
        case airsim.ImageType.DepthPerspective:
            return 'DepthPerspective';
        case airsim.ImageType.DepthVis:
            return 'DepthVis';
        case airsim.ImageType.DisparityNormalized:
            return 'DisparityNormalized';
        case airsim.ImageType.Segmentation:
            return 'Segmentation';
        case airsim.ImageType.SurfaceNormals:
            return 'SurfaceNormals';
        case airsim.ImageType.Infrared:
            return 'Infrared';
        case airsim.ImageType.OpticalFlow:
            return 'OpticalFlow';
        case airsim.ImageType.OpticalFlowVis:
            return 'OpticalFlowVis';
        case _:
            return 'ERR_UNKNOWN_IMG_TYPE';

#==================================================================================================
# This validates for multirotor only (currently), it accepts the name and index integer as valid options
# Dev Note: I did this in a match / case with pairings for both readability and to show the 
#           relationship between Index and Name string.   -DRH 20250415 
#==================================================================================================
def validateCamera(cameraId):
    match cameraId:
        case 0 | 'front_center':   
            return True;
        case 1 | 'front_right':
            return True;
        case 2 | 'front_left':
            return True;
        case 3 | 'bottom_center':
            return True;
        case 4 | 'back_center':
            return True;
        case _:
            return False;

#==================================================================================================
# Returns the camera Enum and the parameters expected for the image request (best guess as of now)
# 
# Return Format: ImageType enum, Pixels_as_Float, CompressData
#
# Dev Note: I've really only played with Scene and Depth cameras.  I'm basing the rest of these params
#           on scanning the documentation, and may need to be adjusted if things aren't idea.
#==================================================================================================
def cameraTypeInfoFromString(strValue):
    match(strValue):
        case 'Scene':
            return airsim.ImageType.Scene, False, True;
        case 'DepthPlanar':
            return airsim.ImageType.DepthPlanar, True, False;
        case 'DepthPerspective':
            return airsim.ImageType.DepthPerspective, True, False;
        case 'DepthVis':
            return airsim.ImageType.DepthVis, True, False;
        case 'DisparityNormalized':
            return airsim.ImageType.DisparityNormalized, True, False;
        case 'Segmentation':
            return airsim.ImageType.Segmentation, False, True;
        case 'SurfaceNormals':
            return airsim.ImageType.SurfaceNormals, False, True;
        case 'Infrared':
            return airsim.ImageType.Infrared, False, True;
        case 'OpticalFlow':
            return airsim.ImageType.OpticalFlow, False, True;
        case 'OpticalFlowVis':
            return airsim.ImageType.OpticalFlowVis, False, True;
        case _:
            return [], False, False;

#==================================================================================================
# returns a dictionary for each known vehicle actor with a list of camera image requests to be made
#==================================================================================================
def createImgRequests(CameraViewData):

    # initialize dictionary with only known usable vehicle as key for now
    # Dev Note: Potential for expansion with new vehicle actors.
    ImgRequests = {key:[] for key in KnownVehicleActors};

    for Cv in CameraViewData:

        # validate the vehicle platform that has the onboard camera.  Right now we only support multirotor vehicles
        # named 'Drone', but I'm thinking this could expand.  This is basically overkill to accomodate future expansion
        # stub code.         -DRH
        camTarget = Cv['Target'];
        if not(camTarget in KnownVehicleActors):
            log(f'Camera Target Vehicle \'{camTarget}\' is not a valid vehicle to use.  Echo only supports {KnownVehicleActors} currently. Skipping');
            continue;

        # Validate basef on the known onboard cameras for a multirotor vehicle.  This is directly tied to what Airsim lists in their doc.
        if not( validateCamera(Cv['Camera']) ):
            log(f"Camera \'{Cv['Camera']}\' is not a valid Airsim Multirotor Camera; See Airsim Documentation. Skipping.");
            continue;

        # Go through each camera vision type and make an image request object for each.
        CameraTypes = Cv['Type'];
        for t in CameraTypes:
            typeEnum, pixelsAsFloat, compressData = cameraTypeInfoFromString(t);
            if typeEnum == []:
                log(f'Camera type \'{t}\' is not a valid Airsim Camera Type; See Airsim Documenation. Skipping.');
                continue;

            ImgRequests[camTarget].append(airsim.ImageRequest(Cv['Camera'], typeEnum, pixelsAsFloat, compressData));


    return ImgRequests;

#===============================================================================================
# Image processing into video
# converts the batch of image files into a video using ffmpeg
#===============================================================================================
def exportToMovie(imgPath, outPath, ext='png', framerate=20, quality=5):
    # go to the directory of the images generated
    os.chdir(imgPath);

    # process the files
    # the current settings here are for a 1080p with a higher quality end result.
    # Dev Note: I'm currently using the quality flag (-q) to set a general trade off between
    #           size and quality.  In some general testing  the '5' I'm using gets me about half
    #           file size between 1 and no entry (the default being very lossy).
    (
        ffmpeg
        .input(f'%05d.{ext}', framerate=framerate)
        .output(outPath,  loglevel='error', q=quality)
        .global_args('-stats')
        .run()
    )

    #return to previous CWD
    os.chdir(startCWD);

#===============================================================================================
# Threading helper function
#===============================================================================================
def processPngExport(index, ImgRequest, destinationPath):
    imgName = f"{index:05d}.png";
    filename = os.path.join(destinationPath,imgName);
    airsim.write_file(filename, ImgRequest.image_data_uint8);

#===============================================================================================
#===============================================================================================
def processPfmExport(index, ImgRequest, destinationPath):
    imgName = f"{index:05d}.pfm";
    filename =  os.path.join(destinationPath,imgName);
  
    # Dev Note: using Airsims own processing of this data.
    imgData = airsim.get_pfm_array(ImgRequest);

    # Flip the float array row wise because the default from airsim is flipped vertically.
    imgData = np.flipud(imgData);

    # use Airsim's own pfm writing routine because I am over trying to find a workable solution
    airsim.write_pfm(filename, imgData);

#===============================================================================================
#===============================================================================================
def validatePath(pathString, doQuitOnFailure = False):
    if os.path.exists(pathString):
        return True;
    else:
        if options.logLevel>0:
            log(f'Warning: Provided path \'{pathString}\' does not exist.');
        if doQuitOnFailure:
            exit();
        return False;

#===============================================================================================
#===============================================================================================
def validateScenarioData(droneData, baseballData):
    droneKeys = set(['timeSec', 'posNED_1', 'posNED_2', 'posNED_3', 'pitchRad','rollRad','yawRad']);
    loadedKeys = set(droneData.keys()); 
    if not droneKeys.issubset(loadedKeys):
        log(f'Error: Loaded drone file does not contain the necessary data elements. '
              'Check the file, a new one may need to be generated. Aborting');
        return False;

    if not options.skipBaseball:
        # This rest of these only matter if the baseball is involved in this execution.

        bbKeys = set(['timeSec', 'posNED_1', 'posNED_2', 'posNED_3', 'trackRad']);
        loadedKeys = set(baseballData.keys()); 
        if not bbKeys.issubset(loadedKeys):
            log(f'Error: Loaded baseball file does not contain the necessary data elements. '
                  'Check the file, a new one may need to be generated. Aborting');
            return False;

        # check data count, must have the same number of time stamps.
        droneTimeCount = len(droneData['timeSec']);
        bbTimeCount = len(baseballData['timeSec']);
        if not(droneTimeCount == bbTimeCount):
            log(f'Error: Drone file ({droneTimeCount}) and Baseball file ({bbTimeCount}) do not have the same '
                 'number of time records. Can not continue. Aborting.');
            return False;

        # verify that the time stamps are roughly the same (epsilon test, because I've long learned
        # my lesson about direct equivalence tests of floating point numbers.)
        for idx, t in enumerate(droneData['timeSec']):
            if not (abs(t-baseballData['timeSec'][idx]) < 0.000001):
                log(f"Error: detected time codes that do not match: Drone({idx})={t}s, "
                    f"Baseball({idx})={baseballData['timeSec'][idx]}s. Can not continue. Aborting.");
                return False;

    return True;

#===============================================================================================
#===============================================================================================
def buildOutputDirectoryStructure(startCWD, recordingTag, timestamp, imgRequestData):
    outputDir = f"{recordingTag}_{timestamp}";
    pathToOutputDir = os.path.join(startCWD, outputDir);

    capturePaths = {key: {} for key in imgRequestData.keys()};
    for cpKey, cpValue in capturePaths.items():
        cpValue.update( {'base': os.path.join(pathToOutputDir, cpKey)} );
        for ir in imgRequestData[cpKey]:
            imgTypeStr = ImageType2Str(ir.image_type);
            irKey = f'{ir.camera_name}-{imgTypeStr}';
            irPath = os.path.join(pathToOutputDir, cpKey, ir.camera_name, imgTypeStr);
            cpValue.update({irKey: irPath});

            if not os.path.exists(irPath):
                os.makedirs(irPath);
            else:
                log(f'Warning: Path structure to \'{irPath}\' already exists.  Consider Aborting.');

    return pathToOutputDir, capturePaths;

#===============================================================================================
# Base application script.
#===============================================================================================
if __name__ == "__main__":
  
    recordingTag = 'Baseline';

    # set up close cleanup before anything else
    atexit.register(on_exit);
    startCWD = os.getcwd();

    # Process options
    # Process JSON first
    # Dev Note: I want the JSON to be the default values loaded in, but they can be over-written
    #           by the command line options. Therefore, these go on the record in the Options object
    #           first, and then process the rest of the command line options.
    with open('BamEcho.json', 'r') as cfg:
        cfgData = json.load(cfg);

        options.logLevel = cfgData['LogLevel'];
        options.doImageCapture = cfgData['ImageCapture'];
        options.doVideoExport = cfgData['VideoExport'];
        options.skipBaseball = cfgData['IgnoreBaseball'];
        options.cleanupFrames = cfgData['CleanupFrames'];
        options.compressFrames = cfgData['CompressFrames'];  # Currently ineffective, hook up in ImgRequest Defaults above.
        recordingTag = cfgData['DefaultTag'];

        # these are only configurable via JSON. so go ahead and process them
        # Should return a dictionary with a valid vehicle and at least one img request.  Otherwise this is a fail.
        imgRequestData = createImgRequests(cfgData['CameraViews']);
        imgReqCount = 0;

        for v in imgRequestData:
            imgReqCount =+ len(v); 
 
    # Process options
    # Dev Note: I'm doing this first even though they are expected after the manadatory file
    #           arguments because of the option to mute the baseball.  if -noBB is enabled, the
    #           2nd file argument, for baseball trajectory, is actually optional.  I'd rather just
    #           conditional ignore all that code instead of coming up with some complex argument
    #           handling.                   -DRH 2025
    # Update: Since I've added JSON defaults above this point. These command line arguments are given 
    #         override priority.  For example, the JSON default may have the LogLevel at Normal(1), 
    #         but the user can specify -v and get the LogLevel overriden and set to Verbose(2)
    if "-s" in sys.argv:
       options.logLevel = LOG_LEVEL_SILENT;    # set silent
    if "-v" in sys.argv:
       options.logLevel = LOG_LEVEL_VERBOSE;    # verbose
    if "-i" in sys.argv:
       options.interactiveMode = True;
    if '-t' in sys.argv:
       recordingTag = sys.argv[sys.argv.index("-t")+1];
    if '-dt' in sys.argv:
       options.doImageCapture = False;
       options.doVideoExport = False;
    if '-noBB' in sys.argv:
       options.skipBaseball = True;
    if '-rm' in sys.argv:
       options.cleanupFrames = True;


    # Processing arguments
    if options.skipBaseball and (len(sys.argv) < 2):
        # require first argument
        log('Error: One file path is required to run (No Baseball Mode).  Example: BamEcho.py <DroneTrajectoryPath> [options]');
        exit();
    if not(options.skipBaseball) and (len(sys.argv) < 3):
        # require first two arguments
        log('Error: Two file paths are required to run.  Example: BamEcho.py <DroneTrajectoryPath> <BaseballTrajectoryPath> [options]');
        exit();

    #acquire parameters
    if len(sys.argv) > 1:
        filenameVehicle = sys.argv[1];
        validatePath(filenameVehicle, True);
        if not(options.skipBaseball):
            filenameBaseball = sys.argv[2];
            validatePath(filenameBaseball, True);
    
    # Safety Check on whether we have enough image information to proceed based on the options set for operations.
    if (options.doImageCapture or options.doVideoExport) and (imgReqCount == 0):
       log('Error: There are no valid camera views specified. Cannot conduct Image/Video operations. See BamEcho.json and Documentation');
       exit();

    # process trajectory data
    droneData = bamCsvLoad.parseCSV(filenameVehicle, True);
    if not(options.skipBaseball):
        baseballData = bamCsvLoad.parseCSV(filenameBaseball, True);
    else:
        baseballData = [];
    validData = validateScenarioData(droneData, baseballData);
    if not validData:
        log('Invalid scenario data detected. Closing'
            '----------------------------------------------------------------------------------------------------');
        exit();

    # timestamping
    datetimestamp = datetime.datetime.now();
    timestamp = datetimestamp.strftime("%Y%m%d_%H%M%S");

    # Pathing for img generation and video export
    # make output directory if it does not exist
    # Generate paths for \Drone\
    # Generate paths for \Drone\Camera1\ImageType1
    # Generate paths for \Drone\Camera1\ImageType2
    pathToOutputDir, capturePaths = buildOutputDirectoryStructure(startCWD, recordingTag, timestamp, imgRequestData);
    
    # Execution Preamble
    log('=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-');
    log(f' BamEcho Video Generation Tool (v.{version})');
    log(f' Project Tag: {recordingTag}');
    log(f' Drone File: {filenameVehicle}');
    if not(options.skipBaseball):
        log(f' Baseball File: {filenameBaseball}');
    log(f' Output Directory: {pathToOutputDir}');
    log('----------------------------------------------------------------------------------------------------');

    # Connect to Airsim and start the preliminaries
    log('Initializing Airsim Connection');
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
    except msgpackrpc.error.TransportError:
        print('Cannot establish connection to Unreal/Airsim Environment. Make sure it is running. Aborting.');
        exit();
    
    log('Enabling API Control', LOG_LEVEL_VERBOSE);
    client.enableApiControl(True)

    # Validate Access to the Drone
    if not('Drone' in client.listVehicles()):
        print('Cannot locate a Multirotor vehicle named \'Drone\' in Airsim.  Please add on in the settings JSON file. Aborting.');
        exit();

    log('Arming the Drone', LOG_LEVEL_VERBOSE);
    client.armDisarm(True, vehicle_name='Drone');

    if not(options.skipBaseball):
        # Validate Access to the Baseball model
        if not('BamBaseball' in client.simListAssets()):
            print('Cannot locate the Asset named \'BamBaseball\' in Unreal.  Please add it to the Environment and Level via Unreal Editor. Aborting.');
            exit();

        # Dev Note: It's important to point out here that I'm not using the REGEX option in 
        #           simListSceneObjects() to narrow down the last for the baseball because I 
        #           had several Crashes of the Unreal environment when I tried to REGEX with 
        #           wildcard operators.  So I'm just avoiding it entirely because I consider
        #           it to be unstable.
        #                            -- DRH 20250320
        if not('BamBaseball' in client.simListSceneObjects()):
            log('Spawning the Baseball', LOG_LEVEL_VERBOSE);
            pose = airsim.Pose(airsim.Vector3r(0,0,-3),None);
            bbObjName = client.simSpawnObject('BamBaseball', 'BamBaseball', pose, airsim.Vector3r(.25,.25,.25), False);
        else:
            bbObjName = 'BamBaseball';

    log('Moving Actor(s) to their starting locations',LOG_LEVEL_VERBOSE);
    # pos = airsim.Vector3r(droneData['posNED_1'][0]*FT_TO_METER,
    #                       droneData['posNED_2'][0]*FT_TO_METER,
    #                       droneData['posNED_3'][0]*FT_TO_METER);
    # orient = airsim.to_quaternion(droneData['pitchRad'][0]*FT_TO_METER,
    #                               droneData['rollRad'][0]*FT_TO_METER,
    #                               droneData['yawRad'][0]*FT_TO_METER);
    pos = airsim.Vector3r(droneData['posNED_1'][0],
                          droneData['posNED_2'][0],
                          droneData['posNED_3'][0]);
    orient = airsim.to_quaternion(droneData['pitchRad'][0],
                                  droneData['rollRad'][0],
                                  droneData['yawRad'][0]);
    client.simSetVehiclePose(airsim.Pose(pos,orient), True, vehicle_name='Drone');

    if not(options.skipBaseball):
        # pos = airsim.Vector3r(baseballData['posNED_1'][0]*FT_TO_METER,
        #                       baseballData['posNED_2'][0]*FT_TO_METER,
        #                       baseballData['posNED_3'][0]*FT_TO_METER);
        pos = airsim.Vector3r(baseballData['posNED_1'][0],
                              baseballData['posNED_2'][0],
                              baseballData['posNED_3'][0]);
        orient = airsim.to_quaternion(0, 0, baseballData['trackRad'][0]);
        client.simSetObjectPose(bbObjName,airsim.Pose(pos,orient))

    

    renderStartTime = time.perf_counter();
    imgThreads = [];

    times = droneData['timeSec'];
    count = len(times);
    processedFrames = 0;
    log('Transmitting flights');
    # DEV NOTE: Assuming that all trajectories loaded have the same number of time stamp data sets.  if this isn't the
    # case, this will break big time.
    #for t in range(20):   # DEBUG
    for t in range(len(times)):

        pos = airsim.Vector3r(droneData["posNED_1"][t],
                              droneData["posNED_2"][t],
                              droneData["posNED_3"][t]);
        # pos = airsim.Vector3r(droneData["posNED_1"][t]*FT_TO_METER,
        #                       droneData["posNED_2"][t]*FT_TO_METER,
        #                       droneData["posNED_3"][t]*FT_TO_METER);
        orient = airsim.to_quaternion(droneData['pitchRad'][t], droneData['rollRad'][t], droneData['yawRad'][t]);
        dronePose = airsim.Pose(pos, orient);
        client.simSetVehiclePose(dronePose,True,vehicle_name='Drone');

        if not(options.skipBaseball):
            pos = airsim.Vector3r(baseballData["posNED_1"][t],
                                  baseballData["posNED_2"][t],
                                  baseballData["posNED_3"][t]);
            # pos = airsim.Vector3r(baseballData["posNED_1"][t]*FT_TO_METER,
            #                       baseballData["posNED_2"][t]*FT_TO_METER,
            #                       baseballData["posNED_3"][t]*FT_TO_METER);
            rot = airsim.to_quaternion(0, 0, baseballData['trackRad'][t]);
            bbPose = airsim.Pose(pos, rot);
            client.simSetObjectPose(bbObjName,bbPose);

 
        if options.doImageCapture:

            # Take the necessary photos (we'll start with Visible and Depth Planar from the front camera only)
            # Output the screen shots to the designated folders (one folder per cam view)
            # Pawning these off to threads provides a performance boost, but the GetImage request above is still a signficant hog.
            imgs = client.simGetImages(imgRequestData['Drone'], vehicle_name='Drone');

            for idx, img in enumerate(imgs):
                pathKey = f"{imgRequestData['Drone'][idx].camera_name}-{ImageType2Str(imgRequestData['Drone'][idx].image_type)}";
                if imgRequestData['Drone'][idx].pixels_as_float:
                    imgThreads.append(threading.Thread(target=processPfmExport, args=(t, img, capturePaths['Drone'][pathKey]) ));
                else:
                    imgThreads.append(threading.Thread(target=processPngExport, args=(t, img, capturePaths['Drone'][pathKey]) ));
                imgThreads[-1].start();

        processedFrames += 1;
        progress = ((t+1)/count) * 100.0;
        if (t+1) == count:
            # last frame, use standard newline printing
            log(f'Processed {t+1} of {count} ({progress:3.1f}%)',LOG_LEVEL_VERBOSE);
        else: 
            log(f'Processed {t+1} of {count} ({progress:3.1f}%)',LOG_LEVEL_VERBOSE, endStr='\r');

        #END For Loop

    
    # Wait for all image capturing threads to terminate before we move onto video processing.
    for t in imgThreads:
        t.join();
    
    # Generate the full videos now that we've produced all of the still image frame for each data point.
    if options.doVideoExport:
        log('Processing Videos');

        # Dev Note: In the future, this will be fully dependent on what is set up in the config file
        #           (Like I've done earlier in the program)
        videoOutputs = {};
        vehicle = 'Drone';                         # parameterize in future
        for idx, imgReq in enumerate(imgRequestData[vehicle]):
            camera = imgReq.camera_name;
            imgtype = ImageType2Str(imgReq.image_type);
            pathKey = f'{camera}-{imgtype}';
            pathToFrames = capturePaths[vehicle][pathKey];
            pathToVideo = os.path.join(pathToOutputDir,f'{vehicle}_{camera}_{imgtype}.avi');
            if imgReq.pixels_as_float:
                fileExtension = 'pfm';
            else:
                fileExtension = 'png';
            
            log(f'{vehicle}\'s {imgtype} from {camera} camera saved to: {pathToVideo}',LOG_LEVEL_VERBOSE);
            # Dev Note: (TODO) This likely needs to account for input framerate incase there
            #           is disparity between the two (eg, really high input, pare down to normal
            #           video output.)
            exportToMovie(pathToFrames, pathToVideo, ext=fileExtension,
                          framerate=cfgData['VideoProcessing']['OutputFrameRate'],
                          quality=cfgData['VideoProcessing']['QualityFactor']);

            videoOutputs.update({f'{vehicle}-{camera}-{imgtype}': pathToVideo});


    # track our process time.
    timeElapsedInSec = time.perf_counter() - renderStartTime;
    
    # that's enough fun for now. let's quit cleanly
    if options.cleanupFrames:
        # remove all the frame subdirectories as requested (helps on the space that rapidly gets eaten up by
        # this process)
        for veh in capturePaths:
            shutil.rmtree(capturePaths[veh]['base']);

    # Execution Summary
    log('----------------------------------------------------------------------------------------------------');
    log(f'Time elapsed: {timeElapsedInSec:.2f}');
    log(f' Processed {processedFrames} Frames');
    log(' Outputs:');
    for out in videoOutputs:
        log(f'   {out}: {videoOutputs[out]}');
    # run post proc
    log('Closing BamEcho.');
    log('=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-');

 