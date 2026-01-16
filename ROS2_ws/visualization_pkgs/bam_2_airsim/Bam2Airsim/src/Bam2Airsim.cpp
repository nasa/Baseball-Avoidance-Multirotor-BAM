/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: Bam2Airsim.cpp                                                                *
 *                                                                                      *
 *  Description: Bam Pose Communication Server Application                              *
 *               This application will receiver published pose messages for a multirotor*
 *               and a baseball from BAM and then transmit that data to an Airsim       *
 *               equipped Unreal Environment.  The application, upon startup, will do   *
 *               some diagnostics and spawing to ensure the environment is correct for  *
 *               for operations.  Any issues will be logged and the node will terminate.*
 *                                                                                      *
 *               The tool has two command line options (to be passed before --ros-args) *
 *               "-nobb" - Disables Baseball subscription and transmission.             *
 *               "-do"   - Enable debugging output (will make large log files)          *
 ****************************************************************************************/
//
//  Mapping for Nodes / Topics
//
//  s-function::Node(multirotor_pub) --> Topic(/pub_pose) --> Bam2Airsim::Node(multirotor_sub)
//  s-function::Node(baseball_pub) --> Topic(/bball_pub_pose) --> Bam2Airsim::Node(baseball_sub)
//
// ***************************************
//  
// Bam2Airsim and Airsim 
// 
// Currently, there should be the following actors:
// --multirotor vehicle named "Drone" from the JSON
// --baseball 3d mesh named "BamBaseball" spawned by Airsim
//

#pragma warning( disable : 4305)

// Disabled because I'm not sure if I need this or if it even exists in ROS2 (StrictMode)
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"  // gone in Ros2
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>


#include "BamAirsimNode.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"


using namespace msr::airlib;

//=======================================================================================
//=======================================================================================
bool bUseBaseball = true;
bool bDebugOutput = false;


//=======================================================================================
//=======================================================================================
std::string getTimeStamp()
{
  // using c++11 approach (gotta get away from old C style of doing things if possible
  auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  std::stringstream ssStamp;
  ssStamp << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");

  return ssStamp.str();
}

//=======================================================================================
//=======================================================================================
std::shared_ptr<spdlog::logger> startDebugLogger (void)
{
  std::stringstream ssFilename;
  std::string strDir = "log/";
  ssFilename << "Bam2Airsim_output_" << getTimeStamp() << ".log";

  // Create a console and file target for the logger (we want both), speed logger uses "sink" classes
  //  Dev Note: this code is based on the example code found in the spdlog gitrepo MD.
  auto skConsole = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  skConsole->set_level(spdlog::level::info);
  skConsole->set_pattern("[Bam2Airsim][%Y-%m-%d %H:%M:%S.%e] >> %v");

  auto skFile = std::make_shared<spdlog::sinks::basic_file_sink_mt>(strDir+ssFilename.str(), true);
  skFile->set_level(spdlog::level::trace);
  skFile->set_pattern("[Bam2Airsim][%Y-%m-%d %H:%M:%S.%e] >> %v");

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(skConsole);
  sinks.push_back(skFile);

  auto logger = std::make_shared<spdlog::logger>("main", sinks.begin(), sinks.end());
  logger->set_level(spdlog::level::debug);

  // register logger for whole program access.
  spdlog::register_logger(logger);

  return logger;
}


//=======================================================================================
//=======================================================================================
void stopDebugLogger (void)
{
  spdlog::info("Shutting down Log and Application...");

  spdlog::shutdown();
}


//=======================================================================================
//=======================================================================================
void parseArguments(int argc, char* argv[])
{
  bool bTokenFound = false;
  auto log = spdlog::get("main");
  
  for (int i = 0; i < argc; i++)
  {
    if (strcmp("-nobb", argv[i]) == 0)
    {
      bUseBaseball = false;
      log->info("Detected flag to disable Baseball Logic.");
    }
    else if (strcmp("-do", argv[i]) == 0)
    {
      bDebugOutput = true;
      log->info("Detected flag enable Pose output debugging.");
    }
  }
}


//=======================================================================================
//=======================================================================================
const bool validateAirsimConfiguration(msr::airlib::MultirotorRpcLibClient* pClient)
{
  auto log = spdlog::get("main");

  // Dev Note: Need error check here, but the c++ version might be slightly different than the python one.  The python
  //           one let me try/catch for an RPC error.
  // Dev Note: The C++ version of this check seems to wait indefinitely whereas on python it doesn't block.  I'm leaving the
  //           try / catch for now for any critical failure, but really the blocking wait isn't ideal as a "check" for Airsim.
  //                                 -DRH 202505
  try
  {
    pClient->confirmConnection();
  }
  catch (const rpc::rpc_error& e)
  {
    std::ostringstream ssOut;
    ssOut << "Error: There was a critical communication failure when attempting to confirm Airsim connection. Error Code:" << e.what();
    log->info(ssOut.str());
    return false;
  }

  // Do some safety checks on the configured environment
  auto activeVehicles = pClient->listVehicles();
  if (std::find(activeVehicles.begin(), activeVehicles.end(), std::string("Drone")) == activeVehicles.end())
  {
    std::ostringstream ssOut;
    log->info("Error: Unable to detect 'Drone' vehicle in Unreal Airsim Environment.  Please check your Environment and retry.");
    return false;
  }
  log->info("Multirotor vehicle \'Drone\' Confirmed.");

  if (bUseBaseball)
  {
    auto sceneObjs = pClient->simListSceneObjects();
    if (std::find(sceneObjs.begin(), sceneObjs.end(), std::string("BamBaseball")) == sceneObjs.end())
    {
      // Baseball not detected as an active vehicle.  Try to spawn it.  If this fails, abort.
      auto assets = pClient->simListAssets();
      if (std::find(assets.begin(), assets.end(), std::string("BamBaseball")) == assets.end())
      {
        // Failed to find even the baseball asset.  Quit out
        log->info("Error: Unable to spawn the baseball because the asset 'BamBaseball' is not an asset in the Airsim Unreal Environment."
                  " See the README documentation on the BamBaseball. You likely need to add the asset to the Unreal level and rebuild.");
        return false;
      }
      log->info("Object asset \'BamBaseball\' found, but is not spawned.");

      // Spawn the baseball
      try
      {
        auto objName = pClient->simSpawnObject("BamBaseball", "BamBaseball",
          Pose(Vector3r(0, 0, -2), Quaternionr(1, 0, 0, 0)),
          Vector3r(.12, .12, .12), false);
      }
      catch (const rpc::rpc_error& e)
      {
        std::ostringstream ssOut;
        ssOut << "Error: There was a critical communication failure when attempting to spawn the 'BamBaseball'. Error Code:" << e.what();
        log->info(ssOut.str());
        return false;
      }
      log->info("Object asset \'BamBaseball\' spawned as Actor.");
    }
    else
      log->info("Object asset \'BamBaseball\' Confirmed.");
  }
  else
    log->info("Skipping Baseball (Disabled via Options)");

  return true;
}

//=======================================================================================
// Bam2Airsim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//=======================================================================================
int main(int argc, char *argv[])
{
  // start Logging
  auto log = startDebugLogger();
  log->info("Starting up Bam2Airsim");
 
  // Checking for some command line options.  this is an initial expansion and small, but 
  // it would be nice to be able to easily turn on / off the baseball from the command line.
  parseArguments(argc, argv);
 
  // create connection to Airsim in the Unreal Environment
  msr::airlib::MultirotorRpcLibClient client;

  // Run several checks and operations to make sure the Airsim environment is ready for BAM operations.
  // Dev Note:  this is preliminary based on assumptions of how things are going to work.  They might need
  //            to be changed, but the honus is largely on the user to get the Unreal environment set just 
  //            right (using the documentation for instruction). -DRH 202503
  log->info("Checking Airsim Unreal Environment...");
  if (validateAirsimConfiguration(&client) == false)
  {
    return EXIT_FAILURE;
  }

  log->info("Enabling Airsim controls...");
  client.enableApiControl(true);
  client.armDisarm(true);

  log->info("Intializing ROS Connection...");
  rclcpp::init(argc, argv);

  // Create the ROS Nodes and Topics for each potential actor in Unreal/Airsim that will receive BAM pose data.
  // Up top, I've added the expected NodeName-->Topic name mapping on each object for clarity / expectations as of 
  // this initial version.      -DRH
  log->info("Registering ROS Nodes and Topics...");


  // Dev Note:  Fun note, at least for the subscribers, the forward slash is required at the front of topics in ros2 humble.  THe
  //            publishers seem to to implicitly add this (as I found out through testing / debugging), but the subscriber did not.
  //            So save yourself some trouble and add that forward slash.  Don't find out the way I did, kids.
  //                                    -DRH 202505
  // Create Node / Msg handler -> Unreal Actor name, Unreal Actor Type, ROS2 Unique Node Name, ROS2 Topic Name, Airsim Client
  auto mrNode = std::make_shared<BamAirsimNode>("Drone", AT_VEHICLE, "multirotor_sub_node", "/pub_pose", &client, bDebugOutput);

  std::shared_ptr<BamAirsimNode> bbNode = nullptr;  // Instantiated here to allow for scope if being used.
  if (bUseBaseball)
    bbNode = std::make_shared<BamAirsimNode>("BamBaseball", AT_OBJECT, "baseball_sub_node", "/bball_pub_pose", &client, bDebugOutput);
  
  // Create an executor to run the N nodes.  (this can be done with a multi-threaded executor, but I'm going for
  // relatively simplicity.
  log->info("Running main ROS execution...");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mrNode);
  if (bUseBaseball)
    executor.add_node(bbNode);

  // Run the ROS cycle indefinitely
  executor.spin();
  
  log->info("Terminating ROS execution...");
  rclcpp::shutdown();

  log->info("Disabling Airsim controls...");
  client.armDisarm(false);

  log->info("Closing ROS Server...");

  // stop logging
  stopDebugLogger();

  return EXIT_SUCCESS;
}
