/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: TestPosePub.cpp                                                               *
 *                                                                                      *
 *  Description: Test Publish Application                                               *
 *               This server puts out sample pose messages for testing Bam2Airsim.  It  *
 *               will put out a multirotor and baseball pose message which will travel  *
 *               in circular paths around the origin of the Unreal level.               *
 *                                                                                      *
 *               The tool has two command line options (to be passed before --ros-args) *
 *               "-nobb" - Disables Baseball subscription and transmission.             *
 *               "-do"   - Enable debugging output (will make large log files)          *
****************************************************************************************/
//
//  Mapping for Nodes / Topics
//
//  s-function::Node(multirotor_pub) --> Topic(multirotor/pose) --> Bam2Airsim::Node(multirotor_sub)
//  s-function::Node(baseball_pub) --> Topic(baseball/pose) --> Bam2Airsim::Node(baseball_sub)
//
//


#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>


#include "BamTestPubNode.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"


char szBaseballPoseTopic[24] = "bball_pub_pose";
char szOwnshipTopic[24] = "pub_pose";

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
  ssFilename << "TestPosePub_output_" << getTimeStamp() << ".log";

  // Create a console and file target for the logger (we want both), speed logger uses "sink" classes
  //  Dev Note: this code is based on the example code found in the spdlog gitrepo MD.
  auto skConsole = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  skConsole->set_level(spdlog::level::info);
  skConsole->set_pattern("[TestPosePub][%Y-%m-%d %H:%M:%S.%e] >> %v");

  auto skFile = std::make_shared<spdlog::sinks::basic_file_sink_mt>(strDir+ssFilename.str(), true);
  skFile->set_level(spdlog::level::trace);
  skFile->set_pattern("[TestPosePub][%Y-%m-%d %H:%M:%S.%e] >> %v");

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
// Bam2Airsim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//=======================================================================================
int main(int argc, char *argv[])
{
  // start Logging
  auto log = startDebugLogger();
  log->info("Starting up TestPubPose");
  
  // Checking for some command line options.  this is an initial expansion and small, but 
  // it would be nice to be able to easily turn on / off the baseball from the command line.
  parseArguments(argc, argv);

  log->info("Enabling ROS Connection...");
  rclcpp::init(argc, argv);

  // Create the ROS Nodes and Topics for each potential actor in Unreal/Airsim that will receive BAM pose data.
  // Up top, I've added the expected NodeName-->Topic name mapping on each object for clarity / expectations as of 
  // this initial version.      -DRH
  log->info("Registering ROS Nodes and Topics...");

  // Create Node / Msg handler -> Unreal Actor name, Unreal Actor Type, ROS2 Unique Node Name, ROS2 Topic Name, Airsim Client
  auto mrNode = std::make_shared<BamTestPubNode>("multirotor_pub_node", "/pub_pose", bDebugOutput);

  std::shared_ptr<BamTestPubNode> bbNode = nullptr;  // Instantiated here to allow for scope if being used.
  if (bUseBaseball)
    bbNode = std::make_shared<BamTestPubNode>("baseball_pub_node", "/bball_pub_pose", bDebugOutput);
  
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

  // stop logging
  log->info("Closing ROS Server...");
  stopDebugLogger();

  return EXIT_SUCCESS;
}
