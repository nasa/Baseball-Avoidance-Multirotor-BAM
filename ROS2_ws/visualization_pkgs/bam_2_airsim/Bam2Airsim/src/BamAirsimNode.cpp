/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: BamAirsimNode.cpp                                                             *
 *                                                                                      *
 *  Description: BamAirsimNode Class Implementation                                     *
 *                                                                                      *
 ****************************************************************************************/

#pragma warning( disable : 4244)

#include <iomanip>

#include "spdlog/spdlog.h"
#include "BamAirsimNode.hpp"

using namespace msr::airlib;


//***************************************************************************************
//=======================================================================================
//  CONSTRUCTION AND DESTRUCTION
//=======================================================================================
//***************************************************************************************


//=======================================================================================
//=======================================================================================
BamAirsimNode::BamAirsimNode(const char* szActorName, const ActorType eType,
                             const char* szUniqueNodeName, const char* szTopicName,
                             msr::airlib::MultirotorRpcLibClient* pAirsimClient,
                             bool bDebugOutput)
  : Node(szUniqueNodeName),
    m_eType(eType),
    m_strActorName(szActorName),
    m_strNodeName(szUniqueNodeName),
    m_strTopicName(szTopicName),
    m_pClient(pAirsimClient),
    m_bDebugOutput(bDebugOutput),
    m_dElapsedTimeSinceLastDebugMsg(1.1),  // default above 1 so we trasmit the first data record if debugging.
    m_uiDebugCount(0)
{
  // Create the connection to the desired Topic in ROS
  m_pSubscriber = this->create_subscription<pose_msg_t>(szTopicName, 10,
                          std::bind(&BamAirsimNode::updateMsg, this, std::placeholders::_1));
  
  std::ostringstream ssOut;
  ssOut << "BamAirsimNode('" << szUniqueNodeName << "," << szTopicName << "') has been created.";
  RCLCPP_INFO(this->get_logger(), ssOut.str().c_str());
  spdlog::get("main")->info(ssOut.str());
}


//=======================================================================================
//=======================================================================================
BamAirsimNode::~BamAirsimNode()
{
  std::ostringstream ssOut;
  ssOut << "BamAirsimNode('" << this->get_name() << "," << m_strTopicName << "') has been destroyed.";
  RCLCPP_INFO(this->get_logger(), ssOut.str().c_str());
}

//=======================================================================================
//=======================================================================================
void BamAirsimNode::updateMsg(const std::shared_ptr<pose_msg_t> msg)
{
  std::cout << std::fixed;
  std::cout << std::setprecision(3);

  auto x = msg->pose.position.x;
  auto y = msg->pose.position.y;
  auto z = msg->pose.position.z;
  auto ow = msg->pose.orientation.x; // Note that the quat w component is the 
  auto ox = msg->pose.orientation.y; // last element which agrees with the 
  auto oy = msg->pose.orientation.z; // ROS2 msg PoseStamped
  auto oz = msg->pose.orientation.w;  

  if (m_bDebugOutput)
  {
    auto hrtTime = std::chrono::high_resolution_clock::now();
    auto timeElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(hrtTime - m_hrtLastTimestamp);
    m_dElapsedTimeSinceLastDebugMsg = static_cast<double>(timeElapsedMs.count() / 1000.0);  // convert to seconds from milli

    if (m_dElapsedTimeSinceLastDebugMsg >= 1.0)
    {
      std::ostringstream ssOut;
      ssOut << " POS(x,y,z): " << std::right << std::setw(7) << x;
      ssOut << ", " << std::right << std::setw(7) << y;
      ssOut << ", " << std::right << std::setw(7) << z;

      ssOut << "ROT(w,x,y,z): " << std::right << std::setw(7) << ow;
      ssOut << ", " << std::right << std::setw(7) << ox;
      ssOut << ", " << std::right << std::setw(7) << oy;
      ssOut << ", " << std::right << std::setw(7) << oz;

      m_dElapsedTimeSinceLastDebugMsg = 0.0;
      spdlog::get("main")->info(ssOut.str());
    }

    m_hrtLastTimestamp = hrtTime;
  }

  msr::airlib::Vector3r p(x, y, z); // What frame is expected by AirSim?
  msr::airlib::Quaternionr o(ow, ox, oy, oz);  // What frame is expected by AirSim?

  // Publish pose to Airsim
  // The type of Actor matters here because Airsim handles its custom vehicles different than
  // the general Unreal Actor objects.  I assume this is book-keeping on the backend from which
  // list of objects they are referencing mostly (because the data otherwise is the same).  Anyway,
  // if things are failing, make sure the appropriate actor type was given at creation.  Otherwise this 
  // will do nothing (don't believe it will cause an exception. hahahahahahahaha..... sorry if I'm wrong).
  //                      -DRH 202505
  switch (m_eType)
  {
  case AT_VEHICLE:
    m_pClient->simSetVehiclePose(Pose(p, o), true, m_strActorName);
    break;
  case AT_OBJECT:
    m_pClient->simSetObjectPose(m_strActorName, Pose(p, o), true);
    break;
  default:
    // Type unknown type.  Ignore.
    break;
  }

  m_uiDebugCount++;
  if ((m_uiDebugCount % 1000) == 0)
  {
    std::ostringstream ssOut;
    ssOut << "(" << m_strTopicName << "->" << m_strActorName << "): Read and Re-Published " << m_uiDebugCount << " Pose records";
    spdlog::get("main")->info(ssOut.str());
  }
}