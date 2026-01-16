/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: BamTestPubNode.cpp                                                            *
 *                                                                                      *
 *  Description: BamTestPubNode Class Implementation                                    *
 *                                                                                      *
 ****************************************************************************************/

#pragma warning( disable : 4244)

#include <chrono>
#include <iomanip>
#include <cmath>
#include <random>

#include "spdlog/spdlog.h"
#include "BamTestPubNode.hpp"

const double PI = 3.1415926535;
const double RATE_ONE = 3.0; // Degree per second
const double RATE_ONE_RAD = RATE_ONE * (PI / 180.0);
const double DEG2RAD = PI / 180.0;
using namespace std;

//***************************************************************************************
//=======================================================================================
//  CONSTRUCTION AND DESTRUCTION
//=======================================================================================
//***************************************************************************************


//=======================================================================================
//=======================================================================================
BamTestPubNode::BamTestPubNode(const char* szUniqueNodeName, const char* szTopicName, bool bDebugOutput)
  : Node(szUniqueNodeName),
    m_strNodeName(szUniqueNodeName),
    m_strTopicName(szTopicName),
    m_dFrequency(20), // HZ
    m_dTheta(0.0),
    m_dRadius(10.0),
    m_dTurnRate(RATE_ONE_RAD),
    m_bDebugOutput(bDebugOutput),
    m_dElapsedTimeSinceLastDebugMsg(1.1),  // default above 1 so we trasmit the first data record if debugging.
    m_uiDebugCount(0)
{
  // Create the connection to the desired Topic in ROS
  // This is hardcoded right now because I wasn't sure my frequency was dynamically working
  auto timeStep = std::chrono::milliseconds(50);

  m_pPublisher = this->create_publisher<pose_msg_t>(szTopicName, 10);
  m_pTimerFunc = this->create_wall_timer(timeStep, std::bind(&BamTestPubNode::publishPose, this));

  // Give the theta a random starting value along 0->359.9 degrees converted to rads
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr(0, 3599); // define the range
  m_dTheta = DEG2RAD * (distr(gen) / 10.0);

  std::ostringstream ssOut;
  ssOut << "BamTestPubNode('" << szUniqueNodeName << "," << szTopicName << "') has been created.";
  RCLCPP_INFO(this->get_logger(), ssOut.str().c_str());
  spdlog::get("main")->info(ssOut.str());
}


//=======================================================================================
//=======================================================================================
BamTestPubNode::~BamTestPubNode()
{
  std::ostringstream ssOut;
  ssOut << "BamTestPubNode('" << this->get_name() << "," << m_strTopicName << "') has been destroyed.";
  RCLCPP_INFO(this->get_logger(), ssOut.str().c_str());
}

//=======================================================================================
//=======================================================================================
void BamTestPubNode::publishPose(void)
{
  auto msg = geometry_msgs::msg::PoseStamped();

  // Determine new position and rotation
  double pos[3] = { 0.0, 0.0, -2.5};  // NED
  double yaw = 0.0;
  getUpdatedPose(pos[0], pos[1], yaw);

  double q[4] = {1.0,0.0,0.0,0.0};
  //toQuaternion(0.0,0.0,yaw,q);    // Something is busted in here according to debug prints.  disabling since it doesn't matter much for now.

  // Set the position in the message
	msg.pose.position.x = pos[0];
  msg.pose.position.y = pos[1];
  msg.pose.position.z = pos[2];

  // Set the quaternion in the message
  msg.pose.orientation.w = q[0];
  msg.pose.orientation.x = q[1];
  msg.pose.orientation.y = q[2];
  msg.pose.orientation.z = q[3];

  if (m_bDebugOutput)
  {
    auto hrtTime = std::chrono::high_resolution_clock::now();
    auto timeElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(hrtTime - m_hrtLastTimestamp);
    m_dElapsedTimeSinceLastDebugMsg = static_cast<double>(timeElapsedMs.count() / 1000.0);  // convert to seconds from milli

    if (m_dElapsedTimeSinceLastDebugMsg >= 1.0)
    {
      std::ostringstream ssOut;
      ssOut << " POS(x,y,z): " << std::right << std::setw(7) << pos[0]
            << ", " << std::right << std::setw(7) << pos[1]
            << ", " << std::right << std::setw(7) << pos[2];

      ssOut << " ROT(w,x,y,z): " << std::right << std::setw(7) << q[0]
            << ", " << std::right << std::setw(7) << q[1]
            << ", " << std::right << std::setw(7) << q[2]
            << ", " << std::right << std::setw(7) << q[3];

      m_dElapsedTimeSinceLastDebugMsg = 0.0;
      spdlog::get("main")->info(ssOut.str());
    }

    m_hrtLastTimestamp = hrtTime;
  }
    
  m_pPublisher->publish(msg);
  
  m_uiDebugCount++;
  if ((m_uiDebugCount % 1000) == 0)
  {
    std::ostringstream ssOut;
    ssOut << "Published " << m_uiDebugCount << " Pose records.";
    spdlog::get("main")->info(ssOut.str());
  }
}


//=======================================================================================
//=======================================================================================
void BamTestPubNode::getUpdatedPose(double& x, double& y, double& yaw)
{

  x = m_dRadius * sin(m_dTheta);
  y = m_dRadius * cos(m_dTheta);

  yaw = tan( y / x) + (PI/2.0);

  // Advance theta for next turn
  double tDelta = 1 / m_dFrequency;
  double dAdvance = tDelta * m_dTurnRate;
  m_dTheta = adjustHeading(m_dTheta + dAdvance);
}


//=======================================================================================
// This is not in game format, it is in mathematical format.
// From Wikipedia's article on Quaternions. -- https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
//=======================================================================================
void BamTestPubNode::toQuaternion(const double& dRoll, const double& dPitch, const double& dYaw, double dQuatOut[4])
{
  // Quaternion Order is w, x, y, z

  // Abbreviations for the various angular functions
  double cr = cos(dRoll * 0.5);
  double sr = sin(dRoll * 0.5);
  double cp = cos(dPitch * 0.5);
  double sp = sin(dPitch * 0.5);
  double cy = cos(dYaw * 0.5);
  double sy = sin(dYaw * 0.5);

  dQuatOut[0] = cr * cp * cy + sr * sp * sy;
  dQuatOut[1] = sr * cp * cy - cr * sp * sy;
  dQuatOut[2] = cr * sp * cy + sr * cp * sy;
  dQuatOut[3] = cr * cp * sy - sr * sp * cy;
}

//=======================================================================================
//=======================================================================================
double BamTestPubNode::adjustHeading(double dYawInRads)
{
    const double tworad = 2*PI;
    double dYawOut = dYawInRads;
    while(dYawOut >= tworad)
      dYawOut -= tworad;
      
    while(dYawOut < 0)
      dYawOut += tworad;

    return dYawOut;
}