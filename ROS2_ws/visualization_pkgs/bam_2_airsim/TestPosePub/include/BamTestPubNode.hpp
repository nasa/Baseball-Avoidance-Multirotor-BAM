/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: BamTestPubNode.hpp                                                            *
 *                                                                                      *
 *  Description: BamTestPubNode Class Declaration                                       *
 *                                                                                      *
 ****************************************************************************************/

#ifndef BAMTESTPUBNODE_HPP
#define BAMTESTPUBNODE_HPP

#include <typeinfo>
#include <iomanip>
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>  // DESIRED RO2 MSG TYPE
#include "rclcpp/rclcpp.hpp"    // Access to rclcpp::node

#define pose_msg_t geometry_msgs::msg::PoseStamped
#define highres_time  std::chrono::time_point<std::chrono::high_resolution_clock>


//=======================================================================================
// BamAirsimNode class
// This class creates a node that will subscribe to a topic of choosing that is being populated
// (at least as intended) by ROS elsewhere.  
// Dev Note: Because I'm templating this class, all of the definitions will be placed below
// Dev Note: I was having some issues with the std::bind function in a base parent class ctor
//           so I decided to abandon that implemenation and just go with a all encompassing
//           class.
//=======================================================================================
class BamTestPubNode : public rclcpp::Node
{
public:

  // Constructor and Destructor
  BamTestPubNode (const char* szUniqueNodeName, const char* szTopicName, bool bDebugOutput = false);
  virtual ~BamTestPubNode (void);

  // Main Functions 
  // Child classes must implement this for whatever function type they are chosing to 
  // set up node access.
  void publishPose(void);

  // Accessors
  inline const std::string& getNodeName(void) const;
  inline const std::string& getTopicName(void) const;


protected:
  std::string   m_strNodeName;    // Unique Node name that was used
  std::string   m_strTopicName;   // Topic name in ROS that is being subscribed to.

  double        m_dFrequency;     // Rate for the timer.
  double        m_dTheta;         // Current Angle to be published
  double        m_dRadius;        // Radius of the flight in Meters
  double        m_dTurnRate;      // Turn rate of the flight in Rads/s
  
 // key objects to operations
 rclcpp::Publisher<pose_msg_t>::SharedPtr m_pPublisher;
  
 rclcpp::TimerBase::SharedPtr m_pTimerFunc;        // Testing class scope of this var because it's possible keeping it func local is kill the callback

private:

  void getUpdatedPose(double& x, double& y, double& yaw);
  void toQuaternion(const double& dRoll, const double& dPitch, const double& dYaw, double dQuatOut[4]);
  double adjustHeading(double dYawInRads);

  // options
  bool           m_bDebugOutput;
  double         m_dElapsedTimeSinceLastDebugMsg;
  highres_time   m_hrtLastTimestamp;
  int            m_uiDebugCount;
};


//***************************************************************************************
//=======================================================================================
//  INLINE FUNCTIONS
//=======================================================================================
//***************************************************************************************

//==============================================================================
//==============================================================================
inline const std::string& BamTestPubNode::getNodeName(void) const
{
  return m_strNodeName;
}

//==============================================================================
//==============================================================================
inline const std::string& BamTestPubNode::getTopicName(void) const
{
  return m_strTopicName;
}

#endif //BAMTESTPUBNODE_HPP