/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: BamAirsimNode.hpp                                                             *
 *                                                                                      *
 *  Description: BamAirsimNode Class Declaration                                        *
 *                                                                                      *
 ****************************************************************************************/

#ifndef BAMAIRSIMNODE_HPP
#define BAMAIRSIMNODE_HPP

#include <typeinfo>
#include <iomanip>
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>  // DESIRED RO2 MSG TYPE
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rclcpp/rclcpp.hpp"    // Access to rclcpp::node

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"   // Airsim stuff
STRICT_MODE_ON


#define pose_msg_t geometry_msgs::msg::PoseStamped
#define highres_time  std::chrono::time_point<std::chrono::high_resolution_clock>

//=======================================================================================
// The enum represents the two type of actors / objects that the airsim client knows
// how to maniplate for us in Unreal, and we will be leveraging both depending on whether
// we are reading in vehicle data or another object like the baseball.
//=======================================================================================
enum ActorType
{
  AT_VEHICLE = 0,
  AT_OBJECT = 1,
  AT_COUNT = 2
};


//=======================================================================================
// BamAirsimNode class
// This class creates a node that will subscribe to a topic of choosing that is being populated
// (at least as intended) by ROS elsewhere.  
// Dev Note: Because I'm templating this class, all of the definitions will be placed below
// Dev Note: I was having some issues with the std::bind function in a base parent class ctor
//           so I decided to abandon that implemenation and just go with a all encompassing
//           class.
//=======================================================================================
class BamAirsimNode : public rclcpp::Node
{
public:

  // Constructor and Destructor
  BamAirsimNode (const char* szActorName, const ActorType eType, 
                 const char* szUniqueNodeName, const char* szTopicName,
                 msr::airlib::MultirotorRpcLibClient* pAirsimClient,
                 bool bDebugOutput = false);
  virtual ~BamAirsimNode (void);

  // Main Functions 
  // Child classes must implement this for whatever function type they are chosing to 
  // set up node access.
  void updateMsg(const std::shared_ptr<pose_msg_t> msg);

  // Accessors
  inline const ActorType&  getActorType(void) const;
  inline const std::string& getActorName(void) const;
  inline const std::string& getNodeName(void) const;
  inline const std::string& getTopicName(void) const;


protected:
  ActorType     m_eType;          // Whether Airsim will treat it as a vehicle or object actor
  std::string   m_strActorName;   // Name of the actor instance in Unreal
  std::string   m_strNodeName;    // Unique Node name that was used
  std::string   m_strTopicName;   // Topic name in ROS that is being subscribed to.

  msr::airlib::MultirotorRpcLibClient*           m_pClient;
  rclcpp::Subscription<pose_msg_t>::SharedPtr    m_pSubscriber;

  // options
  bool           m_bDebugOutput;
  double         m_dElapsedTimeSinceLastDebugMsg;
  highres_time   m_hrtLastTimestamp;
  int            m_uiDebugCount;

private:
};


//***************************************************************************************
//=======================================================================================
//  INLINE FUNCTIONS
//=======================================================================================
//***************************************************************************************


//==============================================================================
//==============================================================================
inline const ActorType& BamAirsimNode::getActorType(void) const
{
  return m_eType;
}


//==============================================================================
//==============================================================================
inline const std::string& BamAirsimNode::getActorName(void) const
{
  return m_strActorName;
}

//==============================================================================
//==============================================================================
inline const std::string& BamAirsimNode::getNodeName(void) const
{
  return m_strNodeName;
}

//==============================================================================
//==============================================================================
inline const std::string& BamAirsimNode::getTopicName(void) const
{
  return m_strTopicName;
}

#endif //BAMAIRSIMNODE_HPP