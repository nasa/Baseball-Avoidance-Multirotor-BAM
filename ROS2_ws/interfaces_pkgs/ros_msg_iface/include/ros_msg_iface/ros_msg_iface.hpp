/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: ros_msg_iface.hpp                                                         *
 *                                                                                      *
 *  Description: ros_msg_iface Declaration                                    *
 *                                                                                      *
 ****************************************************************************************/

#ifndef ROS_MSG_IFACE_HPP
#define ROS_MSG_IFACE_HPP

#include <string>

namespace ros_msg_iface
{
  void start (const std::string& strNodeName, const std::string& strTopicName);
  void outputs (
                // inputs
                const double* Pos_bii_in,
                const double* Q_i2b_in,
                // outputs
                double* y
              );
  void terminate (void);
}

#endif
