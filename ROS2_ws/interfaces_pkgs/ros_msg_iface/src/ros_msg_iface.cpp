/****************************************************************************************
 *  NOTICE:                                                                             *
 *                                                                                      *
 *  THIS SOFTWARE MAY BE USED, COPIED, AND PROVIDED TO OTHERS ONLY AS PERMITTED UNDER   *
 *  THE TERMS OF THE CONTRACT OR OTHER AGREEMENT UNDER WHICH IT WAS ACQUIRED FROM THE   *
 *  U.S. GOVERNMENT.  NEITHER TITLE TO NOR OWNERSHIP OF THIS SOFTWARE IS HEREBY         *
 *  TRANSFERRED.  THIS NOTICE SHALL REMAIN ON ALL COPIES OF THE SOFTWARE.  SOURCE       *
 *  AGENCY: NASA LANGLEY RESEARCH CENTER, HAMPTON, VA 23681.                            *
 *                                                                                      *
 *  File: ros_msg_iface.cpp                                                         *
 *                                                                                      *
 *  Description: ros_msg_iface Class Implementation                                 *
 *                                                                                      *
 ****************************************************************************************/

#include <iostream>
#include <sstream>

#include <cmath>

// ros includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // DESIRED RO2 MSG TYPE HERE

#include "ros_msg_iface/ros_msg_iface.hpp"


//=======================================================================================
// MACRO DEFINITIONS
//=======================================================================================
#define DEBUG_FLAG 1


namespace ros_msg_iface
{

  class PubPoseNode : public rclcpp::Node // MODIFY CLASS NAME
  {
  public:
    PubPoseNode (const std::string& strNodeName, const std::string& strTopicName) : Node(strNodeName) // MODIFY NODE NAME
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(strTopicName, 10); //TOPIC NAME HERE
      RCLCPP_INFO(this->get_logger(), "Publish Pose Node Station has been started.");
      if (DEBUG_FLAG)
      {
        std::cout << "Constructor for "
                  << strNodeName
                  << " node is executed, topic name: " 
                  << strTopicName 
                  << std::endl; 
      }
    }

    ~PubPoseNode ()
    {
      if (DEBUG_FLAG) { std::cout << "Destructor for node is executed\n"; }
    }

    void publishPose (double* pos, double* quat)
    {
      auto msg = geometry_msgs::msg::PoseStamped();
      // Set the position in the message
      msg.pose.position.x = pos[0];
      msg.pose.position.y = pos[1];
      msg.pose.position.z = pos[2];
      // Set the quaternion in the message
      msg.pose.orientation.x = quat[0];
      msg.pose.orientation.y = quat[1];
      msg.pose.orientation.z = quat[2];
      msg.pose.orientation.w = quat[3];

      publisher_->publish(msg);
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_; //create publisher
  };

  PubPoseNode* pPoseNode = 0;


  //=======================================================================================
  //=======================================================================================
  void start (const std::string& strNodeName, const std::string& strTopicName)
  {
    if (!rclcpp::ok())
    {
      std::cout << "Initializing rclcpp" << std::endl;
      rclcpp::init(0,nullptr); // Initialize ROS2
    }
    else
    {
      std::cout << "rclcpp already initialized" << std::endl;
    }
    if (DEBUG_FLAG) { std::cout << "Node is being created..\n"; }
    pPoseNode = new PubPoseNode(strNodeName, strTopicName); // creates the new object

    if (DEBUG_FLAG) { std::cout << "Exiting mdlStart...\n"; }
  }


  //=======================================================================================
  //=======================================================================================
  void outputs ( 
                // inputs
                const double* Pos_bii_in,
                const double* Q_i2b_in,
                // outputs
                double* y
               )
  {
    double dPosBii[3] = { 0.0, 0.0, 0.0 };
    double dQi2b[4] = { 0.0, 0.0, 0.0, 0.0};

    // copy input data to local variables
    memcpy(dPosBii, Pos_bii_in, sizeof(double) * 3);
    memcpy(dQi2b, Q_i2b_in, sizeof(double) * 4);

    // Output the values...
    y[0] = dPosBii[0]; // set the first position output 
    y[1] = dPosBii[1]; // set the second position output
    y[2] = dPosBii[2]; // set the third position output
    y[3] = dQi2b[0]; // set the first quaternion output
    y[4] = dQi2b[1]; // set the second quaternion output
    y[5] = dQi2b[2]; // set the third quaternion output
    y[6] = dQi2b[3]; // set the fourth quaternion output

    if (DEBUG_FLAG) 
    {
//      std::cout << "pos = [" << y[0] << ", " << y[1] << ", " << y[2] << "]" << std::endl;
//      std::cout << "quat = [" << y[3] << ", " << y[4] << ", " << y[5] << ", " << y[6] << "]" << std::endl;

    }

    if (pPoseNode != 0)
    {
//      std::cout << "Publishing node..." << std::endl;
      pPoseNode->publishPose(dPosBii, dQi2b);
    }
  }


  //=======================================================================================
  //=======================================================================================
  void terminate (void)
  {
    if (DEBUG_FLAG) { std::cout << "Starting model terminate..\n"; }
    
    if (DEBUG_FLAG) { std::cout << "Deleting Node..\n"; }
    //delete node; // Had to comment this out on windows...
    if (pPoseNode != 0)
    {
      delete pPoseNode;
      pPoseNode = 0;
    }

    if (DEBUG_FLAG) { std::cout << "Shutting Down rclcpp..\n"; }

    rclcpp::shutdown(); // Terminate ROS2

    if (DEBUG_FLAG) { std::cout << "Exiting model terminate..\n"; }
  }
}