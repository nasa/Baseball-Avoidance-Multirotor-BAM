// This executable is used to subscribe to the specified ros2 pose msg
// which is then supplied to airsim (on ultimately on to unreal engine). 
// It makes use of the built in ros2 message type: 

// ***************************************
// geometry_msgs/msg/PoseStamped
// std_msgs/Header header
//        builtin_interfaces/Time stamp
//                int32 sec
//                uint32 nanosec
//        string frame_id
// Pose pose
//        Point position
//                float64 x
//                float64 y
//                float64 z
//        Quaternion orientation
//                float64 x 0
//                float64 y 0
//                float64 z 0
//                float64 w 1
// **************************************

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <chrono>
#include <iostream>

// ************ USERS SHOULD WRITE THE FOLLOWING DECLARATIONS **************************
#define node_classnm AirSimNode  // Node class name can't be in quotes
#define node_name "airsimnode"       // Node name must be in quotes
#define topic_name "pub_pose"      // Topic name must be in quotes
#define mess_fname geometry_msgs::msg::PoseStamped
#define DEBUG_FLAG 1 // Non-zero implies debug => output msgs to stdout

#include "geometry_msgs/msg/pose_stamped.hpp" // DESIRED RO2 MSG TYPE HERE
#include "rclcpp/rclcpp.hpp"

constexpr int NWIDTH = 7;

using namespace msr::airlib;

msr::airlib::MultirotorRpcLibClient client;

class node_classnm : public rclcpp::Node // MODIFY CLASS NAME
{
    public:
        node_classnm() : Node(node_name) // Node Creator
        {
        subscriber_ = this->create_subscription<mess_fname>(topic_name, 10, std::bind(&node_classnm::updatePose,this, std::placeholders::_1)); //the placeholders are for arguements to pass
            RCLCPP_INFO(this->get_logger(), "Air Sim Node has been created.");
        }
        ~node_classnm() // Node Destructor
        {
            RCLCPP_INFO(this->get_logger(), "Air Sim Node has been destroyed.");
        }
        
    private:
        void updatePose(const mess_fname::SharedPtr msg)
        {
            // msg = geometry_msgs::msg::PoseStamped();
            std::cout << std::fixed;
            std::cout << std::setprecision(3);
            // static int count = 0;

            auto x = msg->pose.position.x;
            auto y = msg->pose.position.y;
            auto z = msg->pose.position.z;
            auto ow = msg->pose.orientation.x;
            auto ox = msg->pose.orientation.y;
            auto oy = msg->pose.orientation.z;
            auto oz = msg->pose.orientation.w;

            // Output message within the updatePose function if DEBUG_FLAG was set
            if (DEBUG_FLAG){
                // std::cout << "local (" << std::setw(2) << i << ") ";
                // std::cout << std::left << std::setw(32) << msg->pose(i).name();
                std::cout << " x: " << std::right << std::setw(NWIDTH) << x;
                std::cout << " y: " << std::right << std::setw(NWIDTH) << y;
                std::cout << " z: " << std::right << std::setw(NWIDTH) << z;

                std::cout << " ow: " << std::right << std::setw(NWIDTH) << ow;
                std::cout << " ox: " << std::right << std::setw(NWIDTH) << ox;
                std::cout << " oy: " << std::right << std::setw(NWIDTH) << oy;
                std::cout << " oz: " << std::right << std::setw(NWIDTH) << oz;
                std::cout << std::endl;
            }

            msr::airlib::Vector3r p(x, -y, -z); // What frame is expected by AirSim?
            msr::airlib::Quaternionr o(ow, ox, -oy, -oz);  // What frame is expected by AirSim?

            client.simSetVehiclePose(Pose(p, o), true); // Publish pose to Airsim
        }
        rclcpp::Subscription<mess_fname>::SharedPtr subscriber_;
            
};



    int main(int _argc, char** _argv)
{
    // Initialize and connect to airsim..
    // client = airsim.MultirotorClient()
    rclcpp::init(argc, argv); // Initialize ros2
    client.confirmConnection();
    client.enableApiControl(true);
    client.armDisarm(true);

    auto node = std::make_shared<node_classnm>(); // User defined node class name
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    client.armDisarm(false);
    // How do I terminate airsim? And/or is it even required?
    return 0;
}