#include <wbc_lwr/boost/RobotState.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSRobotStatePlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/wbc_lwr/RobotState")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<wbc_lwr::RobotState>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"RobotState";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "RobotState" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSRobotStatePlugin )
