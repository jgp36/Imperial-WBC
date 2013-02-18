#include <wbc_lwr/RobotState.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_wbc_lwr_RobotState();

   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROSwbc_lwrTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("ros-")+"wbc_lwr";
      }

      virtual bool loadTypes() {
          // call all factory functions
                  rtt_ros_addType_wbc_lwr_RobotState(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSwbc_lwrTypekitPlugin )

