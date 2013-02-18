#include <wbc_lwr/boost/RobotState.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::DataSource< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::AssignCommand< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::ValueDataSource< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::OutputPort< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::InputPort< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::Property< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::Attribute< wbc_lwr::RobotState >;
template class RTT_EXPORT RTT::Constant< wbc_lwr::RobotState >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_wbc_lwr_RobotState() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<wbc_lwr::RobotState>("/wbc_lwr/RobotState") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<wbc_lwr::RobotState> >("/wbc_lwr/RobotState[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<wbc_lwr::RobotState> >("/wbc_lwr/cRobotState[]") );
        }

    
}

