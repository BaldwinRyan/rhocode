/* Auto-generated by genmsg_cpp for file /home/noam/ros_workspace/PanTilt/DPPanTilt_msgs/msg/SetAbsolutePosition.msg */
#ifndef DPPANTILT_MSGS_MESSAGE_SETABSOLUTEPOSITION_H
#define DPPANTILT_MSGS_MESSAGE_SETABSOLUTEPOSITION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace DPPanTilt_msgs
{
template <class ContainerAllocator>
struct SetAbsolutePosition_ {
  typedef SetAbsolutePosition_<ContainerAllocator> Type;

  SetAbsolutePosition_()
  : panPosition(0.0)
  , tiltPosition(0.0)
  {
  }

  SetAbsolutePosition_(const ContainerAllocator& _alloc)
  : panPosition(0.0)
  , tiltPosition(0.0)
  {
  }

  typedef float _panPosition_type;
  float panPosition;

  typedef float _tiltPosition_type;
  float tiltPosition;


private:
  static const char* __s_getDataType_() { return "DPPanTilt_msgs/SetAbsolutePosition"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "06c39833eb74fde9834f06e8b2303d18"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 panPosition\n\
float32 tiltPosition\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, panPosition);
    ros::serialization::serialize(stream, tiltPosition);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, panPosition);
    ros::serialization::deserialize(stream, tiltPosition);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(panPosition);
    size += ros::serialization::serializationLength(tiltPosition);
    return size;
  }

  typedef boost::shared_ptr< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetAbsolutePosition
typedef  ::DPPanTilt_msgs::SetAbsolutePosition_<std::allocator<void> > SetAbsolutePosition;

typedef boost::shared_ptr< ::DPPanTilt_msgs::SetAbsolutePosition> SetAbsolutePositionPtr;
typedef boost::shared_ptr< ::DPPanTilt_msgs::SetAbsolutePosition const> SetAbsolutePositionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace DPPanTilt_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "06c39833eb74fde9834f06e8b2303d18";
  }

  static const char* value(const  ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x06c39833eb74fde9ULL;
  static const uint64_t static_value2 = 0x834f06e8b2303d18ULL;
};

template<class ContainerAllocator>
struct DataType< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "DPPanTilt_msgs/SetAbsolutePosition";
  }

  static const char* value(const  ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 panPosition\n\
float32 tiltPosition\n\
\n\
";
  }

  static const char* value(const  ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.panPosition);
    stream.next(m.tiltPosition);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetAbsolutePosition_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::DPPanTilt_msgs::SetAbsolutePosition_<ContainerAllocator> & v) 
  {
    s << indent << "panPosition: ";
    Printer<float>::stream(s, indent + "  ", v.panPosition);
    s << indent << "tiltPosition: ";
    Printer<float>::stream(s, indent + "  ", v.tiltPosition);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DPPANTILT_MSGS_MESSAGE_SETABSOLUTEPOSITION_H

