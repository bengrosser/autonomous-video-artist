// Generated by gencpp from file im_msgs/BumperState.msg
// DO NOT EDIT!


#ifndef IM_MSGS_MESSAGE_BUMPERSTATE_H
#define IM_MSGS_MESSAGE_BUMPERSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace im_msgs
{
template <class ContainerAllocator>
struct BumperState_
{
  typedef BumperState_<ContainerAllocator> Type;

  BumperState_()
    : bumper_state(false)  {
    }
  BumperState_(const ContainerAllocator& _alloc)
    : bumper_state(false)  {
  (void)_alloc;
    }



   typedef uint8_t _bumper_state_type;
  _bumper_state_type bumper_state;




  typedef boost::shared_ptr< ::im_msgs::BumperState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::im_msgs::BumperState_<ContainerAllocator> const> ConstPtr;

}; // struct BumperState_

typedef ::im_msgs::BumperState_<std::allocator<void> > BumperState;

typedef boost::shared_ptr< ::im_msgs::BumperState > BumperStatePtr;
typedef boost::shared_ptr< ::im_msgs::BumperState const> BumperStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::im_msgs::BumperState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::im_msgs::BumperState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace im_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'im_msgs': ['/home/zhang/catkin_ws/src/im_msgs/msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/indigo/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/indigo/share/visualization_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::im_msgs::BumperState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::im_msgs::BumperState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::im_msgs::BumperState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::im_msgs::BumperState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::im_msgs::BumperState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::im_msgs::BumperState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::im_msgs::BumperState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "46e92a89f2364b7e5f107580d7840ab7";
  }

  static const char* value(const ::im_msgs::BumperState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x46e92a89f2364b7eULL;
  static const uint64_t static_value2 = 0x5f107580d7840ab7ULL;
};

template<class ContainerAllocator>
struct DataType< ::im_msgs::BumperState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "im_msgs/BumperState";
  }

  static const char* value(const ::im_msgs::BumperState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::im_msgs::BumperState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool bumper_state\n\
";
  }

  static const char* value(const ::im_msgs::BumperState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::im_msgs::BumperState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bumper_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BumperState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::im_msgs::BumperState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::im_msgs::BumperState_<ContainerAllocator>& v)
  {
    s << indent << "bumper_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bumper_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IM_MSGS_MESSAGE_BUMPERSTATE_H