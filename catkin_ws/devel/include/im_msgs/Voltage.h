// Generated by gencpp from file im_msgs/Voltage.msg
// DO NOT EDIT!


#ifndef IM_MSGS_MESSAGE_VOLTAGE_H
#define IM_MSGS_MESSAGE_VOLTAGE_H


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
struct Voltage_
{
  typedef Voltage_<ContainerAllocator> Type;

  Voltage_()
    : left_motor(0.0)
    , right_motor(0.0)  {
    }
  Voltage_(const ContainerAllocator& _alloc)
    : left_motor(0.0)
    , right_motor(0.0)  {
  (void)_alloc;
    }



   typedef float _left_motor_type;
  _left_motor_type left_motor;

   typedef float _right_motor_type;
  _right_motor_type right_motor;




  typedef boost::shared_ptr< ::im_msgs::Voltage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::im_msgs::Voltage_<ContainerAllocator> const> ConstPtr;

}; // struct Voltage_

typedef ::im_msgs::Voltage_<std::allocator<void> > Voltage;

typedef boost::shared_ptr< ::im_msgs::Voltage > VoltagePtr;
typedef boost::shared_ptr< ::im_msgs::Voltage const> VoltageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::im_msgs::Voltage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::im_msgs::Voltage_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::im_msgs::Voltage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::im_msgs::Voltage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::im_msgs::Voltage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::im_msgs::Voltage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::im_msgs::Voltage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::im_msgs::Voltage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::im_msgs::Voltage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e3717ac8e9443aa62d7102a5860f5e7";
  }

  static const char* value(const ::im_msgs::Voltage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e3717ac8e9443aaULL;
  static const uint64_t static_value2 = 0x62d7102a5860f5e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::im_msgs::Voltage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "im_msgs/Voltage";
  }

  static const char* value(const ::im_msgs::Voltage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::im_msgs::Voltage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 left_motor\n\
float32 right_motor\n\
";
  }

  static const char* value(const ::im_msgs::Voltage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::im_msgs::Voltage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left_motor);
      stream.next(m.right_motor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Voltage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::im_msgs::Voltage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::im_msgs::Voltage_<ContainerAllocator>& v)
  {
    s << indent << "left_motor: ";
    Printer<float>::stream(s, indent + "  ", v.left_motor);
    s << indent << "right_motor: ";
    Printer<float>::stream(s, indent + "  ", v.right_motor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IM_MSGS_MESSAGE_VOLTAGE_H