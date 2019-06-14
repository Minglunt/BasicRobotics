// Generated by gencpp from file cartesian_control/CartesianCommand.msg
// DO NOT EDIT!


#ifndef CARTESIAN_CONTROL_MESSAGE_CARTESIANCOMMAND_H
#define CARTESIAN_CONTROL_MESSAGE_CARTESIANCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Transform.h>

namespace cartesian_control
{
template <class ContainerAllocator>
struct CartesianCommand_
{
  typedef CartesianCommand_<ContainerAllocator> Type;

  CartesianCommand_()
    : x_target()
    , secondary_objective(false)
    , q0_target(0.0)  {
    }
  CartesianCommand_(const ContainerAllocator& _alloc)
    : x_target(_alloc)
    , secondary_objective(false)
    , q0_target(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _x_target_type;
  _x_target_type x_target;

   typedef uint8_t _secondary_objective_type;
  _secondary_objective_type secondary_objective;

   typedef float _q0_target_type;
  _q0_target_type q0_target;





  typedef boost::shared_ptr< ::cartesian_control::CartesianCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartesian_control::CartesianCommand_<ContainerAllocator> const> ConstPtr;

}; // struct CartesianCommand_

typedef ::cartesian_control::CartesianCommand_<std::allocator<void> > CartesianCommand;

typedef boost::shared_ptr< ::cartesian_control::CartesianCommand > CartesianCommandPtr;
typedef boost::shared_ptr< ::cartesian_control::CartesianCommand const> CartesianCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartesian_control::CartesianCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartesian_control::CartesianCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cartesian_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'cartesian_control': ['/home/minglun/ros_wkspace_asgn3/src/assignment3/cartesian_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_control::CartesianCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_control::CartesianCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_control::CartesianCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5ffe3cb2eac6f1acc27e5635f953be7";
  }

  static const char* value(const ::cartesian_control::CartesianCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5ffe3cb2eac6f1aULL;
  static const uint64_t static_value2 = 0xcc27e5635f953be7ULL;
};

template<class ContainerAllocator>
struct DataType< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartesian_control/CartesianCommand";
  }

  static const char* value(const ::cartesian_control::CartesianCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Transform x_target\n\
bool secondary_objective\n\
float32 q0_target\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::cartesian_control::CartesianCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x_target);
      stream.next(m.secondary_objective);
      stream.next(m.q0_target);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CartesianCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartesian_control::CartesianCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartesian_control::CartesianCommand_<ContainerAllocator>& v)
  {
    s << indent << "x_target: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.x_target);
    s << indent << "secondary_objective: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.secondary_objective);
    s << indent << "q0_target: ";
    Printer<float>::stream(s, indent + "  ", v.q0_target);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTESIAN_CONTROL_MESSAGE_CARTESIANCOMMAND_H
