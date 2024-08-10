// Generated by gencpp from file gazebo_grasp_plugin_ros/GazeboGraspEvent.msg
// DO NOT EDIT!


#ifndef GAZEBO_GRASP_PLUGIN_ROS_MESSAGE_GAZEBOGRASPEVENT_H
#define GAZEBO_GRASP_PLUGIN_ROS_MESSAGE_GAZEBOGRASPEVENT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gazebo_grasp_plugin_ros
{
template <class ContainerAllocator>
struct GazeboGraspEvent_
{
  typedef GazeboGraspEvent_<ContainerAllocator> Type;

  GazeboGraspEvent_()
    : arm()
    , object()
    , attached(false)  {
    }
  GazeboGraspEvent_(const ContainerAllocator& _alloc)
    : arm(_alloc)
    , object(_alloc)
    , attached(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _arm_type;
  _arm_type arm;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _object_type;
  _object_type object;

   typedef uint8_t _attached_type;
  _attached_type attached;





  typedef boost::shared_ptr< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> const> ConstPtr;

}; // struct GazeboGraspEvent_

typedef ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<std::allocator<void> > GazeboGraspEvent;

typedef boost::shared_ptr< ::gazebo_grasp_plugin_ros::GazeboGraspEvent > GazeboGraspEventPtr;
typedef boost::shared_ptr< ::gazebo_grasp_plugin_ros::GazeboGraspEvent const> GazeboGraspEventConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator1> & lhs, const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator2> & rhs)
{
  return lhs.arm == rhs.arm &&
    lhs.object == rhs.object &&
    lhs.attached == rhs.attached;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator1> & lhs, const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gazebo_grasp_plugin_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a5b6c6f554465c6bcbcad9409a41137a";
  }

  static const char* value(const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa5b6c6f554465c6bULL;
  static const uint64_t static_value2 = 0xcbcad9409a41137aULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_grasp_plugin_ros/GazeboGraspEvent";
  }

  static const char* value(const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Message to notify about grasp events in gazebo\n"
"\n"
"# name of grasping arm/gripper \n"
"string arm\n"
"\n"
"# collision shape name of grasped object\n"
"string object\n"
"\n"
"# detached if false\n"
"bool attached\n"
;
  }

  static const char* value(const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.arm);
      stream.next(m.object);
      stream.next(m.attached);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GazeboGraspEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_grasp_plugin_ros::GazeboGraspEvent_<ContainerAllocator>& v)
  {
    s << indent << "arm: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.arm);
    s << indent << "object: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.object);
    s << indent << "attached: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.attached);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_GRASP_PLUGIN_ROS_MESSAGE_GAZEBOGRASPEVENT_H
