// Generated by gencpp from file pharos_vlp_tilt/state.msg
// DO NOT EDIT!


#ifndef PHAROS_VLP_TILT_MESSAGE_STATE_H
#define PHAROS_VLP_TILT_MESSAGE_STATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pharos_vlp_tilt
{
template <class ContainerAllocator>
struct state_
{
  typedef state_<ContainerAllocator> Type;

  state_()
    : is_ground(0)
    , is_del(0)
    , is_infect(0)  {
    }
  state_(const ContainerAllocator& _alloc)
    : is_ground(0)
    , is_del(0)
    , is_infect(0)  {
  (void)_alloc;
    }



   typedef int32_t _is_ground_type;
  _is_ground_type is_ground;

   typedef int32_t _is_del_type;
  _is_del_type is_del;

   typedef int32_t _is_infect_type;
  _is_infect_type is_infect;





  typedef boost::shared_ptr< ::pharos_vlp_tilt::state_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pharos_vlp_tilt::state_<ContainerAllocator> const> ConstPtr;

}; // struct state_

typedef ::pharos_vlp_tilt::state_<std::allocator<void> > state;

typedef boost::shared_ptr< ::pharos_vlp_tilt::state > statePtr;
typedef boost::shared_ptr< ::pharos_vlp_tilt::state const> stateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pharos_vlp_tilt::state_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pharos_vlp_tilt::state_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pharos_vlp_tilt

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'pharos_vlp_tilt': ['/home/jwkolab/pp_ws/src/pharos/pharos_vlp_tilt/msg'], 'pharos_msgs': ['/home/jwkolab/pp_ws/src/pharos/pharos_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pharos_vlp_tilt::state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pharos_vlp_tilt::state_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::state_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::state_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pharos_vlp_tilt::state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4dde323cb233595cceb5a9451b77b1b4";
  }

  static const char* value(const ::pharos_vlp_tilt::state_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4dde323cb233595cULL;
  static const uint64_t static_value2 = 0xceb5a9451b77b1b4ULL;
};

template<class ContainerAllocator>
struct DataType< ::pharos_vlp_tilt::state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pharos_vlp_tilt/state";
  }

  static const char* value(const ::pharos_vlp_tilt::state_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pharos_vlp_tilt::state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 is_ground\n\
int32 is_del\n\
int32 is_infect\n\
";
  }

  static const char* value(const ::pharos_vlp_tilt::state_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pharos_vlp_tilt::state_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_ground);
      stream.next(m.is_del);
      stream.next(m.is_infect);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct state_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pharos_vlp_tilt::state_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pharos_vlp_tilt::state_<ContainerAllocator>& v)
  {
    s << indent << "is_ground: ";
    Printer<int32_t>::stream(s, indent + "  ", v.is_ground);
    s << indent << "is_del: ";
    Printer<int32_t>::stream(s, indent + "  ", v.is_del);
    s << indent << "is_infect: ";
    Printer<int32_t>::stream(s, indent + "  ", v.is_infect);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHAROS_VLP_TILT_MESSAGE_STATE_H
