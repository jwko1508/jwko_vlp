// Generated by gencpp from file pharos_vlp_tilt/perfect.msg
// DO NOT EDIT!


#ifndef PHAROS_VLP_TILT_MESSAGE_PERFECT_H
#define PHAROS_VLP_TILT_MESSAGE_PERFECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pharos_vlp_tilt/info.h>
#include <pharos_vlp_tilt/point.h>
#include <pharos_vlp_tilt/state.h>

namespace pharos_vlp_tilt
{
template <class ContainerAllocator>
struct perfect_
{
  typedef perfect_<ContainerAllocator> Type;

  perfect_()
    : info()
    , point()
    , state()  {
    }
  perfect_(const ContainerAllocator& _alloc)
    : info(_alloc)
    , point(_alloc)
    , state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::pharos_vlp_tilt::info_<ContainerAllocator>  _info_type;
  _info_type info;

   typedef  ::pharos_vlp_tilt::point_<ContainerAllocator>  _point_type;
  _point_type point;

   typedef  ::pharos_vlp_tilt::state_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::pharos_vlp_tilt::perfect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pharos_vlp_tilt::perfect_<ContainerAllocator> const> ConstPtr;

}; // struct perfect_

typedef ::pharos_vlp_tilt::perfect_<std::allocator<void> > perfect;

typedef boost::shared_ptr< ::pharos_vlp_tilt::perfect > perfectPtr;
typedef boost::shared_ptr< ::pharos_vlp_tilt::perfect const> perfectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pharos_vlp_tilt::perfect_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pharos_vlp_tilt::perfect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::perfect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::perfect_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "90a56c4e8308c1352b958efc8367b00b";
  }

  static const char* value(const ::pharos_vlp_tilt::perfect_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x90a56c4e8308c135ULL;
  static const uint64_t static_value2 = 0x2b958efc8367b00bULL;
};

template<class ContainerAllocator>
struct DataType< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pharos_vlp_tilt/perfect";
  }

  static const char* value(const ::pharos_vlp_tilt::perfect_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "info info\n\
point point\n\
state state\n\
\n\
================================================================================\n\
MSG: pharos_vlp_tilt/info\n\
int32 laser\n\
int32 hori\n\
\n\
================================================================================\n\
MSG: pharos_vlp_tilt/point\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 intensity\n\
\n\
================================================================================\n\
MSG: pharos_vlp_tilt/state\n\
int32 is_ground\n\
int32 is_del\n\
int32 is_infect\n\
";
  }

  static const char* value(const ::pharos_vlp_tilt::perfect_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.info);
      stream.next(m.point);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct perfect_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pharos_vlp_tilt::perfect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pharos_vlp_tilt::perfect_<ContainerAllocator>& v)
  {
    s << indent << "info: ";
    s << std::endl;
    Printer< ::pharos_vlp_tilt::info_<ContainerAllocator> >::stream(s, indent + "  ", v.info);
    s << indent << "point: ";
    s << std::endl;
    Printer< ::pharos_vlp_tilt::point_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
    s << indent << "state: ";
    s << std::endl;
    Printer< ::pharos_vlp_tilt::state_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHAROS_VLP_TILT_MESSAGE_PERFECT_H
