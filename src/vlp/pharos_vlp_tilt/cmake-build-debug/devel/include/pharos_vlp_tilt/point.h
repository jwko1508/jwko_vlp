// Generated by gencpp from file pharos_vlp_tilt/point.msg
// DO NOT EDIT!


#ifndef PHAROS_VLP_TILT_MESSAGE_POINT_H
#define PHAROS_VLP_TILT_MESSAGE_POINT_H


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
struct point_
{
  typedef point_<ContainerAllocator> Type;

  point_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , intensity(0.0)  {
    }
  point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , intensity(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _intensity_type;
  _intensity_type intensity;





  typedef boost::shared_ptr< ::pharos_vlp_tilt::point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pharos_vlp_tilt::point_<ContainerAllocator> const> ConstPtr;

}; // struct point_

typedef ::pharos_vlp_tilt::point_<std::allocator<void> > point;

typedef boost::shared_ptr< ::pharos_vlp_tilt::point > pointPtr;
typedef boost::shared_ptr< ::pharos_vlp_tilt::point const> pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pharos_vlp_tilt::point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pharos_vlp_tilt::point_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pharos_vlp_tilt::point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pharos_vlp_tilt::point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pharos_vlp_tilt::point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pharos_vlp_tilt::point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pharos_vlp_tilt::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "00ee07b5674ef45ef5f2f55dbd93bbe7";
  }

  static const char* value(const ::pharos_vlp_tilt::point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x00ee07b5674ef45eULL;
  static const uint64_t static_value2 = 0xf5f2f55dbd93bbe7ULL;
};

template<class ContainerAllocator>
struct DataType< ::pharos_vlp_tilt::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pharos_vlp_tilt/point";
  }

  static const char* value(const ::pharos_vlp_tilt::point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pharos_vlp_tilt::point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n\
float64 y\n\
float64 z\n\
float64 intensity\n\
";
  }

  static const char* value(const ::pharos_vlp_tilt::point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pharos_vlp_tilt::point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.intensity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pharos_vlp_tilt::point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pharos_vlp_tilt::point_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "intensity: ";
    Printer<double>::stream(s, indent + "  ", v.intensity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHAROS_VLP_TILT_MESSAGE_POINT_H
