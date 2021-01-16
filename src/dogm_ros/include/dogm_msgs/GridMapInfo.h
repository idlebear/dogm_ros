// Generated by gencpp from file dogm_msgs/GridMapInfo.msg
// DO NOT EDIT!


#ifndef DOGM_MSGS_MESSAGE_GRIDMAPINFO_H
#define DOGM_MSGS_MESSAGE_GRIDMAPINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace dogm_msgs
{
template <class ContainerAllocator>
struct GridMapInfo_
{
  typedef GridMapInfo_<ContainerAllocator> Type;

  GridMapInfo_()
    : resolution(0.0)
    , length(0.0)
    , size(0)
    , pose()  {
    }
  GridMapInfo_(const ContainerAllocator& _alloc)
    : resolution(0.0)
    , length(0.0)
    , size(0)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef float _resolution_type;
  _resolution_type resolution;

   typedef float _length_type;
  _length_type length;

   typedef int32_t _size_type;
  _size_type size;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::dogm_msgs::GridMapInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dogm_msgs::GridMapInfo_<ContainerAllocator> const> ConstPtr;

}; // struct GridMapInfo_

typedef ::dogm_msgs::GridMapInfo_<std::allocator<void> > GridMapInfo;

typedef boost::shared_ptr< ::dogm_msgs::GridMapInfo > GridMapInfoPtr;
typedef boost::shared_ptr< ::dogm_msgs::GridMapInfo const> GridMapInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dogm_msgs::GridMapInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dogm_msgs::GridMapInfo_<ContainerAllocator1> & lhs, const ::dogm_msgs::GridMapInfo_<ContainerAllocator2> & rhs)
{
  return lhs.resolution == rhs.resolution &&
    lhs.length == rhs.length &&
    lhs.size == rhs.size &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dogm_msgs::GridMapInfo_<ContainerAllocator1> & lhs, const ::dogm_msgs::GridMapInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dogm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dogm_msgs::GridMapInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dogm_msgs::GridMapInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dogm_msgs::GridMapInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e187ade6551b6a67abc91fee215b4147";
  }

  static const char* value(const ::dogm_msgs::GridMapInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe187ade6551b6a67ULL;
  static const uint64_t static_value2 = 0xabc91fee215b4147ULL;
};

template<class ContainerAllocator>
struct DataType< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dogm_msgs/GridMapInfo";
  }

  static const char* value(const ::dogm_msgs::GridMapInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# The map resolution [m/cell]\n"
"float32 resolution\n"
"\n"
"# Map length [m]\n"
"float32 length\n"
"\n"
"# Map size [cells]\n"
"int32 size\n"
"\n"
"# Pose of the grid map center [m]\n"
"geometry_msgs/Pose pose\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::dogm_msgs::GridMapInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.resolution);
      stream.next(m.length);
      stream.next(m.size);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GridMapInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dogm_msgs::GridMapInfo_<ContainerAllocator>& v)
  {
    s << indent << "resolution: ";
    Printer<float>::stream(s, indent + "  ", v.resolution);
    s << indent << "length: ";
    Printer<float>::stream(s, indent + "  ", v.length);
    s << indent << "size: ";
    Printer<int32_t>::stream(s, indent + "  ", v.size);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOGM_MSGS_MESSAGE_GRIDMAPINFO_H
