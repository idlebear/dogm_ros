// Generated by gencpp from file dogm_msgs/DynamicOccupancyGrid.msg
// DO NOT EDIT!


#ifndef DOGM_MSGS_MESSAGE_DYNAMICOCCUPANCYGRID_H
#define DOGM_MSGS_MESSAGE_DYNAMICOCCUPANCYGRID_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <dogm_msgs/GridMapInfo.h>
#include <dogm_msgs/GridCell.h>

namespace dogm_msgs
{
template <class ContainerAllocator>
struct DynamicOccupancyGrid_
{
  typedef DynamicOccupancyGrid_<ContainerAllocator> Type;

  DynamicOccupancyGrid_()
    : header()
    , info()
    , data()  {
    }
  DynamicOccupancyGrid_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , info(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::dogm_msgs::GridMapInfo_<ContainerAllocator>  _info_type;
  _info_type info;

   typedef std::vector< ::dogm_msgs::GridCell_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::dogm_msgs::GridCell_<ContainerAllocator> >::other >  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> const> ConstPtr;

}; // struct DynamicOccupancyGrid_

typedef ::dogm_msgs::DynamicOccupancyGrid_<std::allocator<void> > DynamicOccupancyGrid;

typedef boost::shared_ptr< ::dogm_msgs::DynamicOccupancyGrid > DynamicOccupancyGridPtr;
typedef boost::shared_ptr< ::dogm_msgs::DynamicOccupancyGrid const> DynamicOccupancyGridConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator1> & lhs, const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.info == rhs.info &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator1> & lhs, const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dogm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d9bc7c56d0d67f36e37a85c344793df";
  }

  static const char* value(const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d9bc7c56d0d67f3ULL;
  static const uint64_t static_value2 = 0x6e37a85c344793dfULL;
};

template<class ContainerAllocator>
struct DataType< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dogm_msgs/DynamicOccupancyGrid";
  }

  static const char* value(const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Header (time and frame)\n"
"Header header\n"
"\n"
"# Grid map info\n"
"GridMapInfo info\n"
"\n"
"# Grid map data\n"
"GridCell[] data\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: dogm_msgs/GridMapInfo\n"
"# The map resolution [m/cell]\n"
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
"\n"
"================================================================================\n"
"MSG: dogm_msgs/GridCell\n"
"# Free mass\n"
"float32 free_mass\n"
"\n"
"# Occupied mass\n"
"float32 occ_mass\n"
"\n"
"# Mean velocity in x direction\n"
"float32 mean_x_vel\n"
"\n"
"# Mean velocity in y direction\n"
"float32 mean_y_vel\n"
"\n"
"# Velocity variance in x direction\n"
"float32 var_x_vel\n"
"\n"
"# Velocity variance in y direction\n"
"float32 var_y_vel\n"
"\n"
"# Covariance of x and y\n"
"float32 covar_xy_vel\n"
;
  }

  static const char* value(const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.info);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamicOccupancyGrid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dogm_msgs::DynamicOccupancyGrid_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "info: ";
    s << std::endl;
    Printer< ::dogm_msgs::GridMapInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.info);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dogm_msgs::GridCell_<ContainerAllocator> >::stream(s, indent + "    ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOGM_MSGS_MESSAGE_DYNAMICOCCUPANCYGRID_H
