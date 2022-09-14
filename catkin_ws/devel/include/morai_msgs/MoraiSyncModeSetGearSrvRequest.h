// Generated by gencpp from file morai_msgs/MoraiSyncModeSetGearSrvRequest.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_MORAISYNCMODESETGEARSRVREQUEST_H
#define MORAI_MSGS_MESSAGE_MORAISYNCMODESETGEARSRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <morai_msgs/SyncModeSetGear.h>

namespace morai_msgs
{
template <class ContainerAllocator>
struct MoraiSyncModeSetGearSrvRequest_
{
  typedef MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> Type;

  MoraiSyncModeSetGearSrvRequest_()
    : request()  {
    }
  MoraiSyncModeSetGearSrvRequest_(const ContainerAllocator& _alloc)
    : request(_alloc)  {
  (void)_alloc;
    }



   typedef  ::morai_msgs::SyncModeSetGear_<ContainerAllocator>  _request_type;
  _request_type request;





  typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MoraiSyncModeSetGearSrvRequest_

typedef ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<std::allocator<void> > MoraiSyncModeSetGearSrvRequest;

typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeSetGearSrvRequest > MoraiSyncModeSetGearSrvRequestPtr;
typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeSetGearSrvRequest const> MoraiSyncModeSetGearSrvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.request == rhs.request;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a38de95f9e9ee5e44b8b9bd7cda023a8";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa38de95f9e9ee5e4ULL;
  static const uint64_t static_value2 = 0x4b8b9bd7cda023a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/MoraiSyncModeSetGearSrvRequest";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SyncModeSetGear request\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/SyncModeSetGear\n"
"int32 gear\n"
"uint64 frame\n"
;
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.request);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoraiSyncModeSetGearSrvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::MoraiSyncModeSetGearSrvRequest_<ContainerAllocator>& v)
  {
    s << indent << "request: ";
    s << std::endl;
    Printer< ::morai_msgs::SyncModeSetGear_<ContainerAllocator> >::stream(s, indent + "  ", v.request);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_MORAISYNCMODESETGEARSRVREQUEST_H
