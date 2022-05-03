// Generated by gencpp from file netft_rdt_driver/String_cmdResponse.msg
// DO NOT EDIT!


#ifndef NETFT_RDT_DRIVER_MESSAGE_STRING_CMDRESPONSE_H
#define NETFT_RDT_DRIVER_MESSAGE_STRING_CMDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace netft_rdt_driver
{
template <class ContainerAllocator>
struct String_cmdResponse_
{
  typedef String_cmdResponse_<ContainerAllocator> Type;

  String_cmdResponse_()
    : res()  {
    }
  String_cmdResponse_(const ContainerAllocator& _alloc)
    : res(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _res_type;
  _res_type res;





  typedef boost::shared_ptr< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> const> ConstPtr;

}; // struct String_cmdResponse_

typedef ::netft_rdt_driver::String_cmdResponse_<std::allocator<void> > String_cmdResponse;

typedef boost::shared_ptr< ::netft_rdt_driver::String_cmdResponse > String_cmdResponsePtr;
typedef boost::shared_ptr< ::netft_rdt_driver::String_cmdResponse const> String_cmdResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator1> & lhs, const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator2> & rhs)
{
  return lhs.res == rhs.res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator1> & lhs, const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace netft_rdt_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "53af918a2a4a2a182c184142fff49b0c";
  }

  static const char* value(const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x53af918a2a4a2a18ULL;
  static const uint64_t static_value2 = 0x2c184142fff49b0cULL;
};

template<class ContainerAllocator>
struct DataType< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "netft_rdt_driver/String_cmdResponse";
  }

  static const char* value(const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string res\n"
"\n"
;
  }

  static const char* value(const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct String_cmdResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::netft_rdt_driver::String_cmdResponse_<ContainerAllocator>& v)
  {
    s << indent << "res: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NETFT_RDT_DRIVER_MESSAGE_STRING_CMDRESPONSE_H
