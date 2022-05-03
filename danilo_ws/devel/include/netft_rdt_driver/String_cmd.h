// Generated by gencpp from file netft_rdt_driver/String_cmd.msg
// DO NOT EDIT!


#ifndef NETFT_RDT_DRIVER_MESSAGE_STRING_CMD_H
#define NETFT_RDT_DRIVER_MESSAGE_STRING_CMD_H

#include <ros/service_traits.h>


#include <netft_rdt_driver/String_cmdRequest.h>
#include <netft_rdt_driver/String_cmdResponse.h>


namespace netft_rdt_driver
{

struct String_cmd
{

typedef String_cmdRequest Request;
typedef String_cmdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct String_cmd
} // namespace netft_rdt_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::netft_rdt_driver::String_cmd > {
  static const char* value()
  {
    return "d4463b49bd5bb77dbd8c4356f5dc1c28";
  }

  static const char* value(const ::netft_rdt_driver::String_cmd&) { return value(); }
};

template<>
struct DataType< ::netft_rdt_driver::String_cmd > {
  static const char* value()
  {
    return "netft_rdt_driver/String_cmd";
  }

  static const char* value(const ::netft_rdt_driver::String_cmd&) { return value(); }
};


// service_traits::MD5Sum< ::netft_rdt_driver::String_cmdRequest> should match
// service_traits::MD5Sum< ::netft_rdt_driver::String_cmd >
template<>
struct MD5Sum< ::netft_rdt_driver::String_cmdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::netft_rdt_driver::String_cmd >::value();
  }
  static const char* value(const ::netft_rdt_driver::String_cmdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_rdt_driver::String_cmdRequest> should match
// service_traits::DataType< ::netft_rdt_driver::String_cmd >
template<>
struct DataType< ::netft_rdt_driver::String_cmdRequest>
{
  static const char* value()
  {
    return DataType< ::netft_rdt_driver::String_cmd >::value();
  }
  static const char* value(const ::netft_rdt_driver::String_cmdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::netft_rdt_driver::String_cmdResponse> should match
// service_traits::MD5Sum< ::netft_rdt_driver::String_cmd >
template<>
struct MD5Sum< ::netft_rdt_driver::String_cmdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::netft_rdt_driver::String_cmd >::value();
  }
  static const char* value(const ::netft_rdt_driver::String_cmdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_rdt_driver::String_cmdResponse> should match
// service_traits::DataType< ::netft_rdt_driver::String_cmd >
template<>
struct DataType< ::netft_rdt_driver::String_cmdResponse>
{
  static const char* value()
  {
    return DataType< ::netft_rdt_driver::String_cmd >::value();
  }
  static const char* value(const ::netft_rdt_driver::String_cmdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NETFT_RDT_DRIVER_MESSAGE_STRING_CMD_H
