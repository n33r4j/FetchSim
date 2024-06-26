// Generated by gencpp from file fetchit_challenge/SchunkMachineFeedback.msg
// DO NOT EDIT!


#ifndef FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEFEEDBACK_H
#define FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fetchit_challenge
{
template <class ContainerAllocator>
struct SchunkMachineFeedback_
{
  typedef SchunkMachineFeedback_<ContainerAllocator> Type;

  SchunkMachineFeedback_()
    {
    }
  SchunkMachineFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct SchunkMachineFeedback_

typedef ::fetchit_challenge::SchunkMachineFeedback_<std::allocator<void> > SchunkMachineFeedback;

typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineFeedback > SchunkMachineFeedbackPtr;
typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineFeedback const> SchunkMachineFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace fetchit_challenge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fetchit_challenge/SchunkMachineFeedback";
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SchunkMachineFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::fetchit_challenge::SchunkMachineFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEFEEDBACK_H
