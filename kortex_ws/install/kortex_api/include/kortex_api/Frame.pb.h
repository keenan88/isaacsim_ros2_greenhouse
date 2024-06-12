// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Frame.proto

#ifndef PROTOBUF_Frame_2eproto__INCLUDED
#define PROTOBUF_Frame_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "Errors.pb.h"  // IWYU pragma: export
// @@protoc_insertion_point(includes)

namespace protobuf_Frame_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[3];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsFrameImpl();
void InitDefaultsFrame();
void InitDefaultsHeaderImpl();
void InitDefaultsHeader();
void InitDefaultsErrorImpl();
void InitDefaultsError();
inline void InitDefaults() {
  InitDefaultsFrame();
  InitDefaultsHeader();
  InitDefaultsError();
}
}  // namespace protobuf_Frame_2eproto
namespace Kinova {
namespace Api {
class Error;
class ErrorDefaultTypeInternal;
extern ErrorDefaultTypeInternal _Error_default_instance_;
class Frame;
class FrameDefaultTypeInternal;
extern FrameDefaultTypeInternal _Frame_default_instance_;
class Header;
class HeaderDefaultTypeInternal;
extern HeaderDefaultTypeInternal _Header_default_instance_;
}  // namespace Api
}  // namespace Kinova
namespace Kinova {
namespace Api {

enum HeaderVersion {
  RESERVED_0 = 0,
  CURRENT_VERSION = 1,
  HeaderVersion_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  HeaderVersion_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool HeaderVersion_IsValid(int value);
const HeaderVersion HeaderVersion_MIN = RESERVED_0;
const HeaderVersion HeaderVersion_MAX = CURRENT_VERSION;
const int HeaderVersion_ARRAYSIZE = HeaderVersion_MAX + 1;

const ::google::protobuf::EnumDescriptor* HeaderVersion_descriptor();
inline const ::std::string& HeaderVersion_Name(HeaderVersion value) {
  return ::google::protobuf::internal::NameOfEnum(
    HeaderVersion_descriptor(), value);
}
inline bool HeaderVersion_Parse(
    const ::std::string& name, HeaderVersion* value) {
  return ::google::protobuf::internal::ParseNamedEnum<HeaderVersion>(
    HeaderVersion_descriptor(), name, value);
}
enum FrameTypes {
  RESERVED = 0,
  MSG_FRAME_REQUEST = 1,
  MSG_FRAME_RAW = 2,
  MSG_FRAME_RESPONSE = 3,
  MSG_FRAME_NOTIFICATION = 5,
  MSG_FRAME_PING = 7,
  MSG_FRAME_PONG = 8,
  FrameTypes_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  FrameTypes_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool FrameTypes_IsValid(int value);
const FrameTypes FrameTypes_MIN = RESERVED;
const FrameTypes FrameTypes_MAX = MSG_FRAME_PONG;
const int FrameTypes_ARRAYSIZE = FrameTypes_MAX + 1;

const ::google::protobuf::EnumDescriptor* FrameTypes_descriptor();
inline const ::std::string& FrameTypes_Name(FrameTypes value) {
  return ::google::protobuf::internal::NameOfEnum(
    FrameTypes_descriptor(), value);
}
inline bool FrameTypes_Parse(
    const ::std::string& name, FrameTypes* value) {
  return ::google::protobuf::internal::ParseNamedEnum<FrameTypes>(
    FrameTypes_descriptor(), name, value);
}
// ===================================================================

class Frame : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:Kinova.Api.Frame) */ {
 public:
  Frame();
  virtual ~Frame();

  Frame(const Frame& from);

  inline Frame& operator=(const Frame& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Frame(Frame&& from) noexcept
    : Frame() {
    *this = ::std::move(from);
  }

  inline Frame& operator=(Frame&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Frame& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Frame* internal_default_instance() {
    return reinterpret_cast<const Frame*>(
               &_Frame_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Frame* other);
  friend void swap(Frame& a, Frame& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Frame* New() const PROTOBUF_FINAL { return New(NULL); }

  Frame* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Frame& from);
  void MergeFrom(const Frame& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Frame* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // bytes payload = 2;
  void clear_payload();
  static const int kPayloadFieldNumber = 2;
  const ::std::string& payload() const;
  void set_payload(const ::std::string& value);
  #if LANG_CXX11
  void set_payload(::std::string&& value);
  #endif
  void set_payload(const char* value);
  void set_payload(const void* value, size_t size);
  ::std::string* mutable_payload();
  ::std::string* release_payload();
  void set_allocated_payload(::std::string* payload);

  // .Kinova.Api.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::Kinova::Api::Header& header() const;
  ::Kinova::Api::Header* release_header();
  ::Kinova::Api::Header* mutable_header();
  void set_allocated_header(::Kinova::Api::Header* header);

  // @@protoc_insertion_point(class_scope:Kinova.Api.Frame)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr payload_;
  ::Kinova::Api::Header* header_;
  mutable int _cached_size_;
  friend struct ::protobuf_Frame_2eproto::TableStruct;
  friend void ::protobuf_Frame_2eproto::InitDefaultsFrameImpl();
};
// -------------------------------------------------------------------

class Header : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:Kinova.Api.Header) */ {
 public:
  Header();
  virtual ~Header();

  Header(const Header& from);

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Header(Header&& from) noexcept
    : Header() {
    *this = ::std::move(from);
  }

  inline Header& operator=(Header&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Header& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Header* internal_default_instance() {
    return reinterpret_cast<const Header*>(
               &_Header_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(Header* other);
  friend void swap(Header& a, Header& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Header* New() const PROTOBUF_FINAL { return New(NULL); }

  Header* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Header* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // fixed32 frame_info = 1;
  void clear_frame_info();
  static const int kFrameInfoFieldNumber = 1;
  ::google::protobuf::uint32 frame_info() const;
  void set_frame_info(::google::protobuf::uint32 value);

  // fixed32 message_info = 2;
  void clear_message_info();
  static const int kMessageInfoFieldNumber = 2;
  ::google::protobuf::uint32 message_info() const;
  void set_message_info(::google::protobuf::uint32 value);

  // fixed32 service_info = 3;
  void clear_service_info();
  static const int kServiceInfoFieldNumber = 3;
  ::google::protobuf::uint32 service_info() const;
  void set_service_info(::google::protobuf::uint32 value);

  // fixed32 payload_info = 4;
  void clear_payload_info();
  static const int kPayloadInfoFieldNumber = 4;
  ::google::protobuf::uint32 payload_info() const;
  void set_payload_info(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:Kinova.Api.Header)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 frame_info_;
  ::google::protobuf::uint32 message_info_;
  ::google::protobuf::uint32 service_info_;
  ::google::protobuf::uint32 payload_info_;
  mutable int _cached_size_;
  friend struct ::protobuf_Frame_2eproto::TableStruct;
  friend void ::protobuf_Frame_2eproto::InitDefaultsHeaderImpl();
};
// -------------------------------------------------------------------

class Error : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:Kinova.Api.Error) */ {
 public:
  Error();
  virtual ~Error();

  Error(const Error& from);

  inline Error& operator=(const Error& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Error(Error&& from) noexcept
    : Error() {
    *this = ::std::move(from);
  }

  inline Error& operator=(Error&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Error& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Error* internal_default_instance() {
    return reinterpret_cast<const Error*>(
               &_Error_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(Error* other);
  friend void swap(Error& a, Error& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Error* New() const PROTOBUF_FINAL { return New(NULL); }

  Error* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Error& from);
  void MergeFrom(const Error& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Error* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string error_sub_string = 3;
  void clear_error_sub_string();
  static const int kErrorSubStringFieldNumber = 3;
  const ::std::string& error_sub_string() const;
  void set_error_sub_string(const ::std::string& value);
  #if LANG_CXX11
  void set_error_sub_string(::std::string&& value);
  #endif
  void set_error_sub_string(const char* value);
  void set_error_sub_string(const char* value, size_t size);
  ::std::string* mutable_error_sub_string();
  ::std::string* release_error_sub_string();
  void set_allocated_error_sub_string(::std::string* error_sub_string);

  // .Kinova.Api.ErrorCodes error_code = 1;
  void clear_error_code();
  static const int kErrorCodeFieldNumber = 1;
  ::Kinova::Api::ErrorCodes error_code() const;
  void set_error_code(::Kinova::Api::ErrorCodes value);

  // uint32 error_sub_code = 2;
  void clear_error_sub_code();
  static const int kErrorSubCodeFieldNumber = 2;
  ::google::protobuf::uint32 error_sub_code() const;
  void set_error_sub_code(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:Kinova.Api.Error)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr error_sub_string_;
  int error_code_;
  ::google::protobuf::uint32 error_sub_code_;
  mutable int _cached_size_;
  friend struct ::protobuf_Frame_2eproto::TableStruct;
  friend void ::protobuf_Frame_2eproto::InitDefaultsErrorImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Frame

// .Kinova.Api.Header header = 1;
inline bool Frame::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline void Frame::clear_header() {
  if (GetArenaNoVirtual() == NULL && header_ != NULL) {
    delete header_;
  }
  header_ = NULL;
}
inline const ::Kinova::Api::Header& Frame::header() const {
  const ::Kinova::Api::Header* p = header_;
  // @@protoc_insertion_point(field_get:Kinova.Api.Frame.header)
  return p != NULL ? *p : *reinterpret_cast<const ::Kinova::Api::Header*>(
      &::Kinova::Api::_Header_default_instance_);
}
inline ::Kinova::Api::Header* Frame::release_header() {
  // @@protoc_insertion_point(field_release:Kinova.Api.Frame.header)
  
  ::Kinova::Api::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::Kinova::Api::Header* Frame::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::Kinova::Api::Header;
  }
  // @@protoc_insertion_point(field_mutable:Kinova.Api.Frame.header)
  return header_;
}
inline void Frame::set_allocated_header(::Kinova::Api::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete header_;
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:Kinova.Api.Frame.header)
}

// bytes payload = 2;
inline void Frame::clear_payload() {
  payload_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Frame::payload() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Frame.payload)
  return payload_.GetNoArena();
}
inline void Frame::set_payload(const ::std::string& value) {
  
  payload_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:Kinova.Api.Frame.payload)
}
#if LANG_CXX11
inline void Frame::set_payload(::std::string&& value) {
  
  payload_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:Kinova.Api.Frame.payload)
}
#endif
inline void Frame::set_payload(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  payload_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:Kinova.Api.Frame.payload)
}
inline void Frame::set_payload(const void* value, size_t size) {
  
  payload_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:Kinova.Api.Frame.payload)
}
inline ::std::string* Frame::mutable_payload() {
  
  // @@protoc_insertion_point(field_mutable:Kinova.Api.Frame.payload)
  return payload_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Frame::release_payload() {
  // @@protoc_insertion_point(field_release:Kinova.Api.Frame.payload)
  
  return payload_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Frame::set_allocated_payload(::std::string* payload) {
  if (payload != NULL) {
    
  } else {
    
  }
  payload_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), payload);
  // @@protoc_insertion_point(field_set_allocated:Kinova.Api.Frame.payload)
}

// -------------------------------------------------------------------

// Header

// fixed32 frame_info = 1;
inline void Header::clear_frame_info() {
  frame_info_ = 0u;
}
inline ::google::protobuf::uint32 Header::frame_info() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Header.frame_info)
  return frame_info_;
}
inline void Header::set_frame_info(::google::protobuf::uint32 value) {
  
  frame_info_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Header.frame_info)
}

// fixed32 message_info = 2;
inline void Header::clear_message_info() {
  message_info_ = 0u;
}
inline ::google::protobuf::uint32 Header::message_info() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Header.message_info)
  return message_info_;
}
inline void Header::set_message_info(::google::protobuf::uint32 value) {
  
  message_info_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Header.message_info)
}

// fixed32 service_info = 3;
inline void Header::clear_service_info() {
  service_info_ = 0u;
}
inline ::google::protobuf::uint32 Header::service_info() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Header.service_info)
  return service_info_;
}
inline void Header::set_service_info(::google::protobuf::uint32 value) {
  
  service_info_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Header.service_info)
}

// fixed32 payload_info = 4;
inline void Header::clear_payload_info() {
  payload_info_ = 0u;
}
inline ::google::protobuf::uint32 Header::payload_info() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Header.payload_info)
  return payload_info_;
}
inline void Header::set_payload_info(::google::protobuf::uint32 value) {
  
  payload_info_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Header.payload_info)
}

// -------------------------------------------------------------------

// Error

// .Kinova.Api.ErrorCodes error_code = 1;
inline void Error::clear_error_code() {
  error_code_ = 0;
}
inline ::Kinova::Api::ErrorCodes Error::error_code() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Error.error_code)
  return static_cast< ::Kinova::Api::ErrorCodes >(error_code_);
}
inline void Error::set_error_code(::Kinova::Api::ErrorCodes value) {
  
  error_code_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Error.error_code)
}

// uint32 error_sub_code = 2;
inline void Error::clear_error_sub_code() {
  error_sub_code_ = 0u;
}
inline ::google::protobuf::uint32 Error::error_sub_code() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Error.error_sub_code)
  return error_sub_code_;
}
inline void Error::set_error_sub_code(::google::protobuf::uint32 value) {
  
  error_sub_code_ = value;
  // @@protoc_insertion_point(field_set:Kinova.Api.Error.error_sub_code)
}

// string error_sub_string = 3;
inline void Error::clear_error_sub_string() {
  error_sub_string_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Error::error_sub_string() const {
  // @@protoc_insertion_point(field_get:Kinova.Api.Error.error_sub_string)
  return error_sub_string_.GetNoArena();
}
inline void Error::set_error_sub_string(const ::std::string& value) {
  
  error_sub_string_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:Kinova.Api.Error.error_sub_string)
}
#if LANG_CXX11
inline void Error::set_error_sub_string(::std::string&& value) {
  
  error_sub_string_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:Kinova.Api.Error.error_sub_string)
}
#endif
inline void Error::set_error_sub_string(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  error_sub_string_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:Kinova.Api.Error.error_sub_string)
}
inline void Error::set_error_sub_string(const char* value, size_t size) {
  
  error_sub_string_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:Kinova.Api.Error.error_sub_string)
}
inline ::std::string* Error::mutable_error_sub_string() {
  
  // @@protoc_insertion_point(field_mutable:Kinova.Api.Error.error_sub_string)
  return error_sub_string_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Error::release_error_sub_string() {
  // @@protoc_insertion_point(field_release:Kinova.Api.Error.error_sub_string)
  
  return error_sub_string_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Error::set_allocated_error_sub_string(::std::string* error_sub_string) {
  if (error_sub_string != NULL) {
    
  } else {
    
  }
  error_sub_string_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), error_sub_string);
  // @@protoc_insertion_point(field_set_allocated:Kinova.Api.Error.error_sub_string)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace Api
}  // namespace Kinova

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::Kinova::Api::HeaderVersion> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::Kinova::Api::HeaderVersion>() {
  return ::Kinova::Api::HeaderVersion_descriptor();
}
template <> struct is_proto_enum< ::Kinova::Api::FrameTypes> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::Kinova::Api::FrameTypes>() {
  return ::Kinova::Api::FrameTypes_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Frame_2eproto__INCLUDED
