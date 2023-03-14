// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/hybrid_grid.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "cartographer/mapping/proto/hybrid_grid.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace mapping {
namespace proto {

namespace {

const ::google::protobuf::Descriptor* HybridGrid_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  HybridGrid_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() {
  protobuf_AddDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "cartographer/mapping/proto/hybrid_grid.proto");
  GOOGLE_CHECK(file != NULL);
  HybridGrid_descriptor_ = file->message_type(0);
  static const int HybridGrid_offsets_[5] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, resolution_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, x_indices_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, y_indices_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, z_indices_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, values_),
  };
  HybridGrid_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      HybridGrid_descriptor_,
      HybridGrid::default_instance_,
      HybridGrid_offsets_,
      -1,
      -1,
      -1,
      sizeof(HybridGrid),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, _internal_metadata_),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, _is_default_instance_));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      HybridGrid_descriptor_, &HybridGrid::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() {
  delete HybridGrid::default_instance_;
  delete HybridGrid_reflection_;
}

void protobuf_AddDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n,cartographer/mapping/proto/hybrid_grid"
    ".proto\022\032cartographer.mapping.proto\"i\n\nHy"
    "bridGrid\022\022\n\nresolution\030\001 \001(\002\022\021\n\tx_indice"
    "s\030\003 \003(\021\022\021\n\ty_indices\030\004 \003(\021\022\021\n\tz_indices\030"
    "\005 \003(\021\022\016\n\006values\030\006 \003(\005b\006proto3", 189);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/hybrid_grid.proto", &protobuf_RegisterTypes);
  HybridGrid::default_instance_ = new HybridGrid();
  HybridGrid::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto {
  StaticDescriptorInitializer_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto() {
    protobuf_AddDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto();
  }
} static_descriptor_initializer_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int HybridGrid::kResolutionFieldNumber;
const int HybridGrid::kXIndicesFieldNumber;
const int HybridGrid::kYIndicesFieldNumber;
const int HybridGrid::kZIndicesFieldNumber;
const int HybridGrid::kValuesFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

HybridGrid::HybridGrid()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.HybridGrid)
}

void HybridGrid::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

HybridGrid::HybridGrid(const HybridGrid& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.HybridGrid)
}

void HybridGrid::SharedCtor() {
    _is_default_instance_ = false;
  _cached_size_ = 0;
  resolution_ = 0;
}

HybridGrid::~HybridGrid() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.HybridGrid)
  SharedDtor();
}

void HybridGrid::SharedDtor() {
  if (this != default_instance_) {
  }
}

void HybridGrid::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* HybridGrid::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return HybridGrid_descriptor_;
}

const HybridGrid& HybridGrid::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto();
  return *default_instance_;
}

HybridGrid* HybridGrid::default_instance_ = NULL;

HybridGrid* HybridGrid::New(::google::protobuf::Arena* arena) const {
  HybridGrid* n = new HybridGrid;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void HybridGrid::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.HybridGrid)
  resolution_ = 0;
  x_indices_.Clear();
  y_indices_.Clear();
  z_indices_.Clear();
  values_.Clear();
}

bool HybridGrid::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.HybridGrid)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional float resolution = 1;
      case 1: {
        if (tag == 13) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &resolution_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_x_indices;
        break;
      }

      // repeated sint32 x_indices = 3;
      case 3: {
        if (tag == 26) {
         parse_x_indices:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_x_indices())));
        } else if (tag == 24) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 26, input, this->mutable_x_indices())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_y_indices;
        break;
      }

      // repeated sint32 y_indices = 4;
      case 4: {
        if (tag == 34) {
         parse_y_indices:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_y_indices())));
        } else if (tag == 32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 34, input, this->mutable_y_indices())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(42)) goto parse_z_indices;
        break;
      }

      // repeated sint32 z_indices = 5;
      case 5: {
        if (tag == 42) {
         parse_z_indices:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_z_indices())));
        } else if (tag == 40) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 42, input, this->mutable_z_indices())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(50)) goto parse_values;
        break;
      }

      // repeated int32 values = 6;
      case 6: {
        if (tag == 50) {
         parse_values:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, this->mutable_values())));
        } else if (tag == 48) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 1, 50, input, this->mutable_values())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.HybridGrid)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.HybridGrid)
  return false;
#undef DO_
}

void HybridGrid::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.HybridGrid)
  // optional float resolution = 1;
  if (this->resolution() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->resolution(), output);
  }

  // repeated sint32 x_indices = 3;
  if (this->x_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(3, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_x_indices_cached_byte_size_);
  }
  for (int i = 0; i < this->x_indices_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->x_indices(i), output);
  }

  // repeated sint32 y_indices = 4;
  if (this->y_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(4, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_y_indices_cached_byte_size_);
  }
  for (int i = 0; i < this->y_indices_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->y_indices(i), output);
  }

  // repeated sint32 z_indices = 5;
  if (this->z_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(5, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_z_indices_cached_byte_size_);
  }
  for (int i = 0; i < this->z_indices_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->z_indices(i), output);
  }

  // repeated int32 values = 6;
  if (this->values_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(6, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_values_cached_byte_size_);
  }
  for (int i = 0; i < this->values_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32NoTag(
      this->values(i), output);
  }

  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.HybridGrid)
}

::google::protobuf::uint8* HybridGrid::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.HybridGrid)
  // optional float resolution = 1;
  if (this->resolution() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->resolution(), target);
  }

  // repeated sint32 x_indices = 3;
  if (this->x_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      3,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _x_indices_cached_byte_size_, target);
  }
  for (int i = 0; i < this->x_indices_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->x_indices(i), target);
  }

  // repeated sint32 y_indices = 4;
  if (this->y_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      4,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _y_indices_cached_byte_size_, target);
  }
  for (int i = 0; i < this->y_indices_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->y_indices(i), target);
  }

  // repeated sint32 z_indices = 5;
  if (this->z_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      5,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _z_indices_cached_byte_size_, target);
  }
  for (int i = 0; i < this->z_indices_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->z_indices(i), target);
  }

  // repeated int32 values = 6;
  if (this->values_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      6,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _values_cached_byte_size_, target);
  }
  for (int i = 0; i < this->values_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteInt32NoTagToArray(this->values(i), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.HybridGrid)
  return target;
}

int HybridGrid::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.HybridGrid)
  int total_size = 0;

  // optional float resolution = 1;
  if (this->resolution() != 0) {
    total_size += 1 + 4;
  }

  // repeated sint32 x_indices = 3;
  {
    int data_size = 0;
    for (int i = 0; i < this->x_indices_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        SInt32Size(this->x_indices(i));
    }
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _x_indices_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 y_indices = 4;
  {
    int data_size = 0;
    for (int i = 0; i < this->y_indices_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        SInt32Size(this->y_indices(i));
    }
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _y_indices_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 z_indices = 5;
  {
    int data_size = 0;
    for (int i = 0; i < this->z_indices_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        SInt32Size(this->z_indices(i));
    }
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _z_indices_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated int32 values = 6;
  {
    int data_size = 0;
    for (int i = 0; i < this->values_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        Int32Size(this->values(i));
    }
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _values_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void HybridGrid::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.HybridGrid)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const HybridGrid* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const HybridGrid>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.HybridGrid)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.HybridGrid)
    MergeFrom(*source);
  }
}

void HybridGrid::MergeFrom(const HybridGrid& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.HybridGrid)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  x_indices_.MergeFrom(from.x_indices_);
  y_indices_.MergeFrom(from.y_indices_);
  z_indices_.MergeFrom(from.z_indices_);
  values_.MergeFrom(from.values_);
  if (from.resolution() != 0) {
    set_resolution(from.resolution());
  }
}

void HybridGrid::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.HybridGrid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void HybridGrid::CopyFrom(const HybridGrid& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.HybridGrid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool HybridGrid::IsInitialized() const {

  return true;
}

void HybridGrid::Swap(HybridGrid* other) {
  if (other == this) return;
  InternalSwap(other);
}
void HybridGrid::InternalSwap(HybridGrid* other) {
  std::swap(resolution_, other->resolution_);
  x_indices_.UnsafeArenaSwap(&other->x_indices_);
  y_indices_.UnsafeArenaSwap(&other->y_indices_);
  z_indices_.UnsafeArenaSwap(&other->z_indices_);
  values_.UnsafeArenaSwap(&other->values_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata HybridGrid::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = HybridGrid_descriptor_;
  metadata.reflection = HybridGrid_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// HybridGrid

// optional float resolution = 1;
void HybridGrid::clear_resolution() {
  resolution_ = 0;
}
 float HybridGrid::resolution() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.resolution)
  return resolution_;
}
 void HybridGrid::set_resolution(float value) {
  
  resolution_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.resolution)
}

// repeated sint32 x_indices = 3;
int HybridGrid::x_indices_size() const {
  return x_indices_.size();
}
void HybridGrid::clear_x_indices() {
  x_indices_.Clear();
}
 ::google::protobuf::int32 HybridGrid::x_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.x_indices)
  return x_indices_.Get(index);
}
 void HybridGrid::set_x_indices(int index, ::google::protobuf::int32 value) {
  x_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.x_indices)
}
 void HybridGrid::add_x_indices(::google::protobuf::int32 value) {
  x_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.x_indices)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::x_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.x_indices)
  return x_indices_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_x_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.x_indices)
  return &x_indices_;
}

// repeated sint32 y_indices = 4;
int HybridGrid::y_indices_size() const {
  return y_indices_.size();
}
void HybridGrid::clear_y_indices() {
  y_indices_.Clear();
}
 ::google::protobuf::int32 HybridGrid::y_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.y_indices)
  return y_indices_.Get(index);
}
 void HybridGrid::set_y_indices(int index, ::google::protobuf::int32 value) {
  y_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.y_indices)
}
 void HybridGrid::add_y_indices(::google::protobuf::int32 value) {
  y_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.y_indices)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::y_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.y_indices)
  return y_indices_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_y_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.y_indices)
  return &y_indices_;
}

// repeated sint32 z_indices = 5;
int HybridGrid::z_indices_size() const {
  return z_indices_.size();
}
void HybridGrid::clear_z_indices() {
  z_indices_.Clear();
}
 ::google::protobuf::int32 HybridGrid::z_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.z_indices)
  return z_indices_.Get(index);
}
 void HybridGrid::set_z_indices(int index, ::google::protobuf::int32 value) {
  z_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.z_indices)
}
 void HybridGrid::add_z_indices(::google::protobuf::int32 value) {
  z_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.z_indices)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::z_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.z_indices)
  return z_indices_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_z_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.z_indices)
  return &z_indices_;
}

// repeated int32 values = 6;
int HybridGrid::values_size() const {
  return values_.size();
}
void HybridGrid::clear_values() {
  values_.Clear();
}
 ::google::protobuf::int32 HybridGrid::values(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.values)
  return values_.Get(index);
}
 void HybridGrid::set_values(int index, ::google::protobuf::int32 value) {
  values_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.values)
}
 void HybridGrid::add_values(::google::protobuf::int32 value) {
  values_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.values)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::values() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.values)
  return values_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_values() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.values)
  return &values_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)