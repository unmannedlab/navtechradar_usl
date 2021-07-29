#pragma once

#include <google/protobuf/text_format.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Pointer_types.h"

namespace Navtech::Protobuf {

    enum class Format { text, binary };

    template<typename Protobuf_Ty>
    std::optional<std::string> string_from(const Protobuf_Ty& object_to_serialize, Format format = Format::binary)
    {
        try {
            std::string output_str {};

            if (object_to_serialize.SerializeToString(&output_str)) { return output_str; }
            else {
                return std::nullopt;
            }
        }
        catch (std::exception&) {
            return std::nullopt;
        }
    }


    template<class Protobuf_Ty>
    std::optional<Protobuf_Ty> from_string_into(const std::string& input_str)
    {
        try {
            Protobuf_Ty proto_buf {};

            if (proto_buf.ParseFromString(input_str)) { return proto_buf; }
            else {
                return std::nullopt;
            }
        }
        catch (std::exception&) {
            return std::nullopt;
        }
    }


    template<class Protobuf_Ty>
    std::optional<std::vector<std::uint8_t>> vector_from(Protobuf_Ty& proto_buf)
    {
        try {
            std::string str;
            if (proto_buf.SerializeToString(&str)) { return std::vector<uint8_t> { str.begin(), str.end() }; }
            else {
                return std::nullopt;
            }
        }
        catch (std::exception&) {
            return std::nullopt;
        }
    }


    template<class Protobuf_Ty>
    std::optional<Protobuf_Ty> from_vector_into(const std::vector<std::uint8_t>& data)
    {
        try {
            Protobuf_Ty proto_buf;
            std::string data_str { data.begin(), data.end() };

            if (proto_buf.ParseFromString(data_str)) { return proto_buf; }
            else {
                return std::nullopt;
            }
        }
        catch (std::exception&) {
            return std::nullopt;
        }
    }


    template<class T, class U>
    static bool serialize_to_protocol_buffer_string(std::string& object_string, const U& object_to_serialize)
    {
        try {
            T protoObject;
            object_to_serialize->to_protocol_buffer(&protoObject);
            google::protobuf::TextFormat::PrintToString(protoObject, &object_string);

            return true;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T, class U>
    static bool deserialize_from_protocol_buffer_string(const std::string& object_string,
                                                        Shared_owner<U>& object_to_deserialize_into)
    {
        try {
            T protoObject;
            auto result = false;
            result      = google::protobuf::TextFormat::ParseFromString(object_string, &protoObject);

            if (result) // Do not replace the object we passed if the decode was bad
                object_to_deserialize_into = allocate_shared<U>(protoObject);

            return result;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T>
    static bool serialize_to_protocol_buffer(std::vector<std::uint8_t>& data,
                                             const Shared_owner<T>& proto_object,
                                             bool human_readable = false)
    {
        try {
            std::string objectString;

            if (human_readable)
                google::protobuf::TextFormat::PrintToString(*proto_object, &objectString);
            else
                proto_object->SerializeToString(&objectString);

            data = std::vector<uint8_t>(objectString.begin(), objectString.end());
            return true;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T>
    static bool deserialize_from_protocol_buffer(const std::vector<std::uint8_t>& data,
                                                 Shared_owner<T>& object_to_deserialize_into,
                                                 bool human_readable = false)
    {
        try {
            T protoObject;

            auto result = false;

            auto objectString = std::string(data.begin(), data.end());

            if (human_readable) { result = google::protobuf::TextFormat::ParseFromString(objectString, &protoObject); }
            else {
                result = protoObject.ParseFromString(objectString);
            }

            if (result) { // Do not replace the object we passed if the decode was bad
                object_to_deserialize_into = allocate_shared<T>(protoObject);
            }

            return result;
        }
        catch (std::exception&) {
            return false;
        }
    }
} // namespace Navtech::Protobuf
