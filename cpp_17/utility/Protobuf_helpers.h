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
    std::optional<std::vector<uint8_t>> vector_from(Protobuf_Ty& proto_buf)
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
    std::optional<Protobuf_Ty> from_vector_into(const std::vector<uint8_t>& data)
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
    static bool SerializeToProtocolBufferString(std::string& objectString, const U& objectToSerialize)
    {
        try {
            T protoObject;
            objectToSerialize->ToProtocolBuffer(&protoObject);
            google::protobuf::TextFormat::PrintToString(protoObject, &objectString);

            return true;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T, class U>
    static bool DeserializeFromProtocolBufferString(const std::string& objectString,
                                                    std::shared_ptr<U>& objectToDeserializeInto)
    {
        try {
            T protoObject;
            auto result = false;
            result      = google::protobuf::TextFormat::ParseFromString(objectString, &protoObject);

            if (result) // Do not replace the object we passed if the decode was bad
                objectToDeserializeInto = make_shared_owner<U>(protoObject);

            return result;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T>
    static bool SerializeToProtocolBuffer(std::vector<uint8_t>& data,
                                          const std::shared_ptr<T>& protoObject,
                                          bool humanReadable = false)
    {
        try {
            std::string objectString;

            if (humanReadable)
                google::protobuf::TextFormat::PrintToString(*protoObject, &objectString);
            else
                protoObject->SerializeToString(&objectString);

            data = std::vector<uint8_t>(objectString.begin(), objectString.end());
            return true;
        }
        catch (std::exception&) {
            return false;
        }
    }


    template<class T>
    static bool DeserializeFromProtocolBuffer(const std::vector<uint8_t>& data,
                                              std::shared_ptr<T>& objectToDeserializeInto,
                                              bool humanReadable = false)
    {
        try {
            T protoObject;

            auto result = false;

            auto objectString = std::string(data.begin(), data.end());

            if (humanReadable)
                result = google::protobuf::TextFormat::ParseFromString(objectString, &protoObject);
            else
                result = protoObject.ParseFromString(objectString);

            if (result) // Do not replace the object we passed if the decode was bad
                objectToDeserializeInto = make_shared_owner<T>(protoObject);

            return result;
        }
        catch (std::exception&) {
            return false;
        }
    }
} // namespace Navtech::Protobuf
