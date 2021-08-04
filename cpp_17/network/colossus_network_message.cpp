#include <algorithm>
#include <array>
#include <cstring>
#include <iomanip>
#include <iostream>

#include <arpa/inet.h>
// #include "winsock.h" // For Windows

#include "colossus_network_message.h"

// Constants
//
constexpr unsigned int largest_valid_message { 128 };
constexpr unsigned int largest_payload { 1'000'000 };

namespace Navtech::Colossus_network_protocol {

    // ---------------------------------------------------------------------------------------------------------
    // Message
    //
    Message::Message() { initialize(); }


    Message::Message(const std::string& ip_addr, Message::ID id) : address { ip_addr }, identity { id }
    {
        initialize();
    }


    Message::Message(const Message::Buffer& message) { replace(message); }


    Message::Message(Message::Buffer&& message) { replace(std::move(message)); }


    Message::Message(Const_iterator message_start, std::size_t message_sz) { replace(message_start, message_sz); }


    Message::Message(const std::string& ip_addr, Message::ID id, const Message::Buffer& message) :
        address { ip_addr }, identity { id }
    {
        replace(message);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Message::Buffer&& message) :
        address { ip_addr }, identity { id }
    {
        replace(std::move(message));
    }


    Message::Message(const std::string& ip_addr, ID id, Const_iterator message_start, std::size_t message_sz) :
        address { ip_addr }, identity { id }
    {
        replace(message_start, message_sz);
    }


    Message::ID Message::id() const { return identity; }


    void Message::id(Message::ID new_id) { identity = new_id; }


    Message::Type Message::type() const
    {
        auto header = Header::overlay_onto(data.data());
        return header->id;
    }


    void Message::type(Message::Type t)
    {
        auto header = Header::overlay_onto(data.data());
        header->id  = t;
    }


    const Utility::IP_address& Message::ip_address() const { return address; }


    void Message::ip_address(const std::string& ip_addr) { address = ip_addr; }


    bool Message::is_valid() const
    {
        if (data.empty()) return false;

        return (is_version_valid() && is_signature_valid() && static_cast<unsigned>(type()) <= largest_valid_message &&
                payload_size() < largest_payload);
    }


    std::size_t Message::size() const { return data.size(); }


    std::size_t Message::payload_size() const
    {
        if (data.empty()) return 0;

        auto header = Header::overlay_onto(data.data());
        return ntohl(header->payload_size);
    }


    void Message::payload_size(std::uint32_t sz)
    {
        if (data.empty()) return;

        auto header          = Header::overlay_onto(data.data());
        header->payload_size = htonl(sz);
    }


    Message::Iterator Message::payload_begin() { return data.data() + header_size(); }


    Message::Const_iterator Message::payload_begin() const { return data.data() + header_size(); }


    Message::Iterator Message::payload_end() { return &(*data.end()); }


    Message::Const_iterator Message::payload_end() const { return &(*data.cend()); }


    void Message::set_view_iterators()
    {
        // TODO - Implement
        auto header = Header::overlay_onto(data.data());

        header->signature.as_iterators.begin = payload_begin();
        header->signature.as_iterators.end   = payload_end();
    }


    void Message::replace(const std::vector<std::uint8_t>& src) { data = src; }


    void Message::replace(std::vector<std::uint8_t>&& src)
    {
        using std::move;

        data = move(src);
    }


    void Message::replace(Message::Const_iterator src_start, std::size_t src_size)
    {
        using std::back_inserter;
        using std::copy_n;

        data.clear();
        data.reserve(src_size);
        copy_n(src_start, src_size, back_inserter(data));
    }


    std::vector<std::uint8_t> Message::relinquish()
    {
        using std::move;
        using std::uint8_t;
        using std::vector;

        add_signature();
        return vector<uint8_t> { move(data) };
    }


    Message& Message::append(const Message::Buffer& protocol_buffer)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        data.reserve(data.size() + protocol_buffer.size());
        copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));

        payload_size(payload_size() + protocol_buffer.size());
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(Message::Buffer&& protocol_buffer)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        Buffer temp { move(protocol_buffer) };

        data.reserve(data.size() + temp.size());
        copy(begin(temp), end(temp), back_inserter(data));

        payload_size(payload_size() + temp.size());
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(const std::string& protocol_buffer)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        data.reserve(data.size() + protocol_buffer.size());
        copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));

        payload_size(payload_size() + protocol_buffer.size());
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(std::string&& protocol_buffer)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        std::string temp { move(protocol_buffer) };

        data.reserve(data.size() + temp.size());
        copy(begin(temp), end(temp), back_inserter(data));

        payload_size(payload_size() + temp.size());
        has_protobuf = true;
        return *this;
    }


    Message& Message::operator<<(const Message::Buffer& protocol_buffer) { return append(protocol_buffer); }


    Message& Message::operator<<(Message::Buffer&& protocol_buffer) { return append(std::move(protocol_buffer)); }


    Message& Message::operator<<(const std::string& protocol_buffer) { return append(protocol_buffer); }


    Message& Message::operator<<(std::string&& protocol_buffer) { return append(std::move(protocol_buffer)); }


    void Message::initialize()
    {
        data.resize(header_size());
        add_signature();
        data[signature_sz] = version;
    }


    bool Message::is_version_valid() const
    {
        auto header = Header::overlay_onto(data.data());
        return header->version == version;
    }


    bool Message::is_signature_valid() const
    {
        using std::array;
        using std::begin;
        using std::memcmp;
        using std::uint8_t;

        return (memcmp(valid_signature.begin(), signature_begin(), valid_signature.size()) == 0);
    }


    void Message::add_signature()
    {
        using std::begin;
        using std::copy;
        using std::end;

        copy(begin(valid_signature), end(valid_signature), signature_begin());
    }


    Message::Iterator Message::signature_begin()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature.as_signature[0]);
    }


    Message::Const_iterator Message::signature_begin() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature.as_signature[0]);
    }


    Message::Iterator Message::signature_end()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature.as_signature[signature_sz]); // One-past-the-end
    }


    Message::Const_iterator Message::signature_end() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature.as_signature[signature_sz]);
    }


    void Message::display()
    {
        using std::cout;
        using std::dec;
        using std::endl;
        using std::hex;

        cout << hex;

        for (auto& val : data) {
            cout << reinterpret_cast<unsigned long>(&val) << "\t" << static_cast<int>(val) << "\t" << val << endl;
        }

        cout << dec;
        cout << endl;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Payload
    //
#if 0
    Message::Payload::Payload(Message& parent) : 
        msg { &parent }
    {

    }


    void Message::Payload::append(const std::vector<std::uint8_t>& src)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        msg->data.reserve(msg->data.size() + src.size());
        copy(begin(src), end(src), back_inserter(msg->data));
        size(src.size());
    }


    void Message::Payload::append(std::vector<std::uint8_t>&& src)
    {
        using std::vector;
        using std::uint8_t;
        using std::begin;
        using std::end;
        using std::back_inserter;
        using std::move;

        vector<uint8_t> temp { move(src) };  // To ensure correct move semantics

        msg->data.reserve(msg->data.size() + src.size());
        copy(begin(temp), end(temp), back_inserter(msg->data));
        size(temp.size());
    }


    void Message::Payload::append(Const_iterator start, std::size_t n)
    {
        using std::copy_n;
        using std::back_inserter;

        msg->data.reserve(msg->data.size() + n);
        copy_n(start, n, back_inserter(msg->data));
        size(n);   
    }
    

    void Message::Payload::append(const std::string& str)
    {
        using std::copy;
        using std::back_inserter;
        using std::begin;
        using std::end;

        msg->data.reserve(msg->data.size() + str.size());
        copy(begin(str), end(str), back_inserter(msg->data));
        size(str.size());
    }


    void Message::Payload::append(std::string&& str)
    {
        using std::string;
        using std::begin;
        using std::end;
        using std::back_inserter;
        using std::move;

        string temp { move(str) };  // To ensure correct move semantics

        msg->data.reserve(msg->data.size() + temp.size());
        copy(begin(temp), end(temp), back_inserter(msg->data));
        size(temp.size());
    }


    Message::Payload& Message::Payload::operator+=(const std::vector<std::uint8_t>& data)
    {
        append(data);
        return *this;
    }


    Message::Payload& Message::Payload::operator+=(std::vector<std::uint8_t>&& data)
    {
        append(std::move(data));
        return *this;
    }


    Message::Payload& Message::Payload::operator+=(const std::string& str)
    {
        append(str);
        return *this;
    }


    Message::Payload& Message::Payload::operator+=(std::string&& str)
    {
        append(std::move(str));
        return *this;
    }


    void Message::Payload::replace(const std::vector<std::uint8_t>& src)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        msg->data.resize(msg->data.size() + src.size());
        copy(begin(src), end(src), this->begin());
        size(src.size());
    }


    void Message::Payload::replace(std::vector<std::uint8_t>&& src)
    {
        using std::vector;
        using std::uint8_t;
        using std::begin;
        using std::end;
        using std::back_inserter;
        using std::move;

        vector<uint8_t> temp { move(src) };  // To ensure correct move semantics

        msg->data.resize(msg->data.size() + src.size());
        copy(begin(temp), end(temp), this->begin());
        size(temp.size());
    }


    void Message::Payload::replace(Const_iterator start, std::size_t n)
    {
        using std::copy_n;
        using std::back_inserter;

        msg->data.resize(msg->data.size() + n);
        copy_n(start, n, this->begin());
        size(n);   
    }
    

    void Message::Payload::replace(const std::string& str)
    {
        using std::copy;
        using std::back_inserter;
        using std::begin;
        using std::end;

        msg->data.resize(msg->data.size() + str.size());
        copy(begin(str), end(str), this->begin());
        size(str.size());
    }


    void Message::Payload::replace(std::string&& str)
    {
        using std::string;
        using std::begin;
        using std::end;
        using std::back_inserter;
        using std::move;

        string temp { move(str) };  // To ensure correct move semantics

        msg->data.resize(msg->data.size() + temp.size());
        copy(begin(temp), end(temp), this->begin());
        size(temp.size());
    }
    

    Message::Payload& Message::Payload::operator=(const std::vector<std::uint8_t>& data)
    {
        replace(data);
        return *this;
    }


    Message::Payload& Message::Payload::operator=(std::vector<std::uint8_t>&& data)
    {
        replace(std::move(data));
        return *this;
    }


    Message::Payload& Message::Payload::operator=(const std::string& str)
    {
        replace(str);
        return *this;
    }


    Message::Payload& Message::Payload::operator=(std::string&& str)
    {
        replace(std::move(str));
        return *this;
    }


    std::vector<std::uint8_t> Message::Payload::relinquish()
    {
        using std::vector;
        using std::uint8_t;
        using std::copy;
        using std::back_inserter;

        vector<uint8_t> payload_data { this->begin(), this->end() };
        size(0);
        msg->data.resize(msg->header_size());

        return payload_data;
    }


    void Message::Payload::size(std::uint32_t sz)
    {
        auto header = Header::overlay_onto(msg->data.data());
        header->payload_size = htonl(sz);
    }


    std::uint32_t Message::Payload::size() const
    {
        auto header = Header::overlay_onto(msg->data.data());
        return ntohl(header->payload_size);
    }


    Message::Iterator Message::Payload::begin()
    {
        return &(*(msg->data.begin() + msg->header_size()));
    }


    Message::Iterator Message::Payload::end()
    {
        return &(*(msg->data.end()));
    }


    Message::Const_iterator Message::Payload::begin() const
    {
        return &(*((msg->data.cbegin()) + msg->header_size()));
    }


    Message::Const_iterator Message::Payload::end() const
    {
        return &(*(msg->data.cend()));
    }
#endif // #if 0

} // namespace Navtech::Colossus_network_protocol
