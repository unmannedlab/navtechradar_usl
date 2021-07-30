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
constexpr unsigned int largest_valid_message { 255 };
constexpr unsigned int largest_payload { 1'000'000 };

namespace Navtech::Colossus_network_protocol {

    // ---------------------------------------------------------------------------------------------------------
    // Message
    //

    Message::Message() { initialize(); }


    Message::Message(const std::string& ip_addr, Message::ID id) : address { ip_addr }, identity { id } { initialize(); }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, const std::vector<std::uint8_t>& payload_vector) :
        address { ip_addr }, identity { id }
    {
        initialize();
        type(t);
        payload().append(payload_vector);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, std::vector<std::uint8_t>&& payload_vector) :
        address { ip_addr }, identity { id }
    {
        using std::move;

        initialize();
        type(t);
        payload().append(move(payload_vector));
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, const std::string& payload_string) : address { ip_addr }, identity { id }
    {
        initialize();
        type(t);
        payload().append(payload_string);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, std::string&& payload_string) : address { ip_addr }, identity { id }
    {
        using std::move;

        initialize();
        type(t);
        payload().append(move(payload_string));
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, Const_iterator payload_start, std::size_t payload_sz) :
        address { ip_addr }, identity { id }
    {
        using std::move;

        initialize();
        type(t);
        payload().append(payload_start, payload_sz);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, const std::vector<std::uint8_t>& message_vector) :
        address { ip_addr }, identity { id }
    {
        replace(message_vector);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, std::vector<std::uint8_t>&& message_vector) : address { ip_addr }, identity { id }
    {
        replace(std::move(message_vector));
    }


    Message::Message(const std::string& ip_addr, ID id, Const_iterator message_start, std::size_t message_sz) : address { ip_addr }, identity { id }
    {
        replace(message_start, message_sz);
    }


    Message::Message(const std::vector<std::uint8_t>& message_vector) { replace(message_vector); }


    Message::Message(std::vector<std::uint8_t>&& message_vector) { replace(std::move(message_vector)); }


    Message::Message(Const_iterator message_start, std::size_t message_sz) { replace(message_start, message_sz); }


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
        return (is_version_valid() && is_signature_valid() && static_cast<unsigned>(type()) <= largest_valid_message &&
                payload().size() < largest_payload);
    }


    std::size_t Message::size() const { return data.size(); }


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

        return vector<uint8_t> { move(data) };
    }


    Message::Iterator Message::begin() { return &(*data.begin()); }


    Message::Iterator Message::end() { return &(*data.end()); }


    Message::Const_iterator Message::begin() const { return &(*data.cbegin()); }


    Message::Const_iterator Message::end() const { return &(*data.cend()); }


    void Message::initialize()
    {
        using std::begin;
        using std::copy;
        using std::end;

        data.resize(header_size());
        copy(begin(valid_signature), end(valid_signature), signature_begin());
        auto header     = Header::overlay_onto(data.data());
        header->version = version;
    }


    bool Message::is_signature_valid() const
    {
        using std::array;
        using std::begin;
        using std::memcmp;
        using std::uint8_t;

        return (memcmp(valid_signature.begin(), signature_begin(), valid_signature.size()) == 0);
    }

    bool Message::is_version_valid() const
    {
        auto header = Header::overlay_onto(data.data());
        return header->version == version;
    }

    Message::Iterator Message::signature_begin()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[0]);
    }


    Message::Const_iterator Message::signature_begin() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[0]);
    }


    Message::Iterator Message::signature_end()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[signature_sz]); // One-past-the-end
    }


    Message::Const_iterator Message::signature_end() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[signature_sz]);
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
    Message::Payload::Payload(Message& parent) : msg { &parent } { }


    void Message::Payload::append(const std::vector<std::uint8_t>& src)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.reserve(msg->data.size() + src.size());
        copy(begin(src), end(src), back_inserter(msg->data));
        size(src.size());
    }


    void Message::Payload::append(std::vector<std::uint8_t>&& src)
    {
        using std::back_inserter;
        using std::begin;
        using std::end;
        using std::move;
        using std::uint8_t;
        using std::vector;

        vector<uint8_t> temp { move(src) }; // To ensure correct move semantics

        msg->data.reserve(msg->data.size() + src.size());
        copy(begin(temp), end(temp), back_inserter(msg->data));
        size(temp.size());
    }


    void Message::Payload::append(Const_iterator start, std::size_t n)
    {
        using std::back_inserter;
        using std::copy_n;

        msg->data.reserve(msg->data.size() + n);
        copy_n(start, n, back_inserter(msg->data));
        size(n);
    }


    void Message::Payload::append(const std::string& str)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.reserve(msg->data.size() + str.size());
        copy(begin(str), end(str), back_inserter(msg->data));
        size(str.size());
    }


    void Message::Payload::append(std::string&& str)
    {
        using std::back_inserter;
        using std::begin;
        using std::end;
        using std::move;
        using std::string;

        string temp { move(str) }; // To ensure correct move semantics

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
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.resize(msg->data.size() + src.size());
        copy(begin(src), end(src), this->begin());
        size(src.size());
    }


    void Message::Payload::replace(std::vector<std::uint8_t>&& src)
    {
        using std::back_inserter;
        using std::begin;
        using std::end;
        using std::move;
        using std::uint8_t;
        using std::vector;

        vector<uint8_t> temp { move(src) }; // To ensure correct move semantics

        msg->data.resize(msg->data.size() + src.size());
        copy(begin(temp), end(temp), this->begin());
        size(temp.size());
    }


    void Message::Payload::replace(Const_iterator start, std::size_t n)
    {
        using std::back_inserter;
        using std::copy_n;

        msg->data.resize(msg->data.size() + n);
        copy_n(start, n, this->begin());
        size(n);
    }


    void Message::Payload::replace(const std::string& str)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.resize(msg->data.size() + str.size());
        copy(begin(str), end(str), this->begin());
        size(str.size());
    }


    void Message::Payload::replace(std::string&& str)
    {
        using std::back_inserter;
        using std::begin;
        using std::end;
        using std::move;
        using std::string;

        string temp { move(str) }; // To ensure correct move semantics

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
        using std::back_inserter;
        using std::copy;
        using std::uint8_t;
        using std::vector;

        vector<uint8_t> payload_data { this->begin(), this->end() };
        size(0);
        msg->data.resize(msg->header_size());

        return payload_data;
    }


    void Message::Payload::size(std::uint32_t sz)
    {
        auto header          = Header::overlay_onto(msg->data.data());
        header->payload_size = htonl(sz);
    }


    std::uint32_t Message::Payload::size() const
    {
        auto header = Header::overlay_onto(msg->data.data());
        return ntohl(header->payload_size);
    }


    Message::Iterator Message::Payload::begin() { return &(*(msg->data.begin() + msg->header_size())); }


    Message::Iterator Message::Payload::end() { return &(*(msg->data.end())); }


    Message::Const_iterator Message::Payload::begin() const { return &(*((msg->data.cbegin()) + msg->header_size())); }


    Message::Const_iterator Message::Payload::end() const { return &(*(msg->data.cend())); }


} // namespace Navtech::Colossus_network_protocol
