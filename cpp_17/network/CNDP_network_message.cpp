#include <algorithm>
#include <array>
#include <cstring>

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#include "CNDP_network_message.h"

// Constants
//
constexpr unsigned int largest_valid_message { 128 };
constexpr unsigned int largest_payload { 1'048'576 };

constexpr std::array<uint8_t, 16> valid_signature { 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f,
                                                    0x1f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7f, 0xfe, 0xfe };


namespace Navtech::CNDP_network_protocol {

    // ---------------------------------------------------------------------------------------------------------
    // Message
    //

    Message::Message() { initialize(); }


    Message::Message(const std::string& ip_addr, Message::ID id) : address { ip_addr }, identity { id }
    {
        initialize();
    }


    Message::Message(const std::string& ip_addr,
                     Message::ID id,
                     Type t,
                     const std::vector<std::uint8_t>& payload_vector) :
        address { ip_addr },
        identity { id }
    {
        initialize();
        payload().type(t);
        payload().insert(payload_vector);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, std::vector<std::uint8_t>&& payload_vector) :
        address { ip_addr }, identity { id }
    {
        using std::move;

        initialize();
        payload().type(t);
        payload().insert(move(payload_vector));
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, const std::string& payload_string) :
        address { ip_addr }, identity { id }
    {
        initialize();
        payload().type(t);
        payload().insert(payload_string);
    }


    Message::Message(const std::string& ip_addr, Message::ID id, Type t, std::string&& payload_string) :
        address { ip_addr }, identity { id }
    {
        using std::move;

        initialize();
        payload().type(t);
        payload().insert(move(payload_string));
    }


    Message::Message(const std::string& ip_addr,
                     Message::ID id,
                     Type t,
                     Const_iterator payload_start,
                     std::size_t payload_sz) :
        address { ip_addr },
        identity { id }
    {
        using std::move;

        initialize();
        payload().type(t);
        payload().insert(payload_start, payload_sz);
    }


    Message::ID Message::id() const { return identity; }


    void Message::id(Message::ID new_id) { identity = new_id; }


    const Utility::IP_address& Message::ip_address() const { return address; }


    void Message::ip_address(const std::string& ip_addr) { address = ip_addr; }


    bool Message::is_valid() const
    {
        return (is_signature_valid() && static_cast<unsigned>(payload().type()) <= largest_valid_message &&
                payload().size() <= largest_payload);
    }


    std::size_t Message::size() const { return data.size(); }


    void Message::replace(const std::vector<std::uint8_t>& src) { data = src; }


    void Message::replace(std::vector<std::uint8_t>&& src)
    {
        using std::move;

        data = move(src);
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
    }


    bool Message::is_signature_valid() const
    {
        using std::array;
        using std::begin;
        using std::memcmp;
        using std::uint8_t;

        return (memcmp(valid_signature.begin(), signature_begin(), valid_signature.size()) == 0);
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
        return &(header->signature[Header::signature_sz]); // One-past-the-end
    }


    Message::Const_iterator Message::signature_end() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[Header::signature_sz]);
    }


    // ---------------------------------------------------------------------------------------------------------
    // Payload
    //
    Message::Payload::Payload(Message& parent) : msg { &parent } { }


    void Message::Payload::insert(const std::vector<std::uint8_t>& src)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.reserve(msg->data.size() + src.size());
        copy(begin(src), end(src), back_inserter(msg->data));
        size(src.size());
    }


    void Message::Payload::insert(std::vector<std::uint8_t>&& src)
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


    void Message::Payload::insert(Const_iterator start, std::size_t n)
    {
        using std::back_inserter;
        using std::copy_n;

        msg->data.reserve(msg->data.size() + n);
        copy_n(start, n, back_inserter(msg->data));
        size(n);
    }


    void Message::Payload::insert(const std::string& str)
    {
        using std::back_inserter;
        using std::begin;
        using std::copy;
        using std::end;

        msg->data.reserve(msg->data.size() + str.size());
        copy(begin(str), end(str), back_inserter(msg->data));
        size(str.size());
    }


    void Message::Payload::insert(std::string&& str)
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


    std::string Message::Payload::as_string() const
    {
        using std::string;

        string ret_val { begin(), end() };
        return ret_val;
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


    Message::Type Message::Payload::type() const
    {
        auto header = Message::Header::overlay_onto(msg->data.data());
        return header->id;
    }


    void Message::Payload::type(Message::Type t)
    {
        auto header = Message::Header::overlay_onto(msg->data.data());
        header->id  = t;
    }


    Message::Iterator Message::Payload::begin() { return &(*(msg->data.begin() + msg->header_size())); }


    Message::Iterator Message::Payload::end() { return &(*(msg->data.end())); }


    Message::Const_iterator Message::Payload::begin() const { return &(*((msg->data.cbegin()) + msg->header_size())); }


    Message::Const_iterator Message::Payload::end() const { return &(*(msg->data.cend())); }


} // namespace Navtech::CNDP_network_protocol
