#ifndef CP_NETWORK_MESSAGE_H
#define CP_NETWORK_MESSAGE_H

#include <cstdint>
#include <vector>

#include "../utility/IP_address.h"
#include "../utility/Pointer_types.h"

namespace Navtech::CNDP_network_protocol {

    // --------------------------------------------------------------------------------------------------
    // The Message class provides an interface for storing and accessing
    // CP Network messages.
    //
    class Message {
    public:
        enum class Type : std::uint8_t {
            invalid                   = 0,
            keepalive                 = 1,
            configuration             = 10,
            configurationrequest      = 20,
            startfftdata              = 21,
            stopfftdata               = 22,
            starthealthmsgs           = 23,
            stophealthmsgs            = 24,
            recalibraterfhealth       = 25,
            starttracks               = 26,
            stoptracks                = 27,
            transmiton                = 28,
            transmitoff               = 29,
            fftdata                   = 30,
            highprecisionfftdata      = 31,
            health                    = 40,
            contourupdate             = 50,
            trackupdate               = 60,
            trackerconfiguration      = 70,
            trackerplaybackcommand    = 71,
            trackersavecluttermap     = 72,
            trackerlegacyhealthunits  = 73,
            trackerdatasourceupdate   = 74,
            trackerdistributionupdate = 75,
            systemrestart             = 76,
            logginglevels             = 90,
            logginglevelsrequest      = 100,
            setautotune               = 110,
            startnavdata              = 120,
            stopnavdata               = 121,
            setnavthreshold           = 122,
            navigationdata            = 123,
            setnavrangeoffsetandgain  = 124,
            calibrateaccelerometer    = 125,
            startaccelerometer        = 126,
            stopaccelerometer         = 127,
            accelerometerdata         = 128,
            startnoncontourfftdata    = 140,
            setnavbuffermodeandlength = 141,
            setnavbinoperation        = 142,
            navigationalarmdata       = 143,
            setnavarearules           = 144,
            navradarreset             = 145,
            navradarhalt              = 146,
            keyexchange               = 200,
            contentencryptionkey      = 201,
            encrypteddata             = 202
        };

        using ID             = std::uint32_t;
        using Iterator       = std::uint8_t*;
        using Const_iterator = const std::uint8_t*;

    private:
        // Payload provides an abstract interface into the payload
        // part of the message.  A Payload object is just an overlay
        // into the message's data; it contains no data itself.
        //
        class Payload {
        public:
            Payload(Message& parent);

            void insert(const std::vector<std::uint8_t>& data);
            void insert(std::vector<std::uint8_t>&& data);
            void insert(Const_iterator start, std::size_t n);
            void insert(const std::string& str);
            void insert(std::string&& str);

            std::string as_string() const;
            std::vector<std::uint8_t> relinquish();

            std::uint32_t size() const;
            Type type() const;
            void type(Type t);

            Iterator begin();
            Const_iterator begin() const;
            Iterator end();
            Const_iterator end() const;

        protected:
            void size(std::uint32_t sz);

        private:
            Association_to<Message> msg;
        };

        friend class Payload;

    public:
        Message();
        Message(const std::string& ip_addr, ID id);
        Message(const std::string& ip_addr, ID id, Type t, const std::vector<std::uint8_t>& payload_vector);
        Message(const std::string& ip_addr, ID id, Type t, std::vector<std::uint8_t>&& payload_vector);
        Message(const std::string& ip_addr, ID id, Type t, const std::string& payload_string);
        Message(const std::string& ip_addr, ID id, Type t, std::string&& payload_string);
        Message(const std::string& ip_addr, ID id, Type t, Const_iterator payload_start, std::size_t payload_sz);

        ID id() const;
        void id(ID new_id);

        const Utility::IP_address& ip_address() const;
        void ip_address(const std::string& ip_addr_str);

        // Network message interface - header + payload information
        //
        bool is_valid() const;
        std::size_t size() const;
        static constexpr std::size_t header_size() { return sizeof(Header); }

        void replace(const std::vector<std::uint8_t>& src);
        void replace(std::vector<std::uint8_t>&& src);
        std::vector<std::uint8_t> relinquish();

        Iterator begin();
        Const_iterator begin() const;
        Iterator end();
        Const_iterator end() const;

        Payload payload() { return Payload { *this }; }
        const Payload payload() const { return Payload { *(const_cast<Message*>(this)) }; }

    protected:
        void initialize();
        bool is_signature_valid() const;

        Iterator signature_begin();
        Const_iterator signature_begin() const;
        Iterator signature_end();
        Const_iterator signature_end() const;

    private:
// Header is for overlay only - header information is stored in
// data vector, contigous with the payload.
//
#pragma pack(1)
        struct Header
        {
            static constexpr std::size_t signature_sz { 16 };

            std::uint8_t signature[signature_sz];
            std::uint8_t version;
            Type id;
            std::uint32_t payload_size;

            static Header* overlay_onto(std::uint8_t* from) { return reinterpret_cast<Header*>(from); }
            static const Header* overlay_onto(const std::uint8_t* from)
            {
                return reinterpret_cast<const Header*>(from);
            }
        };
#pragma pack()

        static_assert(sizeof(Message::Header) == 22);

        Utility::IP_address address {};
        ID identity {};
        std::vector<std::uint8_t> data {};
    };

} // namespace Navtech::CNDP_network_protocol

#endif // CP_NETWORK_MESSAGE_H
