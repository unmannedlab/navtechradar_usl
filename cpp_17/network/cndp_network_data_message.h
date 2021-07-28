// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to
// https://opensource.org/licenses/MIT for full license details.
//

#ifndef CNDP_NETWORK_DATA_MESSAGE_H
#define CNDP_NETWORK_DATA_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#include <Pointer_types.h>

namespace Navtech {
    constexpr uint8_t ndmsync_header_length { 16 };
    constexpr uint8_t ndm_signature_length { ndmsync_header_length * sizeof(uint8_t) };
    constexpr uint32_t max_payload_size { 1048576 };
    constexpr uint8_t ndm_signature_offset { 0 };
    constexpr uint8_t ndm_version_offset { ndm_signature_offset + ndm_signature_length };
    constexpr uint8_t ndm_version_length { sizeof(uint8_t) };
    constexpr uint8_t ndm_message_id_offset { ndm_version_offset + ndm_version_length };
    constexpr uint8_t ndm_message_id_length { sizeof(uint8_t) };
    constexpr uint8_t ndm_payload_size_offset { ndm_message_id_offset + ndm_message_id_length };
    constexpr uint8_t ndm_payload_size_length { sizeof(uint32_t) };
    constexpr uint8_t ndm_header_length { ndm_signature_length + ndm_version_length + ndm_message_id_length +
                                          ndm_payload_size_length };
    constexpr uint8_t ndm_protocol_min_version { 1 };
    constexpr uint8_t ndm_protocol_version { 1 };

    constexpr uint8_t ndm_signature_bytes[ndm_signature_length] { 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f,
                                                                  0x1f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7f, 0xfe, 0xfe };

    enum class CNDPNetworkDataMessageType : uint8_t {
        Invalid                   = 0,
        KeepAlive                 = 1,
        Configuration             = 10,
        ConfigurationRequest      = 20,
        StartFFTData              = 21,
        StopFFTData               = 22,
        StartHealthMsgs           = 23,
        StopHealthMsgs            = 24,
        ReCalibrateRfHealth       = 25,
        StartTracks               = 26,
        StopTracks                = 27,
        TransmitOn                = 28,
        TransmitOff               = 29,
        FFTData                   = 30,
        HighPrecisionFFTData      = 31,
        Health                    = 40,
        ContourUpdate             = 50,
        TrackUpdate               = 60,
        TrackerConfiguration      = 70,
        TrackerPlaybackCommand    = 71,
        TrackerSaveClutterMap     = 72,
        TrackerLegacyHealthUnits  = 73,
        TrackerDataSourceUpdate   = 74,
        TrackerDistributionUpdate = 75,
        SystemRestart             = 76,
        LoggingLevels             = 90,
        LoggingLevelsRequest      = 100,
        SetAutoTune               = 110,
        StartNavData              = 120,
        StopNavData               = 121,
        SetNavThreshold           = 122,
        NavigationData            = 123,
        SetNavRangeOffsetAndGain  = 124,
        CalibrateAccelerometer    = 125,
        StartAccelerometer        = 126,
        StopAccelerometer         = 127,
        AccelerometerData         = 128,
        StartNonContourFFTData    = 140,
        SetNavBufferModeAndLength = 141,
        SetNavBinOperation        = 142,
        NavigationAlarmData       = 143,
        SetNavAreaRules           = 144,
        NavRadarReset             = 145,
        NavRadarHalt              = 146,
        KeyExchange               = 200,
        ContentEncryptionKey      = 201,
        EncryptedData             = 202
    };

#pragma pack(push)
#pragma pack(1)
    struct Network_data_header
    {
        uint8_t signature[ndmsync_header_length];
        uint8_t version;
        uint8_t message_id;
        uint32_t payload_size;
        void Init()
        {
            std::memcpy(&signature, &ndm_signature_bytes, sizeof(signature));
            version      = ndm_protocol_version;
            message_id   = static_cast<uint8_t>(CNDPNetworkDataMessageType::Invalid);
            payload_size = 0;
        }
        void Set_message_id(CNDPNetworkDataMessageType messageType) { message_id = static_cast<uint8_t>(messageType); }
        void Set_payload_length(uint32_t length) { payload_size = htonl(length); }
        const uint32_t Payload_length() const { return ntohl(payload_size); }
        const uint32_t Header_length() const { return ndm_header_length; }
        const bool Header_is_valid() const
        {
            auto result = true;

            // Ensure signature is valid
            for (auto i = 0u; i < ndm_signature_length; i++) {
                result &= ndm_signature_bytes[i] == signature[i];
            }

            // Ensure version number is in valid range
            result &= (version >= ndm_protocol_min_version && version <= ndm_protocol_version);

            // Ensure Payload Length is not too large
            result &= (Payload_length() <= max_payload_size);

            return result;
        }
        const std::vector<uint8_t> to_data() const
        {
            std::vector<uint8_t> headerData(ndm_header_length);
            std::memcpy(&headerData[0], &signature, sizeof(signature));
            std::memcpy(&headerData[ndm_version_offset], &version, sizeof(version));
            std::memcpy(&headerData[ndm_message_id_offset], &message_id, sizeof(message_id));
            std::memcpy(&headerData[ndm_payload_size_offset], &payload_size, sizeof(payload_size));
            return headerData;
        }
    };
#pragma pack(pop)

    class Network_data_message {
    public:
        explicit Network_data_message(const Network_data_header header) :
            Network_data_message(header, std::vector<uint8_t>())
        { }

        explicit Network_data_message(const Network_data_header header,
                                      const uint8_t* payload,
                                      const std::size_t size) :
            Network_data_message(header, std::vector<uint8_t>(payload, payload + size))
        { }

        explicit Network_data_message(const Network_data_header header, const std::vector<uint8_t>& data) :
            header(header), payload(data)
        { }

        ~Network_data_message()
        {
            payload.clear();
            std::vector<uint8_t>(payload).swap(payload);
        }

        explicit Network_data_message(const Network_data_message&) = delete;
        Network_data_message& operator=(const Network_data_message&) = delete;

        const std::vector<uint8_t> Payload() const { return payload; }
        const CNDPNetworkDataMessageType Message_id() const
        {
            return static_cast<CNDPNetworkDataMessageType>(header.message_id);
        }
        const uint32_t Payload_size() const { return header.Payload_length(); }
        const uint8_t Version() const { return header.version; }
        const bool Message_valid() const { return header.Header_is_valid(); }
        std::vector<uint8_t> Message_data()
        {
            auto new_header = header.to_data();

            if (payload.size() > 0) new_header.insert(new_header.end(), payload.begin(), payload.end());

            return new_header;
        }

    protected:
        const Network_data_header header;
        std::vector<uint8_t> payload;
    };

    typedef Shared_owner<Network_data_message> CNDPDataMessagePtr_t;

#pragma pack(push)
#pragma pack(1)
    typedef struct CNDPNetworkDataHeader : public Network_data_header
    {
    } CNDPNetworkDataHeaderStruct;
#pragma pack(pop)

} // namespace Navtech

#endif // CNDP_NETWORK_DATA_MESSAGE_H
