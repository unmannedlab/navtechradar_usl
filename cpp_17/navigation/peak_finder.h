#ifndef PEAK_FINDER_H
#define PEAK_FINDER_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "../network/radar_client.h"

namespace Navtech {

    // Buffer modes are really only useful with a staring radar
    //
    enum class BufferModes { off = 0, average = 1, max = 2 };

    struct Target
    {
        Target(double rng, double pow) : range { rng }, power { pow } { }
        double range;
        double power;
    };

    struct Azimuth_target
    {
        Azimuth_target(std::uint16_t azi, double ang, std::uint32_t seconds, std::uint32_t split_seconds) :
            azimuth { azi }, angle { ang }, ntp_seconds(seconds), ntp_split_seconds(split_seconds)
        { }
        std::uint16_t azimuth;
        double angle;
        std::vector<Target> targets;
        std::uint32_t ntp_seconds { 0 };
        std::uint32_t ntp_split_seconds { 0 };
    };

    class Peak_finder {
    public:
        void set_target_callback(std::function<void(const Azimuth_target&)> fn = nullptr);
        void fft_data_handler(const Fft_data::Pointer& fft_data);
        void configure(const Configuration_data::Pointer& data,
                       const Configuration_data::ProtobufPointer& protobuf_configuration,
                       double threshold,
                       std::uint8_t bins_to_operate_upon,
                       std::uint16_t min_bin_to_operate_upon,
                       BufferModes mode,
                       std::size_t buf_length,
                       std::uint32_t max_peaks_per_azi);

    private:
        double threshold { 0 };
        std::uint8_t bins_to_operate_on { 4 };
        std::uint16_t min_bin_to_operate_on { 50 };
        bool awaiting_rise { false };
        std::deque<std::vector<double>> data_buffer {};
        BufferModes buffer_mode { BufferModes::off };
        std::size_t buffer_length { 10 };
        std::uint32_t max_peaks_per_azimuth { 10 };

        Configuration_data::Pointer configuration;
        Configuration_data::ProtobufPointer protobuf_configuration;

        std::function<void(const Azimuth_target&)> target_callback = nullptr;

        double peak_resolve(const std::vector<double>& data,
                            const std::uint16_t& peak_bin,
                            const std::uint8_t& bins_to_operate_upon);
        uint16_t find_peak_bin(const std::vector<double>& data,
                               const std::uint16_t& start_bin,
                               const std::uint16_t& end_bin,
                               const std::uint8_t& bins_to_operate_upon);

        void find_peaks(std::uint16_t azimuth,
                        double angle,
                        std::uint32_t ntp_seconds,
                        std::uint32_t ntp_split_seconds,
                        const std::vector<double>& data);
    };

} // namespace Navtech

#endif // PEAK_FINDER_H
