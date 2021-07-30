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
    enum class BufferModes { Off = 0, Average = 1, Max = 2 };

    class Target {
    public:
        Target(double rng, double pow) : range { rng }, power { pow } { }
        double range;
        double power;
    };

    class Azimuth_target {
    public:
        Azimuth_target(std::uint16_t azi, double ang) : azimuth { azi }, angle { ang } { }
        std::uint16_t azimuth;
        double angle;
        std::vector<Target> targets;
    };

    class Peak_finder {
    public:
        Peak_finder();
        void set_target_callback(std::function<void(Azimuth_target&&)> fn = nullptr);
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
        double threshold;
        std::uint8_t bins_to_operate_on;
        std::uint16_t min_bin_to_operate_on;
        bool awaiting_rise;
        std::deque<std::vector<double>> data_buffer;
        BufferModes buffer_mode;
        std::size_t buffer_length;
        std::uint32_t max_peaks_per_azimuth { 10 };

        Configuration_data::Pointer configuration;
        Configuration_data::ProtobufPointer protobuf_configuration;

        std::function<void(Azimuth_target&&)> target_callback = nullptr;

        double peak_resolve(const std::vector<double>& data, const std::uint16_t& peak_bin, const std::uint8_t& bins_to_operate_upon);
        uint16_t find_peak_bin(const std::vector<double>& data,
                               const std::uint16_t& start_bin,
                               const std::uint16_t& end_bin,
                               const std::uint8_t& bins_to_operate_upon);

        void find_peaks(std::uint16_t azimuth, double angle, const std::vector<double>& data);
    };

} // namespace Navtech

#endif // PEAK_FINDER_H
