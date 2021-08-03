#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../navigation/peak_finder.h"

using namespace Navtech;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::NiceMock;
using ::testing::Return;

constexpr std::array<std::uint8_t, 32> no_peak { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

constexpr std::array<std::uint8_t, 32> single_peak { 0, 0, 0, 0, 0, 0, 80, 100, 120, 100, 80, 0, 0, 0, 0, 0,
                                                     0, 0, 0, 0, 0, 0, 0,  0,   0,   0,   0,  0, 0, 0, 0, 0 };

constexpr std::array<std::uint8_t, 32> twin_peaks { 0, 0, 0, 0, 0, 0, 80, 100, 120, 100, 80,  0,  0, 0, 0, 0,
                                                    0, 0, 0, 0, 0, 0, 0,  80,  100, 120, 100, 80, 0, 0, 0, 0 };

class given_a_peak_finder : public ::testing::Test {
public:
    given_a_peak_finder()
    {
        configuration                = allocate_shared<Configuration_data>();
        configuration->range_in_bins = 32;
        configuration->range_gain    = 1;
        configuration->range_offset  = 0;
        protobuf_configuration       = allocate_shared<Colossus::Protobuf::ConfigurationData>();
        protobuf_configuration->set_rangeresolutionmetres(0.25);
    }

protected:
    Configuration_data::Pointer configuration;
    Configuration_data::ProtobufPointer protobuf_configuration;
};

TEST_F(given_a_peak_finder, WhenCallingFindPeakBinWithNoDataAboveThresholdShouldReturnRangeInBins)
{
    Peak_finder pf {};

    auto target_found                                 = false;
    std::function<void(const Azimuth_target&)> lambda = [&target_found](const Azimuth_target& target) {
        target_found = true;
        return;
    };

    auto threshold             = 90.0;             // Threshold in dB
    auto bins_to_operate_on    = 4;                // Radar bins window size to search for peaks in
    auto start_bin             = 1;                // Start Bin
    auto buffer_mode           = BufferModes::off; // Buffer mode should only be used with a staring radar
    auto buffer_length         = 10;               // Buffer Length
    auto max_peaks_per_azimuth = 2;                // Maximum number of peaks to find in a single azimuth

    pf.set_target_callback(lambda);
    pf.configure(configuration,
                 protobuf_configuration,
                 threshold,
                 bins_to_operate_on,
                 start_bin,
                 buffer_mode,
                 buffer_length,
                 max_peaks_per_azimuth);

    Fft_data::Pointer fft_data  = allocate_shared<Fft_data>();
    fft_data->angle             = 50.4;
    fft_data->azimuth           = 1224;
    fft_data->sweep_counter     = 2;
    fft_data->ntp_seconds       = 0;
    fft_data->ntp_split_seconds = 0;
    fft_data->data              = std::vector<std::uint8_t>(no_peak.begin(), no_peak.end());

    pf.fft_data_handler(fft_data);
    ASSERT_FALSE(target_found);
}


TEST_F(given_a_peak_finder, WhenCallingFindPeakBinWithASinglePeakAboveThresholdShouldReturnRangeInBins)
{
    Peak_finder pf {};

    auto target_count                                 = 0;
    auto target_found                                 = false;
    std::function<void(const Azimuth_target&)> lambda = [&target_found, &target_count](const Azimuth_target& target) {
        target_found = true;
        target_count = target.targets.size();
        ASSERT_EQ(2.0, target.targets[0].range);
        return;
    };

    auto threshold             = 90.0;             // Threshold in dB
    auto bins_to_operate_on    = 4;                // Radar bins window size to search for peaks in
    auto start_bin             = 1;                // Start Bin
    auto buffer_mode           = BufferModes::off; // Buffer mode should only be used with a staring radar
    auto buffer_length         = 10;               // Buffer Length
    auto max_peaks_per_azimuth = 2;                // Maximum number of peaks to find in a single azimuth

    pf.set_target_callback(lambda);
    pf.configure(configuration,
                 protobuf_configuration,
                 threshold,
                 bins_to_operate_on,
                 start_bin,
                 buffer_mode,
                 buffer_length,
                 max_peaks_per_azimuth);

    Fft_data::Pointer fft_data  = allocate_shared<Fft_data>();
    fft_data->angle             = 50.4;
    fft_data->azimuth           = 1224;
    fft_data->sweep_counter     = 2;
    fft_data->ntp_seconds       = 0;
    fft_data->ntp_split_seconds = 0;
    fft_data->data              = std::vector<std::uint8_t>(single_peak.begin(), single_peak.end());

    pf.fft_data_handler(fft_data);
    ASSERT_TRUE(target_found);
    ASSERT_EQ(1, target_count);
}


TEST_F(given_a_peak_finder, WhenCallingFindPeakBinWithATwinPeaksAboveThresholdShouldReturnRangeInBins)
{
    Peak_finder pf {};

    auto target_count                                 = 0;
    auto target_found                                 = false;
    std::function<void(const Azimuth_target&)> lambda = [&target_found, &target_count](const Azimuth_target& target) {
        target_found = true;
        target_count = target.targets.size();
        ASSERT_EQ(2.0, target.targets[0].range);
        ASSERT_EQ(6.25, target.targets[1].range);
        return;
    };

    auto threshold             = 90.0;             // Threshold in dB
    auto bins_to_operate_on    = 4;                // Radar bins window size to search for peaks in
    auto start_bin             = 1;                // Start Bin
    auto buffer_mode           = BufferModes::off; // Buffer mode should only be used with a staring radar
    auto buffer_length         = 10;               // Buffer Length
    auto max_peaks_per_azimuth = 2;                // Maximum number of peaks to find in a single azimuth

    pf.set_target_callback(lambda);
    pf.configure(configuration,
                 protobuf_configuration,
                 threshold,
                 bins_to_operate_on,
                 start_bin,
                 buffer_mode,
                 buffer_length,
                 max_peaks_per_azimuth);

    Fft_data::Pointer fft_data  = allocate_shared<Fft_data>();
    fft_data->angle             = 50.4;
    fft_data->azimuth           = 1224;
    fft_data->sweep_counter     = 2;
    fft_data->ntp_seconds       = 0;
    fft_data->ntp_split_seconds = 0;
    fft_data->data              = std::vector<std::uint8_t>(twin_peaks.begin(), twin_peaks.end());

    pf.fft_data_handler(fft_data);
    ASSERT_TRUE(target_found);
    ASSERT_EQ(2, target_count);
}
