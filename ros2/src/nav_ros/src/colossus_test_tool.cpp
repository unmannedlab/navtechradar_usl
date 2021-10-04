#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../../cpp_17/network/radar_client.h"
#include "colossus_test_tool.h"

Colossus_test_tool::Colossus_test_tool()
{
    radar_ip = "10.77.2.211";
    radar_port = 6317;
}

void Colossus_test_tool::fft_data_handler(const Navtech::Fft_data::Pointer& data)
{
    azimuth_count++;
    if (data->sweep_counter != previous_sweep_counter + sweep_counter_increment) {
        if ((previous_sweep_counter != std::numeric_limits<uint16_t>::max()) && (data->sweep_counter != 0)) {
            fft_data_loss_count++;
        }
    }
    previous_sweep_counter = data->sweep_counter;

    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        std::cout << "Azimuth count per rotation: " << azimuth_count << std::endl;
        std::cout << "FFT data loss per rotation: " << fft_data_loss_count << std::endl;
        fft_data_loss_count = 0;
        azimuth_count = 0;
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}

void Colossus_test_tool::configuration_data_handler(const Navtech::Configuration_data::Pointer& data)
{
    azimuth_samples = data->azimuth_samples;
    radar_client->start_fft_data();
}