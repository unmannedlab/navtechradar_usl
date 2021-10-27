#include "radar_client.h"

class Colossus_test_tool
{
public:
    Colossus_test_tool();

    void set_radar_client(std::shared_ptr<Navtech::Radar_client> client) {
        radar_client = client;
    }

    std::shared_ptr<Navtech::Radar_client> get_radar_client() {
        return radar_client;
    }

    void set_radar_ip(std::string ip) {
        radar_ip = ip;
    }

    std::string get_radar_ip() {
        return radar_ip;
    }

    void get_radar_port(uint16_t port) {
        radar_port = port;
    }

    uint16_t get_radar_port() {
        return radar_port;
    }

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    const int sweep_counter_increment{ 1 };
    const int azimuth_start_index{ 1 };
    const int azimuth_increment{ 14 };
    const int azimuth_limit{ 5587 };

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    uint16_t previous_sweep_counter{ 0 };
    uint16_t fft_data_loss_count{ 0 };
    uint16_t azimuth_count{ 0 };

    int azimuth_samples{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };
};