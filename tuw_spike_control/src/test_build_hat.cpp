#include <iostream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_path.hpp>

#include "tuw_spike_control/build_hat_driver.hpp"

static constexpr char FIRMWARE_FILENAME[] = "2025-01-22_firmware.bin";
static constexpr char SIGNATURE_FILENAME[] = "2025-01-22_signature.bin";

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

    std::filesystem::path firmware_directory =
        ament_index_cpp::get_package_share_path("tuw_spike_control") /
        "firmware";
    std::string firmware =
        (firmware_directory / FIRMWARE_FILENAME).string();
    std::string signature =
        (firmware_directory / SIGNATURE_FILENAME).string();

  tuw_spike_control::BuildHatDriver driver;
  driver.set_device("/dev/ttyAMA0", 115200);
  driver.set_firmware(firmware, signature);
  int result = driver.init();

    driver.set_target_velocity_radian_per_sec(1, 0.2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    driver.set_target_velocity_radian_per_sec(1, -0.2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    driver.set_target_velocity_radian_per_sec(1, 0.0);

  driver.deactivate();

  return result;
}
