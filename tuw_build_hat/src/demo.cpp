#include <filesystem>
#include <iostream>

#include <boost/program_options.hpp>

#include "tuw_build_hat/build_hat.hpp"
#include "tuw_build_hat/motor.hpp"

namespace po = boost::program_options;

static constexpr char FIRMWARE_FILENAME[] = "firmware.bin";
static constexpr char SIGNATURE_FILENAME[] = "signature.bin";

int main(int argc, char **argv) {
    std::string serial_port;
    std::string firmware_directory;
    std::string filename_serial_log;
    std::string filename_msgs_log;
    int loglevel;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("serial-port,p", po::value<std::string>(&serial_port)->default_value("/dev/ttyUSB0"), "serial port the Build HAT is connected to")
        ("firmware-dir,f", po::value<std::string>(&firmware_directory)->default_value("/home/markus/projects/imr2026/ws02/src/tuw_spike/tuw_build_hat/firmware"), "directory containing the firmware.bin and signature.bin files")
        ("serial_log,s", po::value<std::string>(&filename_serial_log)->default_value("/tmp/spike_serial.log"), "file to log the serial communication to")
        ("msgs_log,m", po::value<std::string>(&filename_msgs_log)->default_value(""), "file to log the msgs to if empty it will be printed to std::cout")
        ("loglevel,l", po::value<int>(&loglevel)->default_value(1), "log level: 0 = debug, 1 = info, 2 = warning, 3 = error");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    if (loglevel < 0 || loglevel > 3) {
        std::cerr << "loglevel must be between 0 and 3" << std::endl;
        return 1;
    }

    std::string firmware = (std::filesystem::path(firmware_directory) / FIRMWARE_FILENAME).string();
    std::string signature = (std::filesystem::path(firmware_directory) / SIGNATURE_FILENAME).string();

    auto left_motor = std::make_shared<tuw_build_hat::Motor>(0);
    auto right_motor = std::make_shared<tuw_build_hat::Motor>(1);
    left_motor->selrate(10);
    right_motor->selrate(10);

    tuw_build_hat::BuildHat driver;
    driver.set_device(serial_port, 115200);
    driver.set_firmware(firmware, signature);
    driver.set_logfile_serial(filename_serial_log);
    driver.set_logfile_msgs(filename_msgs_log);
    driver.set_loglevel(loglevel);
    driver.add_device(left_motor);
    driver.add_device(right_motor);
    int result = driver.init();

    right_motor->set_target_radian_per_sec(M_PI/8.0, 0.003, 0.01, 0.00);
    left_motor->set_target_radian_per_sec(M_PI*2.);
    driver.commit();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    left_motor->set_target_radian_per_sec(-M_PI*2.);
    driver.commit();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    left_motor->set_target_radian_per_sec(0.0);
    driver.commit();

    driver.deactivate();

    return result;
}
