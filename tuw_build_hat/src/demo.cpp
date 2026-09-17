#include <filesystem>
#include <iostream>

#include <boost/program_options.hpp>

#include "tuw_build_hat/build_hat.hpp"
#include "tuw_build_hat/motor.hpp"

namespace po = boost::program_options;

static constexpr char FIRMWARE_FILENAME[] = "firmware.bin";
static constexpr char SIGNATURE_FILENAME[] = "signature.bin";

/**
 * @brief Demo program that drives two Build HAT motors through the BuildHat driver.
 *
 * Opens the serial connection to a Raspberry Pi Build HAT, uploads firmware if needed,
 * registers a left and a right Motor, spins the left motor forward then backward for a
 * few seconds while holding the right motor at a fixed speed, and finally deactivates
 * both motors before exiting.
 *
 * @param argc argument count
 * @param argv argument values, parsed as the options below (run with --help to list them):
 *             -p/--serial-port, -f/--firmware-dir, -s/--serial_log, -m/--msgs_log, -l/--loglevel
 * @return the result of BuildHat::init() (BuildHat::OK on success, BuildHat::ERROR otherwise)
 *
 * @par Example
 * @code{.sh}
 * # Run with defaults (serial port /dev/ttyUSB0, info-level logging)
 * ./demo --firmware-dir ./ws02/src/tuw_spike/tuw_build_hat/firmware -r 6.24 -t 5 -p 0
 *
 * # Run against a different debug logging and a custom firmware directory
 * ./demo --serial-device /dev/ttyUSB0 --firmware-dir ./ws02/src/tuw_spike/tuw_build_hat/firmware --serial_log /tmp/build_had_serial.log  --loglevel 0
 *
 * # Show all available options
 * ./demo --help
 * @endcode
 */
int main(int argc, char **argv) {
    std::string serial_port;
    std::string firmware_directory;
    std::string filename_serial_log;
    std::string filename_msgs_log;
    int loglevel;
    int port = 0;
    int time = 5;
    double rad_per_sec = M_PI;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("serial-device,d", po::value<std::string>(&serial_port)->default_value("/dev/ttyUSB0"), "serial port the Build HAT is connected to")
        ("firmware-dir,f", po::value<std::string>(&firmware_directory)->default_value("/home/markus/projects/imr2026/ws02/src/tuw_spike/tuw_build_hat/firmware"), "directory containing the firmware.bin and signature.bin files")
        ("serial_log,s", po::value<std::string>(&filename_serial_log)->default_value(""), "file to log the serial communication to")
        ("msgs_log,m", po::value<std::string>(&filename_msgs_log)->default_value(""), "file to log the msgs to if empty it will be printed to std::cout")
        ("loglevel,l", po::value<int>(&loglevel)->default_value(1), "log level: 0 = debug, 1 = info, 2 = warning, 3 = error")
        ("port,p", po::value<int>(&port)->default_value(0), "Build HAT port the motor is connected to")
        ("time,t", po::value<int>(&time)->default_value(5), "seconds to hold each target speed for")
        ("rad-per-sec,r", po::value<double>(&rad_per_sec)->default_value(M_PI), "target speed in radians per second");

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

    auto motor = std::make_shared<tuw_build_hat::Motor>(port);
    motor->selrate(10);

    tuw_build_hat::BuildHat driver;
    driver.set_device(serial_port, 115200);
    driver.set_firmware(firmware, signature);
    driver.set_logfile_serial(filename_serial_log);
    driver.set_logfile_msgs(filename_msgs_log);
    driver.set_loglevel(loglevel);
    driver.add_device(motor);
    int result = driver.init();

    motor->set_target_radian_per_sec(rad_per_sec, 0.003, 0.01, 0.00);
    driver.commit();
    std::this_thread::sleep_for(std::chrono::seconds(time));
    motor->set_target_radian_per_sec(-rad_per_sec, 0.003, 0.01, 0.00);
    driver.commit();
    std::this_thread::sleep_for(std::chrono::seconds(time));
    motor->set_target_radian_per_sec(0.0);
    driver.commit();

    driver.deactivate();

    return result;
}
