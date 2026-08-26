#include <cstdio>
#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include <string>
#include <math.h>
#include <fstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

// run with "docker -c lego0 compose up --build hardware"
// hardware:
//      extend: runtime
//      command: ros2 run serial_if serial_if

// Declare the io_context and serial port globally
boost::asio::io_context io_context;
boost::asio::serial_port serial(io_context);
boost::asio::streambuf buf;

std::vector<unsigned char> readFile4(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(file), {});
    return buffer;
}

std::string read_serial2() {
    boost::asio::read_until(serial, buf, '\n');
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    std::cout << "Read: " << line << std::endl;
    return line;
}

void getPrompt() {
    std::string bootloader_str = "BHBL>";
    while (true) {
        std::string line = read_serial2();
        if (line.find(bootloader_str) != std::string::npos) {
            break;
        }
    }
}

uint32_t checksum(const std::vector<uint8_t>& data) {
    uint32_t u = 1;
    for (size_t i = 0; i < data.size(); ++i) {
        if ((u & 0x80000000) != 0) {
            u = (u << 1) ^ 0x1d872b41;
        } else {
            u = u << 1;
        }
        u = (u ^ data[i]) & 0xFFFFFFFF;
    }
    return u;
}

void loadFirmware() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("tuw_spike_control");
    std::string firmwarePath = package_share_directory + "/firmware/firmware.bin";
    std::string signaturePath = package_share_directory + "/firmware/signature.bin";

    std::vector<unsigned char> firmware = readFile4(firmwarePath);
    std::vector<unsigned char> signature = readFile4(signaturePath);

    boost::asio::write(serial, boost::asio::buffer("clear\r", 6));
    getPrompt();

    std::string loadCommand = "load " + std::to_string(std::filesystem::file_size(firmwarePath)) + " " + std::to_string(checksum(firmware)) + "\r";
    boost::asio::write(serial, boost::asio::buffer(loadCommand));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    boost::asio::write(serial, boost::asio::buffer("\x02", 1));
    boost::asio::write(serial, boost::asio::buffer(firmware));
    boost::asio::write(serial, boost::asio::buffer("\x03\r", 2));
    getPrompt();    

    std::string signatureCommand = "signature " + std::to_string(std::filesystem::file_size(signaturePath)) + "\r";   
    boost::asio::write(serial, boost::asio::buffer(signatureCommand));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    boost::asio::write(serial, boost::asio::buffer("\x02", 1));
    boost::asio::write(serial, boost::asio::buffer(signature));
    boost::asio::write(serial, boost::asio::buffer("\x03\r", 2));
    getPrompt();

    boost::asio::write(serial, boost::asio::buffer("reboot\r", 7));
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

}



int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;


    // Define serial port settings
    std::string port_name = "/dev/ttyS0";
    unsigned int baud_rate = 115200;

    // Open the serial port
    serial.open(port_name);

    // Set the baud rate
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    boost::asio::write(serial, boost::asio::buffer("version\r", 8));

    // Check if we're in the bootloader or the firmware
    int emptydata = 0;
    int incdata = 0;
    while (true) {
        std::string line = read_serial2();
        if (line.empty()) {
            ++emptydata;
            if (emptydata > 3) {
                break;
            } else {
                continue;
            }
        }

        if (line.find("Firmware version: ") != std::string::npos) {
            std::cout << "Cout: Firmware is already loaded" << std::endl;
            // Parse version here and handle it
            break;
        } else if (line.find("BuildHAT bootloader version") != std::string::npos) {
            std::cout << "Cout: Bootloader" << std::endl;
            loadFirmware();
            break;
        } else {
            ++incdata;
            if (incdata > 5) {
                break;
            } else {
                boost::asio::write(serial, boost::asio::buffer("version\r", 8));
            }
        }
    }

    std::string cmd = "echo 0;\r";
    std::cout << cmd << std::endl; 
    boost::asio::write(serial, boost::asio::buffer(cmd));

    cmd = "plimit 1; port 0; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid_diff 0 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
    std::cout << cmd << std::endl; 
    boost::asio::write(serial, boost::asio::buffer(cmd));

    cmd = "port 1; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid_diff 1 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
    std::cout << cmd << std::endl; 
    boost::asio::write(serial, boost::asio::buffer(cmd));


    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cmd = "port 0; set -1; port 1; set 1;\r";
    std::cout << cmd << std::endl; 
    boost::asio::write(serial, boost::asio::buffer(cmd));

    std::string p2_str = "P0C0";
    std::string p3_str = "P1C0";
    int p2_speed = 0;
    int p2_apos = 0;
    int p3_speed = 0;
    int p3_apos = 0;

    // Read data from serial port
    for (int i = 0; i < 200; i++) {
        // Buffer to store incoming data
        std::vector<char> buffer(128);  // Adjust size as needed
        // Read data from serial port
        boost::system::error_code error;
        
        std::size_t bytes_read = 128;

        while(bytes_read == 128) {
        bytes_read = serial.read_some(boost::asio::buffer(buffer), error);

        std::string current;
        if (error) {
            std::cerr << "Error reading from serial port: " << error.message() << std::endl;
        } else {
            // process buffer
            for (char c : buffer) {
                if (c == '\n') {
                    // std::cout << "Read: " << current << std::endl;

                    // line complete: check for completeness
                    if (current.size() >= 5) {
                        // newer info possibly available
                        auto first_space = current.find(' ');
                        auto second_space = current.find(' ', first_space + 2);
                        auto third_space = current.find(' ', second_space + 2);

                        if (first_space != std::string::npos && second_space != std::string::npos && third_space != std::string::npos) {
                            // Extract substring between the first and second space
                            std::string substring = current.substr(0, 4);

                            std::string speed = current.substr(first_space + 1, second_space - first_space - 1);
                            std::string apos = current.substr(second_space + 1, third_space - second_space - 1);

                            if (speed.size() > 0 && apos.size() > 0) {
                                if (!substring.compare(p2_str)) {
                                    try {
                                        p2_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& ex) {
                                        std::cout << ex.what() << " ; input_s2:" << speed << '\n';
                                    }
                                    try {
                                        p2_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& ex) {
                                        std::cout << ex.what() << " ; input_a2:" << apos << '\n';
                                    }
                                    // p2_speed = std::stoi(speed);
                                    // p2_apos = std::stoi(apos);
                                } else if (!substring.compare(p3_str)) {
                                    try {
                                        p3_speed = std::stoi(speed);
                                    } catch (std::invalid_argument const& ex) {
                                        std::cout << ex.what() << " ; input_s3:" << speed << '\n';
                                    }
                                    try {
                                        p3_apos = std::stoi(apos);
                                    } catch (std::invalid_argument const& ex) {
                                        std::cout << ex.what() << " ; input_a3:" << apos << '\n';
                                    }
                                    // p3_speed = std::stoi(speed);
                                    // p3_apos = std::stoi(apos);
                                }
                            }
                        }
                    }
                    current.clear();
                } else {
                    current += c;
                }
            }
        }
        } 
        std::cout << "p2_speed in rpm: " << p2_speed << "; p2_apos in deg: " << p2_apos << "; p3_speed: " << p3_speed << "; p3_apos: " << p3_apos << std::endl;
        // double p2_rad_s = 2.0 * M_PI * p2_speed / 33;
        // std::cout << "p2_speed in rad/s: " << p2_rad_s << std::endl;

        // if (i == 20) {
        //   cmd = "port 2; set 1;\r";
        //   std::cout << "Command: " << cmd << std::endl; 
        //   boost::asio::write(serial, boost::asio::buffer(cmd));
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }



    std::string end_message = "port 0; select; set 0; port 1; select; set 0;\r";
    boost::asio::write(serial, boost::asio::buffer(end_message)); 

    // Close serial port
    serial.close();
    
    return 0;
}
