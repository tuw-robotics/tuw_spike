#include <cstdio>
#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include <string>
#include <math.h>

// run with "docker -c lego0 compose up --build hardware"
// hardware:
//      extend: runtime
//      command: ros2 run serial_if serial_if

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  // Initialize Boost Asio
  boost::asio::io_service io;
  
  // Define serial port settings
  std::string port_name = "/dev/ttyS0";
  unsigned int baud_rate = 115200;
  
  // Open serial port
  boost::asio::serial_port serial(io, port_name);
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));



  /*int pos = 0;
  int mul = 1;
  int degrees = 540;
  int newpos = ((degrees * mul) + pos) / 360.0;
  pos /= 360.0;*/
  // int dur = abs((newpos - pos));

  // cmd = "set ramp " + std::to_string(pos) + " " + std::to_string(newpos) + " " + std::to_string(dur) + " 0\r";
  // cmd = "set 1; port 3; set -1;";
  // std::cout << cmd << std::endl; 
  // boost::asio::write(serial, boost::asio::buffer(cmd));

  std::string cmd = "echo 0;\r";
  std::cout << cmd << std::endl; 
  boost::asio::write(serial, boost::asio::buffer(cmd));

  cmd = "plimit 1; port 2; combi 0 1 0 2 0 3 0; select 0 ; selrate 10; pid_diff 2 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
  std::cout << cmd << std::endl; 
  boost::asio::write(serial, boost::asio::buffer(cmd));

  cmd = "port 3; combi 0 1 0 2 0 3 0; select 0; selrate 10; pid_diff 3 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01;\r";
  std::cout << cmd << std::endl; 
  boost::asio::write(serial, boost::asio::buffer(cmd));


  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  cmd = "port 2; set -1; port 3; set 1;\r";
  std::cout << cmd << std::endl; 
  boost::asio::write(serial, boost::asio::buffer(cmd));

  std::string p2_str = "P2C0";
  std::string p3_str = "P3C0";
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
        // std::cout << "Read " << bytes_read << " bytes\n";
        // process buffer
        for (char c : buffer) {
          if (c == '\n') {
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



  std::string end_message = "port 2; select; set 0; port 3; select; set 0;\r";
  boost::asio::write(serial, boost::asio::buffer(end_message)); 

  // Close serial port
  serial.close();
  
  return 0;
}
