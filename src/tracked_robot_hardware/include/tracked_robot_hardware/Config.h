#include <string>

struct Config
{
  std::string left_sprocket_name = "sprocket_left_joint";
  std::string right_sprocket_name = "sprocket_right_joint";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
};