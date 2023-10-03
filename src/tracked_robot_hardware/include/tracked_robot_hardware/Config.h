#include <string>

struct Config {
    std::string left_sprocket_name = "sprocket_left_joint";
    std::string right_sprocket_name = "sprocket_right_joint";
    float loop_rate = 100.;
    std::string device = "/dev/ugv";
    int32_t baud_rate = 1000000;
    int timeout = 100;
    int enc_counts_per_rev = 550; //462;
    int pid_p = 0;
    int pid_d = 0;
    int pid_i = 0;
    int pid_o = 0;
};