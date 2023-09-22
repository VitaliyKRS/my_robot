#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sstream>

LibSerial::BaudRate convert_baud_rate(int32_t baud_rate)
{
    // Just handle some common baud rates
    switch (baud_rate) {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    case 1000000:
        return LibSerial::BaudRate::BAUD_1000000;
    default:
        // std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" <<
        // std::endl;
        return LibSerial::BaudRate::BAUD_1000000;  // LibSerial::BaudRate::BAUD_57600;
    }
}

class SerialPortWrapper {
public:
    SerialPortWrapper() = default;

    void connect(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        mTimeout = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect() { serial_conn_.Close(); }

    bool connected() const { return serial_conn_.IsOpen(); }

    std::string send_msg(const std::string& msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers();  // Just in case
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try {
            // Responses end with \r\n so we will read up to (and including) the \n.
            serial_conn_.ReadLine(response, '\n', mTimeout);
        }
        catch (const LibSerial::ReadTimeout&) {
            // RCLCPP_INFO_STREAM(logger_,"The ReadByte() call has timed out");
        }

        if (print_output) {
            // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
            // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
        }

        return response;
    }

    void send_empty_msg() { std::string response = send_msg("\r"); }

    void read_encoder_values(double& val_1, double& val_2)
    {
        std::string response = send_msg("{\"T\":73}\r");

        std::regex enc_regex("[0-9]{1,3}");
        std::smatch enc_match;
        bool first_match = true;

        std::cout << "Read Encoders from Serial: " << response.c_str() << std::endl;

        // {"L":0.376697924,"R":0.376697924}
        while (regex_search(response, enc_match, enc_regex)) {
            // std::cout << enc_match.str() << '\n';
            if (first_match) {
                val_1 = std::atof(enc_match.str().c_str());
                first_match = false;
            }
            else {
                val_2 = std::atof(enc_match.str().c_str());
            }
            response = enc_match.suffix();
        }
    }

    void set_motor_values(double val_1, double val_2)
    {
        std::stringstream ss;

        // {"T":1,"L":0.5,"R":0.5}
        ss << "{\"T\":1,\"L\":" << val_1 << ",\"R\":" << val_2 << "}\r";
        // std::cout << ss.str() << std::endl;
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int mTimeout;
};