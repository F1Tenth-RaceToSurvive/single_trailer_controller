#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include "serial_driver/serial_port.hpp"

class SerialEncoderBridge : public rclcpp::Node
{
public:
    SerialEncoderBridge();

private:
    IoContext ctx_;
    drivers::serial_driver::SerialPortConfig config_;
    drivers::serial_driver::SerialPort port_;
    int initial_encoder_pos_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher of hitch angle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hitch_angle_publisher_;

    void publishHitchAngle();
    int readSerialData();   // Function that reads serial data and publishes hitch angle
    void serialReadCallback(const std::vector<uint8_t> &buffer, const size_t &bytes_transferred);
};
