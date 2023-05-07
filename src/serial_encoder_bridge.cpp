#include "single_trailer_controller/serial_encoder_bridge.hpp"

#define PULSES_PER_REV 600
#define ENCODER_LEAST_COUNT 360/PULSES_PER_REV

SerialEncoderBridge::SerialEncoderBridge()
    :Node("serial_encoder_bridge_node"),
    config_(115200,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE),
    port_(ctx_, "/dev/ttyUSB0", config_)
{
    initial_encoder_pos_ = readSerialData();

    // Create publisher for hitch angle
    hitch_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("hitch_angle", 1);

    // Initialize and bind the timer callback function to read serial data
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SerialEncoderBridge::publishHitchAngle, this));
}

void SerialEncoderBridge::publishHitchAngle()
{
    int current_encoder_pos = readSerialData();

    // Compute the hitch angle
    double hitch_angle = (current_encoder_pos - initial_encoder_pos_) * ENCODER_LEAST_COUNT * M_PI / 180.0;
    // std::cout << "-------------------------------" << std::endl;
    // std::cout << "Intial encoder pos: " << initial_encoder_pos_ << std::endl;
    // std::cout << "Current encoder pos: " << current_encoder_pos << std::endl;
    // std::cout << "Hitch angle (rad): " << hitch_angle << std::endl;
    // std::cout << "Hitch angle (deg): " << hitch_angle * 180.0 / M_PI << std::endl;

    // Publish hitch angle
    std_msgs::msg::Float64 hitch_angle_msg;
    hitch_angle_msg.data = hitch_angle;
    hitch_angle_publisher_->publish(hitch_angle_msg);
}

int SerialEncoderBridge::readSerialData()
{
    port_.open();
    std::vector<uint8_t> receive_buf(1, 0);
    std::string received_data = "";

    // Loop until we recieve the new line character
    while(true)
    {
        port_.receive(receive_buf);
        if(receive_buf[0] == '\n')
        {
            break;
        }
    }

    // Store the the next characters until we receive a return character
    while(true)
    {
        port_.receive(receive_buf);
        if(receive_buf[0] == '\r')
        {
            break;
        }
        received_data += receive_buf[0];
    }
    // std::cout << "Received data: " << received_data << std::endl;
    port_.close();

    return std::stoi(received_data);
}

void SerialEncoderBridge::serialReadCallback(const std::vector<uint8_t> &buffer, const size_t &bytes_transferred)
{

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialEncoderBridge>());
    rclcpp::shutdown();
    return 0;
}
