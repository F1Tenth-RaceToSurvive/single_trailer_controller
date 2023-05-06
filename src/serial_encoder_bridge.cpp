#include "single_trailer_controller/serial_encoder_bridge.hpp"

SerialEncoderBridge::SerialEncoderBridge()
    :Node("serial_encoder_bridge_node"),
    config_(115200,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE),
    port_(ctx_, "/dev/ttyUSB0", config_)
{
    std::cout << "Serial port device name: " << port_.device_name() << std::endl;

    port_.open();
    std::cout << "Port is open: " << port_.is_open() << std::endl;


    // Create publisher for hitch angle
    hitch_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("hitch_angle", 10);

    // Initialize and bind the timer callback function to read serial data
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SerialEncoderBridge::readSerialData, this));
}



void SerialEncoderBridge::readSerialData()
{
    std::vector<uint8_t> receive_buf(10);
    int bubus = port_.receive(receive_buf);

    std::cout << "Printing received bytes: "<< receive_buf.size() << ", bubus:" << bubus << std::endl;
    for(auto& byte : receive_buf)
    {
        std::cout << static_cast<int>(byte);
    }
    
    std::cout << "\n._.\n";
    // Publish hitch angle
    std_msgs::msg::Float64 hitch_angle_msg;
    // hitch_angle_msg.data = std::stod(serial_data);
    hitch_angle_publisher_->publish(hitch_angle_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialEncoderBridge>());
    rclcpp::shutdown();
    return 0;
}