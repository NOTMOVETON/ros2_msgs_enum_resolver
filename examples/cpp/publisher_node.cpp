// publisher_node.cpp — пример паблишера sensor_msgs/msg/NavSatStatus
//
// Запуск:
//   ros2 run ros2_msgs_enum_resolver example_publisher
//
// Публикует на топик /nav_status, циклически перебирая значения status и service.
// Подписчик (example_subscriber) декодирует числа обратно в имена констант.

#include <array>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

using NavSatStatus = sensor_msgs::msg::NavSatStatus;

// Значения status: -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX
static constexpr std::array<int8_t, 4> kStatuses = {
  NavSatStatus::STATUS_NO_FIX,    // -1
  NavSatStatus::STATUS_FIX,       //  0
  NavSatStatus::STATUS_SBAS_FIX,  //  1
  NavSatStatus::STATUS_GBAS_FIX,  //  2
};

// Значения service: включая 3 (GPS|GLONASS bitmask) без именованной константы
static constexpr std::array<uint16_t, 4> kServices = {
  0,                              // нет сервиса
  NavSatStatus::SERVICE_GPS,      // 1
  NavSatStatus::SERVICE_GLONASS,  // 2
  3,                              // GPS|GLONASS — именованной константы нет
};

class NavStatusPublisher : public rclcpp::Node
{
public:
  NavStatusPublisher()
  : Node("nav_status_publisher"), idx_(0)
  {
    pub_ = create_publisher<NavSatStatus>("/nav_status", 10);
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this] { publish(); });

    RCLCPP_INFO(get_logger(), "Публикация на /nav_status раз в секунду...");
  }

private:
  void publish()
  {
    NavSatStatus msg;
    msg.status  = kStatuses[idx_ % kStatuses.size()];
    msg.service = kServices[idx_ % kServices.size()];
    ++idx_;

    RCLCPP_INFO(get_logger(),
      "Публикую  status=%d  service=%u",
      static_cast<int>(msg.status), static_cast<unsigned>(msg.service));

    pub_->publish(msg);
  }

  rclcpp::Publisher<NavSatStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t idx_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
