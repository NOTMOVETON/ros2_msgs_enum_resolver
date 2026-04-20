// subscriber_node.cpp — пример подписчика с декодингом enum-полей
//
// Запуск (в отдельном терминале от паблишера):
//   ros2 run ros2_msgs_enum_resolver example_subscriber
//
// Подписывается на /nav_status и декодирует числовые значения
// status и service в символьные имена через EnumResolver.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ros2_msgs_enum_resolver/ros2_msgs_enum_resolver.hpp"

using NavSatStatus = sensor_msgs::msg::NavSatStatus;
using ER = ros2_msgs_enum_resolver::EnumResolver;

// Форматирует числовое значение как "(value) NAME" или просто "value"
// если именованной константы нет (например, bitmask-комбинации).
static std::string decode(int64_t value,
                          const std::string & type,
                          const std::string & field_type)
{
  auto names = ER::resolve(type, value, field_type);
  if (!names) {
    return std::to_string(value);  // нет имени — показываем число
  }
  std::string result = "(" + std::to_string(value) + ") ";
  for (size_t i = 0; i < names->size(); ++i) {
    if (i) result += '/';
    result += (*names)[i];
  }
  return result;
}

class NavStatusSubscriber : public rclcpp::Node
{
public:
  NavStatusSubscriber()
  : Node("nav_status_subscriber")
  {
    sub_ = create_subscription<NavSatStatus>(
      "/nav_status", 10,
      [this](const NavSatStatus::SharedPtr msg) { callback(msg); });

    RCLCPP_INFO(get_logger(), "Подписан на /nav_status — жду сообщений...");
  }

private:
  void callback(const NavSatStatus::SharedPtr msg)
  {
    const std::string TYPE = "sensor_msgs/msg/NavSatStatus";

    // int8 STATUS_* константы
    std::string status_str  = decode(msg->status,  TYPE, "int8");
    // uint16 SERVICE_* константы
    std::string service_str = decode(msg->service, TYPE, "uint16");

    RCLCPP_INFO(get_logger(),
      "status: %-22s  |  service: %s",
      status_str.c_str(), service_str.c_str());
  }

  rclcpp::Subscription<NavSatStatus>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}
