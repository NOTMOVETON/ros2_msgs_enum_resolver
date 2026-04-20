#!/usr/bin/env python3
# subscriber_node.py — пример подписчика с декодингом enum-полей
#
# Запуск (в отдельном терминале от паблишера):
#   ros2 run ros2_msgs_enum_resolver subscriber_node.py
#
# Подписывается на /nav_status и выводит сообщение с декодированными
# числовыми значениями через format_message.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatStatus
from rosidl_runtime_py import message_to_ordereddict

from ros2_msgs_enum_resolver import format_message

TYPE = 'sensor_msgs/msg/NavSatStatus'


class NavStatusSubscriber(Node):
    def __init__(self):
        super().__init__('nav_status_subscriber')
        self.create_subscription(
            NavSatStatus, '/nav_status', self._callback, 10
        )
        self.get_logger().info('Подписан на /nav_status — жду сообщений...')

    def _callback(self, msg: NavSatStatus):
        # Преобразуем ROS2-сообщение в словарь
        msg_dict = message_to_ordereddict(msg)

        # format_message декодирует числа в символьные имена.
        # Вывод:
        #   status: (0) STATUS_FIX
        #   service: 3                  <- bitmask, именованной константы нет
        formatted = format_message(msg_dict, TYPE)
        self.get_logger().info('\n' + formatted)


def main(args=None):
    rclpy.init(args=args)
    node = NavStatusSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
