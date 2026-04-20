#!/usr/bin/env python3
# publisher_node.py — пример паблишера sensor_msgs/msg/NavSatStatus
#
# Запуск:
#   ros2 run ros2_msgs_enum_resolver publisher_node.py
#
# Публикует на топик /nav_status, циклически перебирая значения status и service.
# Subscriber (subscriber_node.py) декодирует числа обратно в имена констант.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatStatus

# Значения status: -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX
_STATUSES = [
    NavSatStatus.STATUS_NO_FIX,    # -1
    NavSatStatus.STATUS_FIX,       #  0
    NavSatStatus.STATUS_SBAS_FIX,  #  1
    NavSatStatus.STATUS_GBAS_FIX,  #  2
]

# Значения service: включая 3 (GPS|GLONASS bitmask) без именованной константы
_SERVICES = [
    0,                             # нет сервиса
    NavSatStatus.SERVICE_GPS,      # 1
    NavSatStatus.SERVICE_GLONASS,  # 2
    3,                             # GPS|GLONASS — именованной константы нет
]


class NavStatusPublisher(Node):
    def __init__(self):
        super().__init__('nav_status_publisher')
        self._pub = self.create_publisher(NavSatStatus, '/nav_status', 10)
        self._idx = 0
        self.create_timer(1.0, self._publish)
        self.get_logger().info('Публикация на /nav_status раз в секунду...')

    def _publish(self):
        msg = NavSatStatus()
        msg.status  = _STATUSES[self._idx % len(_STATUSES)]
        msg.service = _SERVICES[self._idx % len(_SERVICES)]
        self._idx += 1

        self.get_logger().info(
            f'Публикую  status={msg.status}  service={msg.service}'
        )
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavStatusPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
