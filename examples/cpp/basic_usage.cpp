// basic_usage.cpp — C++ пример использования ros2_msgs_enum_resolver
//
// Запуск (после colcon build && source install/setup.bash):
//   ros2 run ros2_msgs_enum_resolver example_basic_usage
//
// Программа не является ROS2-нодой: rclcpp не используется.
// EnumResolver читает .msg-файлы напрямую через ament_index.

#include <iostream>
#include <string>
#include <vector>

#include "ros2_msgs_enum_resolver/ros2_msgs_enum_resolver.hpp"

using ER = ros2_msgs_enum_resolver::EnumResolver;

// Вспомогательная функция: печатает результат resolve()
static void print_resolve(const std::string & label,
                          const std::optional<std::vector<std::string>> & names)
{
  std::cout << "  " << label << ": ";
  if (!names) {
    std::cout << "(нет совпадения)\n";
    return;
  }
  for (size_t i = 0; i < names->size(); ++i) {
    if (i) std::cout << " / ";
    std::cout << (*names)[i];
  }
  std::cout << '\n';
}

int main()
{
  // sensor_msgs/msg/NavSatStatus — стандартное сообщение с двумя наборами констант:
  //   int8  STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
  //   uint16 SERVICE_GPS=1, SERVICE_GLONASS=2, SERVICE_COMPASS=4, SERVICE_GALILEO=8
  const std::string TYPE = "sensor_msgs/msg/NavSatStatus";

  // --------------------------------------------------------------------------
  // 1. Forward resolve: значение -> имя константы
  //    field_type выбирает только нужный набор констант (int8 или uint16).
  // --------------------------------------------------------------------------
  std::cout << "=== 1. Forward resolve (int8 status) ===\n";
  print_resolve("status -1", ER::resolve(TYPE, -1, "int8"));  // STATUS_NO_FIX
  print_resolve("status  0", ER::resolve(TYPE,  0, "int8"));  // STATUS_FIX
  print_resolve("status  1", ER::resolve(TYPE,  1, "int8"));  // STATUS_SBAS_FIX
  print_resolve("status  2", ER::resolve(TYPE,  2, "int8"));  // STATUS_GBAS_FIX
  print_resolve("status 99", ER::resolve(TYPE, 99, "int8"));  // нет совпадения

  std::cout << "\n=== 1. Forward resolve (uint16 service) ===\n";
  print_resolve("service 1", ER::resolve(TYPE, 1, "uint16"));  // SERVICE_GPS
  print_resolve("service 2", ER::resolve(TYPE, 2, "uint16"));  // SERVICE_GLONASS
  print_resolve("service 4", ER::resolve(TYPE, 4, "uint16"));  // SERVICE_COMPASS
  // 3 = GPS|GLONASS — битовая маска, именованной константы нет
  print_resolve("service 3", ER::resolve(TYPE, 3, "uint16"));

  // Без field_type — ищет среди всех констант файла
  std::cout << "\n  (без field_type) status 0: ";
  auto v = ER::resolve(TYPE, 0);
  if (v) std::cout << (*v)[0] << '\n'; else std::cout << "(нет)\n";

  // --------------------------------------------------------------------------
  // 2. Aliases: одно значение — несколько имён
  //    Пример: если бы STATUS_FIX = STATUS_OK = 0, resolve вернул бы оба имени.
  //    В NavSatStatus алиасов нет, но API поддерживает такой паттерн.
  //    result->size() > 1 означает алиас.
  // --------------------------------------------------------------------------
  std::cout << "\n=== 2. Aliases (примечание) ===\n";
  std::cout << "  resolve вернёт вектор с >1 элементом, если несколько констант\n"
               "  делят одно значение (например: INIT=1, STARTING=1).\n";

  // --------------------------------------------------------------------------
  // 3. Reverse resolve: имя -> значение
  // --------------------------------------------------------------------------
  std::cout << "\n=== 3. Reverse resolve ===\n";
  const auto print_rev = [&](const std::string & name) {
    auto val = ER::reverseResolve(TYPE, name);
    std::cout << "  " << name << " = "
              << (val ? std::to_string(*val) : "(не найдено)") << '\n';
  };
  print_rev("STATUS_FIX");
  print_rev("SERVICE_GPS");
  print_rev("UNKNOWN");

  // --------------------------------------------------------------------------
  // 4. Bulk access: все константы сразу
  // --------------------------------------------------------------------------
  std::cout << "\n=== 4. getConstants (все константы) ===\n";
  auto all = ER::getConstants(TYPE);
  if (all) {
    for (const auto & [value, names] : *all) {
      std::cout << "  " << value << " -> ";
      for (size_t i = 0; i < names.size(); ++i) {
        if (i) std::cout << " / ";
        std::cout << names[i];
      }
      std::cout << '\n';
    }
  }

  // --------------------------------------------------------------------------
  // 5. Фильтрация по типу: только int8-константы
  // --------------------------------------------------------------------------
  std::cout << "\n=== 5. getConstantsForPrimitive(\"int8\") ===\n";
  auto int8c = ER::getConstantsForPrimitive(TYPE, "int8");
  if (int8c) {
    for (const auto & [value, names] : *int8c) {
      std::cout << "  " << value << " -> " << names[0] << '\n';
    }
  } else {
    std::cout << "  (нет int8-констант)\n";
  }

  // --------------------------------------------------------------------------
  // 6. Introspection
  // --------------------------------------------------------------------------
  std::cout << "\n=== 6. Introspection ===\n";
  std::cout << "  isAvailable:   " << (ER::isAvailable(TYPE)   ? "true" : "false") << '\n';
  // NavSatStatus имеет обычные поля — НЕ enum-only
  std::cout << "  isEnumOnly:    " << (ER::isEnumOnlyType(TYPE) ? "true" : "false") << '\n';
  // NavSatStatus не имеет алиасов
  std::cout << "  isUnambiguous: " << (ER::isUnambiguous(TYPE)  ? "true" : "false") << '\n';

  // --------------------------------------------------------------------------
  // 7. srv / action секции
  //    При наличии пакета с сервисом/экшеном:
  //      ER::resolve("my_pkg/srv/SetMode/Request",   10)
  //      ER::resolve("my_pkg/srv/SetMode/Response",   0)
  //      ER::resolve("my_pkg/action/Navigate/Goal",   1)
  //      ER::resolve("my_pkg/action/Navigate/Result", 0)
  //    По умолчанию srv -> Request, action -> Goal.
  // --------------------------------------------------------------------------
  std::cout << "\n=== 7. srv/action (справка) ===\n";
  std::cout << "  resolve(\"pkg/srv/Name/Request\", val)\n"
               "  resolve(\"pkg/srv/Name/Response\", val)\n"
               "  resolve(\"pkg/action/Name/Goal\", val)\n"
               "  resolve(\"pkg/action/Name/Result\", val)\n"
               "  resolve(\"pkg/action/Name/Feedback\", val)\n";

  // --------------------------------------------------------------------------
  // 8. Cache management
  // --------------------------------------------------------------------------
  std::cout << "\n=== 8. Cache management ===\n";
  bool ok = ER::preload(TYPE);
  std::cout << "  preload: " << (ok ? "ok" : "failed") << '\n';

  ER::clearCache();
  std::cout << "  clearCache() — следующий вызов перечитает файл с диска\n";

  print_resolve("status 0 после clearCache", ER::resolve(TYPE, 0, "int8"));

  return 0;
}
