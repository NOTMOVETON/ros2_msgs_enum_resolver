#!/usr/bin/env python3
# basic_usage.py — Python-пример использования ros2_msgs_enum_resolver
#
# Запуск (после colcon build && source install/setup.bash):
#   ros2 run ros2_msgs_enum_resolver basic_usage.py
#   # или напрямую:
#   python3 basic_usage.py

from collections import OrderedDict

from ros2_msgs_enum_resolver import EnumResolver, FieldTypeParser, format_message

# sensor_msgs/msg/NavSatStatus — стандартное сообщение с двумя наборами констант:
#   int8   STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
#   uint16 SERVICE_GPS=1,    SERVICE_GLONASS=2, SERVICE_COMPASS=4, SERVICE_GALILEO=8
TYPE = 'sensor_msgs/msg/NavSatStatus'


def main():
    # --------------------------------------------------------------------------
    # 1. Forward resolve: значение -> имя константы
    #    field_type выбирает только нужный набор констант.
    # --------------------------------------------------------------------------
    print('=== 1. Forward resolve (int8 status) ===')
    for val in (-1, 0, 1, 2, 99):
        names = EnumResolver.resolve(TYPE, val, 'int8')
        label = ' / '.join(names) if names else '(нет совпадения)'
        print(f'  status {val:4d}  ->  {label}')

    print()
    print('=== 1. Forward resolve (uint16 service) ===')
    for val in (0, 1, 2, 4, 3):
        names = EnumResolver.resolve(TYPE, val, 'uint16')
        label = ' / '.join(names) if names else '(нет совпадения, например bitmask)'
        print(f'  service {val}  ->  {label}')

    # Без field_type — ищет среди всех констант файла
    print()
    names = EnumResolver.resolve(TYPE, 0)
    print(f'  (без field_type) status 0  ->  {names}')

    # --------------------------------------------------------------------------
    # 2. Aliases: одно значение — несколько имён
    #    Если несколько констант делят одно значение (INIT=1, STARTING=1),
    #    resolve() вернёт список с >1 элементом.
    # --------------------------------------------------------------------------
    print()
    print('=== 2. Aliases (примечание) ===')
    print('  resolve() возвращает список — если алиасы есть, len > 1.')
    print('  Пример: resolve("pkg/msg/State", 1) -> ["INIT", "STARTING"]')

    # --------------------------------------------------------------------------
    # 3. Reverse resolve: имя -> значение
    # --------------------------------------------------------------------------
    print()
    print('=== 3. Reverse resolve ===')
    for name in ('STATUS_FIX', 'SERVICE_GPS', 'UNKNOWN'):
        val = EnumResolver.reverse_resolve(TYPE, name)
        print(f'  {name:<20s} = {val}')

    # --------------------------------------------------------------------------
    # 4. Bulk access: все константы сразу
    # --------------------------------------------------------------------------
    print()
    print('=== 4. get_constants (все константы) ===')
    all_consts = EnumResolver.get_constants(TYPE)
    if all_consts:
        for value, names in all_consts.items():
            print(f'  {value:4d}  ->  {" / ".join(names)}')

    # --------------------------------------------------------------------------
    # 5. Фильтрация по типу: только int8-константы
    # --------------------------------------------------------------------------
    print()
    print('=== 5. get_constants_for_primitive("int8") ===')
    int8_consts = EnumResolver.get_constants_for_primitive(TYPE, 'int8')
    if int8_consts:
        for value, names in int8_consts.items():
            print(f'  {value:4d}  ->  {" / ".join(names)}')
    else:
        print('  (нет int8-констант)')

    # --------------------------------------------------------------------------
    # 6. Introspection
    # --------------------------------------------------------------------------
    print()
    print('=== 6. Introspection ===')
    print(f'  is_available:   {EnumResolver.is_available(TYPE)}')
    # NavSatStatus имеет обычные поля — НЕ enum-only
    print(f'  is_enum_only:   {EnumResolver.is_enum_only_type(TYPE)}')
    # NavSatStatus не имеет алиасов — True
    print(f'  is_unambiguous: {EnumResolver.is_unambiguous(TYPE)}')

    # --------------------------------------------------------------------------
    # 7. FieldTypeParser: поля и их enum-мэппинг
    #    Используется внутри format_message, но доступен и напрямую.
    # --------------------------------------------------------------------------
    print()
    print('=== 7. FieldTypeParser.get_fields ===')
    fields = FieldTypeParser.get_fields(TYPE)
    for field_name, info in fields.items():
        print(f'  {field_name:<10s}  enum_type={info.enum_type!r}  '
              f'primitive={info.primitive!r}  is_array={info.is_array}')

    # --------------------------------------------------------------------------
    # 8. format_message: форматирование словаря с декодингом
    #    Принимает OrderedDict (как из rosidl_runtime_py.message_to_ordereddict).
    # --------------------------------------------------------------------------
    print()
    print('=== 8. format_message ===')
    msg_dict = OrderedDict([('status', 0), ('service', 3)])
    print(format_message(msg_dict, TYPE))

    # Режим без декодинга (plain):
    print()
    print('  (plain режим, enabled=False):')
    print(format_message(msg_dict, TYPE, enabled=False))

    # --------------------------------------------------------------------------
    # 9. srv / action секции
    # --------------------------------------------------------------------------
    print()
    print('=== 9. srv / action (справка) ===')
    print("  EnumResolver.resolve('pkg/srv/Name/Request',    value)")
    print("  EnumResolver.resolve('pkg/srv/Name/Response',   value)")
    print("  EnumResolver.resolve('pkg/action/Name/Goal',    value)")
    print("  EnumResolver.resolve('pkg/action/Name/Result',  value)")
    print("  EnumResolver.resolve('pkg/action/Name/Feedback',value)")

    # --------------------------------------------------------------------------
    # 10. Cache management
    # --------------------------------------------------------------------------
    print()
    print('=== 10. Cache management ===')
    ok = EnumResolver.preload(TYPE)
    print(f'  preload: {"ok" if ok else "failed"}')
    EnumResolver.clear_cache()
    print('  clear_cache() — следующий вызов перечитает файл с диска')
    names = EnumResolver.resolve(TYPE, 0, 'int8')
    print(f'  status 0 после clear_cache -> {names}')


if __name__ == '__main__':
    main()
