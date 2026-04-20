# ros2_msgs_enum_resolver

C++ and Python utility for ROS 2 that resolves **numeric message constants**
to their **symbolic names** at runtime — and back.

Includes **`enum_echo`** — a drop-in replacement for `ros2 topic echo` that
automatically decodes numeric enum fields to their symbolic names.

Reads `.msg`, `.srv`, `.action` interface files directly from the ament_index.
No code generation. No build-time registration. No message-package dependency.

---

## Install

### apt (Ubuntu Noble + ROS 2 Jazzy)

```bash
curl -fsSL https://NOTMOVETON.github.io/ros2_msgs_enum_resolver/KEY.gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/notmoveton.gpg

echo "deb [signed-by=/etc/apt/keyrings/notmoveton.gpg] https://NOTMOVETON.github.io/ros2_msgs_enum_resolver noble main" \
  | sudo tee /etc/apt/sources.list.d/notmoveton.list

sudo apt update
sudo apt install ros-jazzy-ros2-msgs-enum-resolver
```

### Build from source

```bash
cd ~/ros2_ws/src
git clone https://github.com/NOTMOVETON/ros2_msgs_enum_resolver.git
cd ~/ros2_ws
colcon build --packages-select ros2_msgs_enum_resolver
source install/setup.bash
```

---

## enum_echo

```bash
# Auto-detect topic type and subscribe
ros2 run ros2_msgs_enum_resolver enum_echo /my_topic

# Specify type explicitly (useful before publisher is running)
ros2 run ros2_msgs_enum_resolver enum_echo /my_topic sensor_msgs/msg/NavSatStatus

# Stop after 5 messages
ros2 run ros2_msgs_enum_resolver enum_echo /my_topic --count 5

# Plain mode (no enum decoding)
ros2 run ros2_msgs_enum_resolver enum_echo /my_topic --no-enum

# Increase auto-detect timeout (default: 10s)
ros2 run ros2_msgs_enum_resolver enum_echo /my_topic --wait-timeout 30
```

### Tab completion

Topic names are tab-completable. After `source install/setup.bash` (workspace build)
or `source /opt/ros/jazzy/setup.bash` (apt install) completion activates automatically:

```bash
ros2 run ros2_msgs_enum_resolver enum_echo <TAB>   # lists live topics
enum_echo <TAB>                                     # same, direct invocation
```

### Output format

```
status: (0) STATUS_FIX
service: 3
mode: (1) INIT/STARTING
connection_state: (2) CONNECTED
nested_msg:
  error_code: (3) ERR_TIMEOUT
---
```

- `(value) NAME` — value found in constants
- `(value) NAME_A/NAME_B` — aliased value (multiple constants share the same number)
- `value` — no matching constant found (e.g. bitmask combinations like `service: 3`)

### Supported enum patterns

| Pattern | Example | How detected |
|---|---|---|
| Dedicated enum message | `pkg/msg/ConnectionEventEnum connection_state` | Field type contains only constants |
| Inline constants, same type | `int8 status` + `int8 STATUS_FIX = 0` | Primitive type of field matches constant type |
| Multiple types in one message | `sensor_msgs/msg/NavSatStatus`: `int8 status` + `uint16 service` | Each field matched to constants of its own type |

### Notes

- `enum_echo` works with **topic subscriptions only** (`.msg` types).
  `.srv` and `.action` types are used in service calls and action clients,
  not in topic publications, so they do not appear in topic echo.
- `EnumResolver` supports `.srv` and `.action` sections programmatically
  (see C++/Python API below).

---

## C++ API

```cpp
#include "ros2_msgs_enum_resolver/ros2_msgs_enum_resolver.hpp"
using ER = ros2_msgs_enum_resolver::EnumResolver;

// --- Forward: value -> names ---

// Enum-only message type (field type IS the enum message)
auto names = ER::resolve("my_pkg/msg/ConnectionEventEnum", 1);
// -> {"CONNECTED"}

// Aliased value (two constants share the same number)
auto aliases = ER::resolve("my_pkg/msg/ModeEnum", 1);
// -> {"AUTO", "DEFAULT"}

// Inline constants: pass the primitive type of the field
// (NavSatStatus pattern: int8 STATUS_* + uint16 SERVICE_* in same file)
auto n1 = ER::resolve("sensor_msgs/msg/NavSatStatus", 0, "int8");
// -> {"STATUS_FIX"}

auto n2 = ER::resolve("sensor_msgs/msg/NavSatStatus", 1, "uint16");
// -> {"SERVICE_GPS"}

// Value not found -> nullopt (display as plain number)
auto none = ER::resolve("sensor_msgs/msg/NavSatStatus", 3, "uint16");
// -> std::nullopt  (3 = GPS|GLONASS bitmask, not a named constant)

// --- Reverse: name -> value ---
auto val = ER::reverseResolve("my_pkg/msg/ConnectionEventEnum", "CONNECTED");
// -> 1

// --- Bulk access ---
auto all   = ER::getConstants("my_pkg/msg/ConnectionEventEnum");
auto typed = ER::getConstantsForPrimitive("sensor_msgs/msg/NavSatStatus", "int8");

// --- srv / action sections ---
auto req  = ER::resolve("my_pkg/srv/SetMode/Request",      10);
auto res  = ER::resolve("my_pkg/srv/SetMode/Response",      0);
auto goal = ER::resolve("my_pkg/action/Navigate/Goal",      1);
auto fb   = ER::resolve("my_pkg/action/Navigate/Feedback",  0);

// --- Introspection ---
bool pure  = ER::isEnumOnlyType("my_pkg/msg/ConnectionEventEnum"); // true
bool mixed = ER::isEnumOnlyType("sensor_msgs/msg/NavSatStatus");   // false
bool clean = ER::isUnambiguous("my_pkg/msg/ConnectionEventEnum");

// --- Cache management ---
ER::preload("my_pkg/msg/ConnectionEventEnum");
ER::clearCache();
bool ok = ER::isAvailable("my_pkg/msg/ConnectionEventEnum");
```

**package.xml**
```xml
<depend>ros2_msgs_enum_resolver</depend>
```

**CMakeLists.txt**
```cmake
find_package(ros2_msgs_enum_resolver REQUIRED)
target_link_libraries(my_node ros2_msgs_enum_resolver::ros2_msgs_enum_resolver)
```

---

## Python API

```python
from ros2_msgs_enum_resolver import EnumResolver, FieldTypeParser, format_message

# --- EnumResolver ---

# Enum-only message type
names = EnumResolver.resolve('my_pkg/msg/ConnectionEventEnum', 1)
# -> ['CONNECTED']

# Inline constants with primitive type
names2 = EnumResolver.resolve('sensor_msgs/msg/NavSatStatus', 0, 'int8')
# -> ['STATUS_FIX']

# Aliased value
aliases = EnumResolver.resolve('my_pkg/msg/ModeEnum', 1)
# -> ['AUTO', 'DEFAULT']

# Reverse
val = EnumResolver.reverse_resolve('my_pkg/msg/ConnectionEventEnum', 'CONNECTED')
# -> 1

# Bulk
all_c   = EnumResolver.get_constants('my_pkg/msg/ConnectionEventEnum')
typed_c = EnumResolver.get_constants_for_primitive('sensor_msgs/msg/NavSatStatus', 'int8')

# Introspection
EnumResolver.is_enum_only_type('my_pkg/msg/ConnectionEventEnum')  # True
EnumResolver.is_enum_only_type('sensor_msgs/msg/NavSatStatus')    # False

# srv / action sections
EnumResolver.resolve('my_pkg/srv/SetMode/Request',      10)
EnumResolver.resolve('my_pkg/action/Navigate/Feedback',  0)

# --- FieldTypeParser ---
# Returns per-field info: what interface_type and field_type to pass to resolve()

fields = FieldTypeParser.get_fields('sensor_msgs/msg/NavSatStatus')
# {
#   'status':  FieldInfo(enum_type='sensor_msgs/msg/NavSatStatus',
#                        primitive='int8',   is_array=False),
#   'service': FieldInfo(enum_type='sensor_msgs/msg/NavSatStatus',
#                        primitive='uint16', is_array=False),
# }

# --- format_message ---
# Formats a message dict (from rosidl_runtime_py.message_to_ordereddict)

from collections import OrderedDict
msg_dict = OrderedDict([('status', 0), ('service', 3)])
print(format_message(msg_dict, 'sensor_msgs/msg/NavSatStatus'))
# status: (0) STATUS_FIX
# service: 3
```

---

## Supported type strings

| Format | Default section |
|---|---|
| `pkg/msg/TypeName` | — |
| `pkg/srv/TypeName[/Request]` | Request |
| `pkg/srv/TypeName/Response` | — |
| `pkg/action/TypeName[/Goal]` | Goal |
| `pkg/action/TypeName/Result` | — |
| `pkg/action/TypeName/Feedback` | — |

---

## Examples

Runnable examples are installed alongside the package:

```bash
# C++ — standalone (no ROS spin, prints output and exits)
ros2 run ros2_msgs_enum_resolver example_basic_usage

# C++ — publisher + subscriber nodes
ros2 run ros2_msgs_enum_resolver example_publisher   # terminal 1
ros2 run ros2_msgs_enum_resolver example_subscriber  # terminal 2

# Python — standalone
ros2 run ros2_msgs_enum_resolver basic_usage.py

# Python — publisher + subscriber nodes
ros2 run ros2_msgs_enum_resolver publisher_node.py   # terminal 1
ros2 run ros2_msgs_enum_resolver subscriber_node.py  # terminal 2
```

Source code: [examples/cpp/](examples/cpp/) and [examples/python/](examples/python/).

---

## Tests

```bash
colcon test --packages-select ros2_msgs_enum_resolver
colcon test-result --verbose
```

---

## Repository structure

```
ros2_msgs_enum_resolver/
├── include/ros2_msgs_enum_resolver/
│   └── ros2_msgs_enum_resolver.hpp     C++ header
├── src/
│   └── ros2_msgs_enum_resolver.cpp     C++ implementation
├── ros2_msgs_enum_resolver/
│   ├── __init__.py
│   ├── ros2_msgs_enum_resolver.py      Python EnumResolver
│   ├── field_type_parser.py            Field -> FieldInfo mapping
│   └── message_formatter.py           dict -> formatted string
├── scripts/
│   └── enum_echo                       CLI tool
├── examples/
│   ├── cpp/                            C++ usage examples
│   └── python/                         Python usage examples
├── env-hooks/
│   └── enum_echo_completion.bash       Tab completion hook
└── test/
    ├── test_ros2_msgs_enum_resolver.cpp
    └── test_ros2_msgs_enum_resolver.py
```
