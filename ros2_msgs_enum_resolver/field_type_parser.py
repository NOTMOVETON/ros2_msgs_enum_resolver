# Copyright 2024 Your Name
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""field_type_parser - parse field definitions from ROS2 interface files.

Given a message type string, returns a description of each field:

- If the field type is a dedicated enum-only message, enum_type is set to
  the full interface type string and primitive is None.
- If the field is a primitive and the parent message has inline constants
  of the same type, enum_type is the parent type and primitive is the
  primitive type name (e.g. 'int8').
- Otherwise enum_type and primitive are both None (no enum decoding).

Example output for sensor_msgs/msg/NavSatStatus::

    {
        'status':  FieldInfo(enum_type='sensor_msgs/msg/NavSatStatus',
                             primitive='int8', is_array=False),
        'service': FieldInfo(enum_type='sensor_msgs/msg/NavSatStatus',
                             primitive='uint16', is_array=False),
    }

"""

from __future__ import annotations

import re
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from ros2_msgs_enum_resolver.ros2_msgs_enum_resolver import EnumResolver


_PRIMITIVES = {
    'bool', 'byte', 'char',
    'float32', 'float64',
    'int8', 'int16', 'int32', 'int64',
    'uint8', 'uint16', 'uint32', 'uint64',
    'string', 'wstring',
}

_NUMERIC_PRIMITIVES = {
    'bool', 'byte', 'char',
    'float32', 'float64',
    'int8', 'int16', 'int32', 'int64',
    'uint8', 'uint16', 'uint32', 'uint64',
}

# Field line: <type>[array] <lowercase_name>  (no '=' sign)
# Using non-raw strings to avoid double-backslash confusion.
_FIELD_RE = re.compile(
    r'^\s*'
    r'((?:[a-zA-Z][a-zA-Z0-9_]*/)*[a-zA-Z][a-zA-Z0-9_]*)'
    r'(\[\d*\]|\[<=\d+\])?'
    r'\s+'
    r'([a-z][a-zA-Z0-9_]*)'
    r'(\s+\S+)?\s*$'
)


@dataclass
class FieldInfo:
    """Enum resolution info for one field."""

    # Full interface type to pass to EnumResolver.resolve() as interface_type.
    # None when no enum decoding is possible for this field.
    enum_type: Optional[str]

    # When not None, pass this as field_type to EnumResolver.resolve().
    # None for dedicated enum-only message types (pass field_type='').
    primitive: Optional[str]

    # True when the field is declared as an array.
    is_array: bool


class FieldTypeParser:
    """Parse field definitions and map them to EnumResolver call parameters.

    Results are cached per interface_type so each .msg file is read once.
    """

    _lock: threading.Lock = threading.Lock()
    _cache: Dict[str, Dict[str, FieldInfo]] = {}

    @classmethod
    def get_fields(cls, interface_type: str) -> Dict[str, FieldInfo]:
        """Return field name -> FieldInfo mapping for the given type."""
        with cls._lock:
            if interface_type in cls._cache:
                return cls._cache[interface_type]

        result = cls._parse(interface_type)

        with cls._lock:
            cls._cache[interface_type] = result
        return result

    @classmethod
    def clear_cache(cls) -> None:
        """Flush the field cache."""
        with cls._lock:
            cls._cache.clear()

    @classmethod
    def _parse(cls, interface_type: str) -> Dict[str, FieldInfo]:
        """Read the interface file and build the field map."""
        content = cls._read_section(interface_type)
        if content is None:
            return {}

        # Which primitive types have inline constants in this message?
        inline_primitive_types = cls._find_inline_primitive_types(interface_type)

        result: Dict[str, FieldInfo] = {}

        for line in content.splitlines():
            # Strip inline comment
            comment_pos = line.find('#')
            if comment_pos != -1:
                line = line[:comment_pos]
            line = line.strip()
            if not line:
                continue
            # Constant lines always contain '='
            if '=' in line:
                continue

            m = _FIELD_RE.match(line)
            if not m:
                continue

            field_raw_type = m.group(1)  # e.g. 'int8', 'pkg/Name'
            array_bracket = m.group(2)   # e.g. '[]' or None
            field_name = m.group(3)
            is_array = array_bracket is not None

            # Normalise type to full pkg/msg/Name
            full_field_type = cls._normalise_type(field_raw_type, interface_type)

            if field_raw_type in _NUMERIC_PRIMITIVES:
                if field_raw_type in inline_primitive_types:
                    result[field_name] = FieldInfo(
                        enum_type=interface_type,
                        primitive=field_raw_type,
                        is_array=is_array,
                    )
                else:
                    result[field_name] = FieldInfo(
                        enum_type=None,
                        primitive=None,
                        is_array=is_array,
                    )

            elif full_field_type is not None:
                # Field type is a message (nested or enum-only)
                result[field_name] = FieldInfo(
                    enum_type=full_field_type,
                    primitive=None,
                    is_array=is_array,
                )
            else:
                result[field_name] = FieldInfo(
                    enum_type=None,
                    primitive=None,
                    is_array=is_array,
                )

        return result

    @staticmethod
    def _find_inline_primitive_types(interface_type: str) -> set:
        """Find which numeric primitive types have inline constants in this message."""
        found = set()
        for prim in _NUMERIC_PRIMITIVES:
            consts = EnumResolver.get_constants_for_primitive(interface_type, prim)
            if consts:
                found.add(prim)
        return found

    @staticmethod
    def _normalise_type(raw_type: str, parent_interface_type: str) -> Optional[str]:
        """Expand a raw field type to a full pkg/msg/Name string."""
        if raw_type in _PRIMITIVES:
            return None
        parts = raw_type.split('/')
        if len(parts) == 3:
            return raw_type
        if len(parts) == 2:
            return f'{parts[0]}/msg/{parts[1]}'
        if len(parts) == 1:
            parent_parts = parent_interface_type.split('/')
            if len(parent_parts) >= 1:
                return f'{parent_parts[0]}/msg/{raw_type}'
        return None

    @staticmethod
    def _read_section(interface_type: str) -> Optional[str]:
        """Read and return the relevant section text for an interface type."""
        parts = [p for p in interface_type.split('/') if p]
        if len(parts) < 3:
            return None
        package, kind, name = parts[0], parts[1], parts[2]
        section = parts[3] if len(parts) >= 4 else ''

        try:
            share = get_package_share_directory(package)
        except PackageNotFoundError:
            return None

        path = Path(share) / kind / f'{name}.{kind}'
        if not path.exists():
            return None

        content = path.read_text(encoding='utf-8')
        sections: list = []
        current: list = []
        for line in content.splitlines():
            if line.strip() == '---':
                sections.append('\n'.join(current))
                current = []
            else:
                current.append(line)
        sections.append('\n'.join(current))

        srv_map = {'Request': 0, 'Response': 1}
        action_map = {'Goal': 0, 'Result': 1, 'Feedback': 2}

        if kind == 'srv':
            idx = srv_map.get(section, 0)
        elif kind == 'action':
            idx = action_map.get(section, 0)
        else:
            idx = 0

        return sections[idx] if idx < len(sections) else None