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

"""message_formatter - format a ROS2 message dict with enum name substitution.

Output format::

    connection_state: (1) CONNECTED
    mode: (1) INIT/STARTING
    service: 3
    nested:
      error_code: (3) ERR_TIMEOUT

"""

from __future__ import annotations

from typing import Any, Dict, Optional

from ros2_msgs_enum_resolver.field_type_parser import FieldInfo, FieldTypeParser
from ros2_msgs_enum_resolver.ros2_msgs_enum_resolver import EnumResolver


def format_message(
    msg_dict: Dict[str, Any],
    interface_type: str,
    indent: int = 0,
    enabled: bool = True,
) -> str:
    """Format a message dict as a human-readable string with enum substitution.

    Parameters
    ----------
    msg_dict:
        Ordered dict produced by rosidl_runtime_py.message_to_ordereddict.
    interface_type:
        Full type string, e.g. 'ura_msgs/msg/ServerStatus'.
    indent:
        Current indentation level (used for recursion).
    enabled:
        When False, no enum substitution is applied (plain echo mode).
    """
    if not enabled:
        return _format_plain(msg_dict, indent)

    fields = FieldTypeParser.get_fields(interface_type)
    lines = []
    prefix = '  ' * indent

    for key, value in msg_dict.items():
        info: Optional[FieldInfo] = fields.get(key)

        if isinstance(value, dict):
            lines.append(f'{prefix}{key}:')
            nested_type = info.enum_type if info else None
            if nested_type and not _is_enum_only(info):
                lines.append(
                    format_message(value, nested_type, indent + 1, enabled)
                )
            else:
                lines.append(_format_plain(value, indent + 1))

        elif isinstance(value, list):
            lines.append(f'{prefix}{key}:')
            lines.append(
                _format_list(value, info, interface_type, indent + 1, enabled)
            )

        elif isinstance(value, int) and info and info.enum_type:
            formatted = _resolve_value(value, info)
            lines.append(f'{prefix}{key}: {formatted}')

        else:
            lines.append(f'{prefix}{key}: {_scalar_str(value)}')

    return '\n'.join(lines)


def _is_enum_only(info: Optional[FieldInfo]) -> bool:
    """Check whether a FieldInfo refers to a dedicated enum-only message."""
    if info is None:
        return False
    # Dedicated enum messages have no primitive set
    return info.enum_type is not None and info.primitive is None


def _resolve_value(value: int, info: FieldInfo) -> str:
    """Resolve an integer to '(value) NAME' or just the value string."""
    names = EnumResolver.resolve(
        info.enum_type,
        value,
        info.primitive or '',
    )
    if names:
        names_str = '/'.join(names)
        return f'({value}) {names_str}'
    return str(value)


def _format_list(
    items: list,
    info: Optional[FieldInfo],
    parent_type: str,
    indent: int,
    enabled: bool,
) -> str:
    """Format a list field, applying enum resolution to each element."""
    prefix = '  ' * indent
    lines = []
    for item in items:
        if isinstance(item, dict):
            nested_type = info.enum_type if info else None
            if enabled and nested_type and not _is_enum_only(info):
                lines.append(f'{prefix}-')
                lines.append(
                    format_message(item, nested_type, indent + 1, enabled)
                )
            else:
                lines.append(f'{prefix}- {_format_plain(item, 0)}')
        elif isinstance(item, int) and enabled and info and info.enum_type:
            lines.append(f'{prefix}- {_resolve_value(item, info)}')
        else:
            lines.append(f'{prefix}- {_scalar_str(item)}')
    return '\n'.join(lines)


def _format_plain(obj: Any, indent: int) -> str:
    """Format without any enum substitution."""
    prefix = '  ' * indent
    if isinstance(obj, dict):
        lines = []
        for k, v in obj.items():
            if isinstance(v, dict):
                lines.append(f'{prefix}{k}:')
                lines.append(_format_plain(v, indent + 1))
            elif isinstance(v, list):
                lines.append(f'{prefix}{k}:')
                for item in v:
                    if isinstance(item, dict):
                        lines.append(f'{prefix}  -')
                        lines.append(_format_plain(item, indent + 2))
                    else:
                        lines.append(f'{prefix}  - {_scalar_str(item)}')
            else:
                lines.append(f'{prefix}{k}: {_scalar_str(v)}')
        return '\n'.join(lines)
    return f'{prefix}{_scalar_str(obj)}'


def _scalar_str(value: Any) -> str:
    """Convert a scalar value to its display string."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, bytes):
        return value.hex()
    return str(value)