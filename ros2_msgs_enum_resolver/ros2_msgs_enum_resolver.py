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

"""ros2_msgs_enum_resolver - Python EnumResolver.

Supported type strings::

    pkg/msg/TypeName
    pkg/srv/TypeName[/Request|/Response]          default: Request
    pkg/action/TypeName[/Goal|/Result|/Feedback]  default: Goal

Example
-------
::

    from ros2_msgs_enum_resolver import EnumResolver

    # Enum-only message type (field type IS the enum)
    names = EnumResolver.resolve('ura_msgs/msg/ServerConnectionEventEnum', 1)

    # Inline constants (NavSatStatus pattern)
    names = EnumResolver.resolve('sensor_msgs/msg/NavSatStatus', 0, 'int8')

"""

from __future__ import annotations

from pathlib import Path
import re
import threading
from typing import Dict, List, Optional

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

_VALID_KINDS = {'msg', 'srv', 'action'}
_SRV_SECTIONS = {'Request': 0, 'Response': 1}
_ACTION_SECTIONS = {'Goal': 0, 'Result': 1, 'Feedback': 2}

# Matches any constant (all types)
_CONST_RE = re.compile(
    r'^\s*(?:bool|byte|char|float(?:32|64)|u?int(?:8|16|32|64))\s+'
    r'([A-Z][A-Z0-9_]*)\s*=\s*(-?(?:0[xX][0-9a-fA-F]+|\d+))'
)

# Matches a constant and captures its declared type
_TYPED_CONST_RE = re.compile(
    r'^\s*(bool|byte|char|float(?:32|64)|u?int(?:8|16|32|64))\s+'
    r'([A-Z][A-Z0-9_]*)\s*=\s*(-?(?:0[xX][0-9a-fA-F]+|\d+))'
)

# Matches a regular field (lowercase name, no '=' sign)
_FIELD_RE = re.compile(
    r'^\s*(?:bool|byte|char|string|wstring|float(?:32|64)|u?int(?:8|16|32|64)'
    r'|[a-zA-Z][a-zA-Z0-9_]*/[a-zA-Z][a-zA-Z0-9_/]*)'
    r'\s+[a-z][a-zA-Z0-9_]*(\[.*?\])?\s*$'
)


class _TypeInfo:
    __slots__ = ('package', 'kind', 'name', 'section')

    def __init__(self, package: str, kind: str, name: str, section: str) -> None:
        self.package = package
        self.kind = kind
        self.name = name
        self.section = section


class _ParsedConstants:
    __slots__ = ('forward', 'reverse', 'by_type')

    def __init__(self) -> None:
        self.forward: Dict[int, List[str]] = {}
        self.reverse: Dict[str, int] = {}
        # primitive_type -> {value: [name, ...]}
        self.by_type: Dict[str, Dict[int, List[str]]] = {}


class EnumResolver:
    """Static class - use class methods directly, no instantiation needed.

    Thread-safe lazy cache backed by threading.Lock.
    """

    _lock: threading.Lock = threading.Lock()
    _cache: Dict[str, _ParsedConstants] = {}
    _not_found: set = set()

    @classmethod
    def resolve(
        cls,
        interface_type: str,
        value: int,
        field_type: str = '',
    ) -> Optional[List[str]]:
        """Resolve value to list of names, or None if not found.

        When field_type is empty, searches all constants in the interface.
        When field_type is a primitive type name such as 'int8', only
        constants declared with that exact type are searched.
        """
        pc = cls._get_or_load(interface_type)
        if pc is None:
            return None

        if field_type:
            by_type_map = cls._get_by_type(interface_type, pc, field_type)
            if by_type_map is None:
                return None
            return by_type_map.get(value)

        return pc.forward.get(value)

    @classmethod
    def reverse_resolve(cls, interface_type: str, name: str) -> Optional[int]:
        """Resolve name to numeric value, or None if not found."""
        pc = cls._get_or_load(interface_type)
        if pc is None:
            return None
        return pc.reverse.get(name)

    @classmethod
    def get_constants(cls, interface_type: str) -> Optional[Dict[int, List[str]]]:
        """Return all constants sorted by value, or None if not found."""
        pc = cls._get_or_load(interface_type)
        if pc is None or not pc.forward:
            return None
        return dict(sorted(pc.forward.items()))

    @classmethod
    def get_constants_reverse(cls, interface_type: str) -> Optional[Dict[str, int]]:
        """Return all constants as name->value dict, or None if not found."""
        pc = cls._get_or_load(interface_type)
        if pc is None or not pc.reverse:
            return None
        return dict(pc.reverse)

    @classmethod
    def get_constants_for_primitive(
        cls,
        interface_type: str,
        field_type: str,
    ) -> Optional[Dict[int, List[str]]]:
        """Return constants whose declared type matches field_type exactly."""
        pc = cls._get_or_load(interface_type)
        if pc is None:
            return None
        result = cls._get_by_type(interface_type, pc, field_type)
        if not result:
            return None
        return dict(sorted(result.items()))

    @classmethod
    def is_enum_only_type(cls, interface_type: str) -> bool:
        """Check whether the interface has only constants and no regular fields."""
        info = cls._parse_type_string(interface_type)
        if info is None:
            return False
        path = cls._build_file_path(info)
        if path is None:
            return False
        sections = cls._split_sections(path.read_text(encoding='utf-8'))
        if info.kind == 'srv':
            idx = _SRV_SECTIONS.get(info.section, 0)
        elif info.kind == 'action':
            idx = _ACTION_SECTIONS.get(info.section, 0)
        else:
            idx = 0
        if idx >= len(sections):
            return False
        return cls._section_has_only_constants(sections[idx])

    @classmethod
    def preload(cls, interface_type: str) -> bool:
        """Pre-warm the cache for the given type. Return True on success."""
        return cls._get_or_load(interface_type) is not None

    @classmethod
    def clear_cache(cls) -> None:
        """Flush the entire in-memory cache."""
        with cls._lock:
            cls._cache.clear()
            cls._not_found.clear()

    @classmethod
    def is_available(cls, interface_type: str) -> bool:
        """Check whether the type resolves to a loadable interface file."""
        return cls._get_or_load(interface_type) is not None

    @classmethod
    def is_unambiguous(cls, interface_type: str) -> bool:
        """Check whether every value maps to exactly one name (no aliases)."""
        pc = cls._get_or_load(interface_type)
        if pc is None:
            return False
        return all(len(names) == 1 for names in pc.forward.values())

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @classmethod
    def _get_or_load(cls, interface_type: str) -> Optional[_ParsedConstants]:
        with cls._lock:
            if interface_type in cls._cache:
                return cls._cache[interface_type]
            if interface_type in cls._not_found:
                return None
        parsed = cls._load_type(interface_type)
        with cls._lock:
            if interface_type in cls._cache:
                return cls._cache[interface_type]
            if parsed is not None:
                cls._cache[interface_type] = parsed
                return parsed
            cls._not_found.add(interface_type)
            return None

    @classmethod
    def _get_by_type(
        cls,
        interface_type: str,
        pc: _ParsedConstants,
        field_type: str,
    ) -> Optional[Dict[int, List[str]]]:
        """Return (and cache) constants filtered by declared primitive type."""
        with cls._lock:
            if field_type in pc.by_type:
                return pc.by_type[field_type] or None

        info = cls._parse_type_string(interface_type)
        if info is None:
            return None
        path = cls._build_file_path(info)
        if path is None:
            return None

        sections = cls._split_sections(path.read_text(encoding='utf-8'))
        if info.kind == 'srv':
            idx = _SRV_SECTIONS.get(info.section, 0)
        elif info.kind == 'action':
            idx = _ACTION_SECTIONS.get(info.section, 0)
        else:
            idx = 0
        if idx >= len(sections):
            return None

        result = cls._parse_section_by_type(sections[idx], field_type)
        with cls._lock:
            pc.by_type[field_type] = result
        return result or None

    @staticmethod
    def _parse_type_string(s: str) -> Optional[_TypeInfo]:
        parts = [p for p in s.split('/') if p]
        if len(parts) < 3:
            return None
        package, kind, name = parts[0], parts[1], parts[2]
        section = ''
        if kind not in _VALID_KINDS:
            return None
        if len(parts) >= 4:
            section = parts[3]
            if kind == 'srv' and section not in _SRV_SECTIONS:
                return None
            if kind == 'action' and section not in _ACTION_SECTIONS:
                return None
            if kind == 'msg':
                return None
        return _TypeInfo(package, kind, name, section)

    @staticmethod
    def _build_file_path(info: _TypeInfo) -> Optional[Path]:
        try:
            share = get_package_share_directory(info.package)
        except PackageNotFoundError:
            return None
        p = Path(share) / info.kind / f'{info.name}.{info.kind}'
        return p if p.exists() else None

    @staticmethod
    def _split_sections(content: str) -> List[str]:
        sections: List[str] = []
        current: List[str] = []
        for line in content.splitlines():
            if line.strip() == '---':
                sections.append('\n'.join(current))
                current = []
            else:
                current.append(line)
        sections.append('\n'.join(current))
        return sections

    @staticmethod
    def _parse_section(section_content: str) -> _ParsedConstants:
        pc = _ParsedConstants()
        for line in section_content.splitlines():
            comment = line.find('#')
            if comment != -1:
                line = line[:comment]
            m = _CONST_RE.match(line)
            if not m:
                continue
            const_name = m.group(1)
            const_value = int(m.group(2), 0)
            pc.forward.setdefault(const_value, []).append(const_name)
            pc.reverse[const_name] = const_value
        return pc

    @staticmethod
    def _parse_section_by_type(
        section_content: str,
        field_type: str,
    ) -> Dict[int, List[str]]:
        """Parse constants whose declared type matches field_type exactly."""
        result: Dict[int, List[str]] = {}
        for line in section_content.splitlines():
            comment = line.find('#')
            if comment != -1:
                line = line[:comment]
            m = _TYPED_CONST_RE.match(line)
            if not m:
                continue
            if m.group(1) != field_type:
                continue
            const_name = m.group(2)
            const_value = int(m.group(3), 0)
            result.setdefault(const_value, []).append(const_name)
        return result

    @staticmethod
    def _section_has_only_constants(section_content: str) -> bool:
        """Return True if every meaningful line is a constant declaration."""
        has_any = False
        for line in section_content.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            # strip inline comment
            no_comment = stripped.split('#')[0].strip()
            if not no_comment:
                continue
            has_any = True
            # constant lines always contain '='
            if '=' in no_comment:
                continue
            # regular field - check it matches field pattern
            if _FIELD_RE.match(line):
                return False
        return has_any

    @classmethod
    def _load_type(cls, interface_type: str) -> Optional[_ParsedConstants]:
        info = cls._parse_type_string(interface_type)
        if info is None:
            return None
        path = cls._build_file_path(info)
        if path is None:
            return None
        sections = cls._split_sections(path.read_text(encoding='utf-8'))
        idx = 0
        if info.kind == 'srv' and info.section == 'Response':
            idx = 1
        elif info.kind == 'action' and info.section == 'Result':
            idx = 1
        elif info.kind == 'action' and info.section == 'Feedback':
            idx = 2
        if idx >= len(sections):
            return None
        return cls._parse_section(sections[idx])