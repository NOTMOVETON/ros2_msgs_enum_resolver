// Copyright 2024 Your Name
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_msgs_enum_resolver
{

class EnumResolver
{
public:
  // --------------------------------------------------------------------------
  // Forward resolution: value -> list of names
  //
  // field_type is optional. When empty, searches all constants in the
  // interface file (use this when the field type IS the enum message, e.g.
  // ura_msgs/msg/ServerConnectionEventEnum).
  //
  // When field_type is a primitive type name such as "int8" or "uint16",
  // only constants declared with that exact type are searched. This handles
  // the inline-constant pattern where one message file contains constants of
  // multiple types (e.g. sensor_msgs/msg/NavSatStatus has both int8 STATUS_*
  // and uint16 SERVICE_* constants alongside regular fields).
  //
  // Returns a vector because one value may map to multiple names (aliases).
  // Returns std::nullopt when the type cannot be loaded or the value is absent.
  // --------------------------------------------------------------------------
  static std::optional<std::vector<std::string>> resolve(
    const std::string & interface_type,
    int64_t             value,
    const std::string & field_type = "");

  // --------------------------------------------------------------------------
  // Reverse resolution: name -> value
  // --------------------------------------------------------------------------
  static std::optional<int64_t> reverseResolve(
    const std::string & interface_type,
    const std::string & name);

  // --------------------------------------------------------------------------
  // Bulk access
  // --------------------------------------------------------------------------

  /// All constants as ordered map: value -> [name, ...]. Nullopt if not found.
  static std::optional<std::map<int64_t, std::vector<std::string>>> getConstants(
    const std::string & interface_type);

  /// All constants as unordered map: name -> value. Nullopt if not found.
  static std::optional<std::unordered_map<std::string, int64_t>> getConstantsReverse(
    const std::string & interface_type);

  // --------------------------------------------------------------------------
  // Introspection helpers (used by field_type_parser)
  // --------------------------------------------------------------------------

  /// True if the interface has no regular fields — only constant declarations.
  /// This is the pattern used by dedicated enum messages such as
  /// ServerConnectionEventEnum.msg.
  static bool isEnumOnlyType(const std::string & interface_type);

  /// Returns constants whose declared primitive type matches field_type.
  /// For example, getConstantsForPrimitive("sensor_msgs/msg/NavSatStatus",
  /// "int8") returns only STATUS_NO_FIX, STATUS_FIX, etc.
  /// Returns std::nullopt when no matching constants exist.
  static std::optional<std::map<int64_t, std::vector<std::string>>>
  getConstantsForPrimitive(
    const std::string & interface_type,
    const std::string & field_type);

  // --------------------------------------------------------------------------
  // Cache management
  // --------------------------------------------------------------------------

  /// Pre-warm the cache for the given type. Returns true on success.
  static bool preload(const std::string & interface_type);

  /// Flush the entire in-memory cache. Next call re-reads from disk.
  static void clearCache();

  // --------------------------------------------------------------------------
  // Diagnostics
  // --------------------------------------------------------------------------

  /// True if the type string resolves to a loadable interface file.
  static bool isAvailable(const std::string & interface_type);

  /// True if every value maps to exactly one name (no aliases).
  static bool isUnambiguous(const std::string & interface_type);

private:
  struct TypeInfo
  {
    std::string package;
    std::string kind;
    std::string name;
    std::string section;
  };

  struct ParsedConstants
  {
    /// value -> [name, ...]  (all constants, regardless of declared type)
    std::map<int64_t, std::vector<std::string>> forward;
    /// name -> value
    std::unordered_map<std::string, int64_t> reverse;
    /// primitive_type -> (value -> [name, ...])
    /// Populated lazily by getConstantsForPrimitive / resolve(field_type).
    std::unordered_map<
      std::string,
      std::map<int64_t, std::vector<std::string>>> by_type;
  };

  static std::mutex s_mutex;
  static std::unordered_map<std::string, ParsedConstants> s_cache;
  static std::unordered_map<std::string, bool> s_not_found;

  static bool parseTypeString(const std::string & type, TypeInfo & out);
  static std::string buildFilePath(const TypeInfo & info);
  static std::string readFile(const std::string & path);
  static std::vector<std::string> splitSections(const std::string & content);

  /// Parse all constants from a section (populates forward + reverse).
  static ParsedConstants parseSection(const std::string & section_content);

  /// Parse constants of a specific declared type from a section.
  static std::map<int64_t, std::vector<std::string>> parseSectionByType(
    const std::string & section_content,
    const std::string & field_type);

  /// True when every meaningful line in the section is a constant.
  static bool sectionHasOnlyConstants(const std::string & section_content);

  static std::optional<ParsedConstants> loadType(const std::string & interface_type);
  static const ParsedConstants * getOrLoad(const std::string & interface_type);

  /// Ensure by_type[field_type] is populated. Must be called under s_mutex.
  static void ensureByType(
    ParsedConstants & pc,
    const std::string & section_content,
    const std::string & field_type);
};

}  // namespace ros2_msgs_enum_resolver