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

#include "ros2_msgs_enum_resolver/ros2_msgs_enum_resolver.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace ros2_msgs_enum_resolver
{

// ============================================================================
// Static members
// ============================================================================

std::mutex EnumResolver::s_mutex;
std::unordered_map<std::string, EnumResolver::ParsedConstants>
    EnumResolver::s_cache;
std::unordered_map<std::string, bool> EnumResolver::s_not_found;

// ============================================================================
// Public API
// ============================================================================

std::optional<std::vector<std::string>> EnumResolver::resolve(
    const std::string& interface_type,
    int64_t value,
    const std::string& field_type
)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc)
  {
    return std::nullopt;
  }

  // field_type provided -> search only within that declared type
  if (!field_type.empty())
  {
    // Check if by_type already has this field_type cached
    {
      std::lock_guard<std::mutex> lock(s_mutex);
      auto bt_it = pc->by_type.find(field_type);
      if (bt_it != pc->by_type.end())
      {
        const auto it = bt_it->second.find(value);
        if (it == bt_it->second.end())
        {
          return std::nullopt;
        }
        return it->second;
      }
    }

    // Need to build by_type for this field_type — reload section content
    TypeInfo info;
    if (!parseTypeString(interface_type, info))
    {
      return std::nullopt;
    }
    const std::string content = readFile(buildFilePath(info));
    if (content.empty())
    {
      return std::nullopt;
    }

    const auto sections = splitSections(content);
    std::size_t idx = 0;
    if (info.kind == "srv" && info.section == "Response")
    {
      idx = 1;
    }
    else if (info.kind == "action" && info.section == "Result")
    {
      idx = 1;
    }
    else if (info.kind == "action" && info.section == "Feedback")
    {
      idx = 2;
    }
    if (idx >= sections.size())
    {
      return std::nullopt;
    }

    auto by_type_map = parseSectionByType(sections[idx], field_type);

    {
      std::lock_guard<std::mutex> lock(s_mutex);
      // Non-const pointer needed to mutate cache entry
      auto& cached_pc = s_cache.at(interface_type);
      cached_pc.by_type[field_type] = by_type_map;
      const auto it = cached_pc.by_type[field_type].find(value);
      if (it == cached_pc.by_type[field_type].end())
      {
        return std::nullopt;
      }
      return it->second;
    }
  }

  // No field_type -> search all constants
  const auto it = pc->forward.find(value);
  if (it == pc->forward.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<int64_t> EnumResolver::reverseResolve(
    const std::string& interface_type,
    const std::string& name
)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc)
  {
    return std::nullopt;
  }
  const auto it = pc->reverse.find(name);
  if (it == pc->reverse.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::map<int64_t, std::vector<std::string>>>
EnumResolver::getConstants(const std::string& interface_type)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc || pc->forward.empty())
  {
    return std::nullopt;
  }
  return pc->forward;
}

std::optional<std::unordered_map<std::string, int64_t>>
EnumResolver::getConstantsReverse(const std::string& interface_type)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc || pc->reverse.empty())
  {
    return std::nullopt;
  }
  return pc->reverse;
}

std::optional<std::map<int64_t, std::vector<std::string>>>
EnumResolver::getConstantsForPrimitive(
    const std::string& interface_type,
    const std::string& field_type
)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc)
  {
    return std::nullopt;
  }

  // Check cache
  {
    std::lock_guard<std::mutex> lock(s_mutex);
    auto it = pc->by_type.find(field_type);
    if (it != pc->by_type.end())
    {
      if (it->second.empty())
      {
        return std::nullopt;
      }
      return it->second;
    }
  }

  // Build
  TypeInfo info;
  if (!parseTypeString(interface_type, info))
  {
    return std::nullopt;
  }
  const std::string content = readFile(buildFilePath(info));
  if (content.empty())
  {
    return std::nullopt;
  }

  const auto sections = splitSections(content);
  std::size_t idx = 0;
  if (info.kind == "srv" && info.section == "Response")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Result")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Feedback")
  {
    idx = 2;
  }
  if (idx >= sections.size())
  {
    return std::nullopt;
  }

  auto result = parseSectionByType(sections[idx], field_type);

  {
    std::lock_guard<std::mutex> lock(s_mutex);
    s_cache.at(interface_type).by_type[field_type] = result;
  }

  if (result.empty())
  {
    return std::nullopt;
  }
  return result;
}

bool EnumResolver::isEnumOnlyType(const std::string& interface_type)
{
  TypeInfo info;
  if (!parseTypeString(interface_type, info))
  {
    return false;
  }
  const std::string content = readFile(buildFilePath(info));
  if (content.empty())
  {
    return false;
  }

  const auto sections = splitSections(content);
  std::size_t idx = 0;
  if (info.kind == "srv" && info.section == "Response")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Result")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Feedback")
  {
    idx = 2;
  }
  if (idx >= sections.size())
  {
    return false;
  }

  return sectionHasOnlyConstants(sections[idx]);
}

bool EnumResolver::preload(const std::string& interface_type)
{
  return getOrLoad(interface_type) != nullptr;
}

void EnumResolver::clearCache()
{
  std::lock_guard<std::mutex> lock(s_mutex);
  s_cache.clear();
  s_not_found.clear();
}

bool EnumResolver::isAvailable(const std::string& interface_type)
{
  return getOrLoad(interface_type) != nullptr;
}

bool EnumResolver::isUnambiguous(const std::string& interface_type)
{
  const auto* pc = getOrLoad(interface_type);
  if (!pc)
  {
    return false;
  }
  for (const auto& kv : pc->forward)
  {
    if (kv.second.size() > 1)
    {
      return false;
    }
  }
  return true;
}

// ============================================================================
// Private helpers
// ============================================================================

bool EnumResolver::parseTypeString(const std::string& type, TypeInfo& out)
{
  std::vector<std::string> parts;
  {
    std::istringstream ss(type);
    std::string token;
    while (std::getline(ss, token, '/'))
    {
      if (!token.empty())
      {
        parts.push_back(token);
      }
    }
  }

  if (parts.size() < 3)
  {
    return false;
  }

  out.package = parts[0];
  out.kind = parts[1];
  out.name = parts[2];
  out.section = "";

  if (out.kind != "msg" && out.kind != "srv" && out.kind != "action")
  {
    return false;
  }

  if (parts.size() >= 4)
  {
    out.section = parts[3];
    if (out.kind == "srv")
    {
      if (out.section != "Request" && out.section != "Response")
      {
        return false;
      }
    }
    else if (out.kind == "action")
    {
      if (out.section != "Goal" && out.section != "Result" &&
          out.section != "Feedback")
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  return true;
}

std::string EnumResolver::buildFilePath(const TypeInfo& info)
{
  try
  {
    const std::string share =
        ament_index_cpp::get_package_share_directory(info.package);
    return share + "/" + info.kind + "/" + info.name + "." + info.kind;
  } catch (const std::exception&)
  {
    return {};
  }
}

std::string EnumResolver::readFile(const std::string& path)
{
  if (path.empty())
  {
    return {};
  }
  std::ifstream file(path);
  if (!file.is_open())
  {
    return {};
  }
  std::ostringstream buf;
  buf << file.rdbuf();
  return buf.str();
}

std::vector<std::string> EnumResolver::splitSections(const std::string& content)
{
  std::vector<std::string> sections;
  std::string current;
  std::istringstream stream(content);
  std::string line;

  while (std::getline(stream, line))
  {
    std::string trimmed = line;
    const auto first = trimmed.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
      trimmed.clear();
    }
    else
    {
      trimmed = trimmed.substr(first);
      const auto last = trimmed.find_last_not_of(" \t\r\n");
      if (last != std::string::npos)
      {
        trimmed = trimmed.substr(0, last + 1);
      }
    }
    if (trimmed == "---")
    {
      sections.push_back(std::move(current));
      current.clear();
    }
    else
    {
      current += line;
      current += '\n';
    }
  }
  sections.push_back(std::move(current));
  return sections;
}

EnumResolver::ParsedConstants EnumResolver::parseSection(
    const std::string& section_content
)
{
  ParsedConstants result;

  // Matches:  <type>  <UPPER_NAME>  =  <value>
  static const std::regex kConstRegex(
      "^\\s*(?:bool|byte|char|float(?:32|64)|u?int(?:8|16|32|64))\\s+"
      "([A-Z][A-Z0-9_]*)\\s*=\\s*(-?(?:0[xX][0-9a-fA-F]+|\\d+))",
      std::regex::optimize
  );

  std::istringstream stream(section_content);
  std::string line;
  std::smatch match;

  while (std::getline(stream, line))
  {
    const auto cp = line.find('#');
    if (cp != std::string::npos)
    {
      line = line.substr(0, cp);
    }
    if (!std::regex_search(line, match, kConstRegex))
    {
      continue;
    }
    const std::string const_name = match[1].str();
    const int64_t const_value = std::stoll(match[2].str(), nullptr, 0);
    result.forward[const_value].push_back(const_name);
    result.reverse[const_name] = const_value;
  }
  return result;
}

std::map<int64_t, std::vector<std::string>> EnumResolver::parseSectionByType(
    const std::string& section_content,
    const std::string& field_type
)
{
  std::map<int64_t, std::vector<std::string>> result;

  // Matches:  <exact_type>  <UPPER_NAME>  =  <value>
  // We build the regex dynamically based on field_type.
  const std::string pattern =
      "^\\s*(" + field_type +
      ")\\s+"
      "([A-Z][A-Z0-9_]*)\\s*=\\s*(-?(?:0[xX][0-9a-fA-F]+|\\d+))";

  std::regex typed_re;
  try
  {
    typed_re = std::regex(pattern, std::regex::optimize);
  } catch (const std::regex_error&)
  {
    return result;
  }

  std::istringstream stream(section_content);
  std::string line;
  std::smatch match;

  while (std::getline(stream, line))
  {
    const auto cp = line.find('#');
    if (cp != std::string::npos)
    {
      line = line.substr(0, cp);
    }
    if (!std::regex_search(line, match, typed_re))
    {
      continue;
    }
    const std::string const_name = match[2].str();
    const int64_t const_value = std::stoll(match[3].str(), nullptr, 0);
    result[const_value].push_back(const_name);
  }
  return result;
}

bool EnumResolver::sectionHasOnlyConstants(const std::string& section_content)
{
  // A regular (non-constant) field line looks like:
  //   <type> <lowercase_name>
  // without an '=' sign.
  static const std::regex kFieldRegex(
      "^\\s*(?:bool|byte|char|string|wstring|float(?:32|64)|u?int(?:8|16|32|64)"
      "|[a-zA-Z][a-zA-Z0-9_]*/[a-zA-Z][a-zA-Z0-9_/]*)"
      "\\s+[a-z][a-zA-Z0-9_]*(\\[.*?\\])?(\\s+\\S+)?\\s*$",
      std::regex::optimize
  );

  bool has_any = false;
  std::istringstream stream(section_content);
  std::string line;

  while (std::getline(stream, line))
  {
    const auto cp = line.find('#');
    if (cp != std::string::npos)
    {
      line = line.substr(0, cp);
    }
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
      continue;
    }
    has_any = true;
    // constant lines always contain '='
    if (line.find('=') != std::string::npos)
    {
      continue;
    }
    // line has no '=' -> check if it looks like a field
    if (std::regex_search(line, kFieldRegex))
    {
      return false;
    }
  }
  return has_any;
}

std::optional<EnumResolver::ParsedConstants> EnumResolver::loadType(
    const std::string& interface_type
)
{
  TypeInfo info;
  if (!parseTypeString(interface_type, info))
  {
    return std::nullopt;
  }

  const std::string content = readFile(buildFilePath(info));
  if (content.empty())
  {
    return std::nullopt;
  }

  const auto sections = splitSections(content);

  std::size_t idx = 0;
  if (info.kind == "srv" && info.section == "Response")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Result")
  {
    idx = 1;
  }
  else if (info.kind == "action" && info.section == "Feedback")
  {
    idx = 2;
  }

  if (idx >= sections.size())
  {
    return std::nullopt;
  }
  return parseSection(sections[idx]);
}

const EnumResolver::ParsedConstants* EnumResolver::getOrLoad(
    const std::string& interface_type
)
{
  // Fast path
  {
    std::lock_guard<std::mutex> lock(s_mutex);
    const auto it = s_cache.find(interface_type);
    if (it != s_cache.end())
    {
      return &it->second;
    }
    if (s_not_found.count(interface_type))
    {
      return nullptr;
    }
  }

  // Slow path - I/O outside lock
  auto parsed = loadType(interface_type);

  // Insert under lock with double-check
  {
    std::lock_guard<std::mutex> lock(s_mutex);
    const auto it = s_cache.find(interface_type);
    if (it != s_cache.end())
    {
      return &it->second;
    }
    if (parsed)
    {
      auto& ref = s_cache[interface_type];
      ref = std::move(*parsed);
      return &ref;
    }
    s_not_found[interface_type] = true;
    return nullptr;
  }
}

}  // namespace ros2_msgs_enum_resolver