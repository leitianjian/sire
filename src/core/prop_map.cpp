#include "sire/core/prop_map.hpp"

#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

#include <aris/core/reflection.hpp>

#include "sire/core/string_utils.hpp"
namespace sire::core {
using std::unordered_map;
struct PropMap::Imp {
  unordered_map<string, double> map_;
};
auto PropMap::addProp(const string& name, double value) -> void {
  imp_->map_[name] = value;
}
auto PropMap::rmProp(const string& name) -> bool {
  if (auto search = imp_->map_.find(name); search != imp_->map_.end()) {
    imp_->map_.erase(search);
    return true;
  } else {
    return false;
  }
}
auto PropMap::updProp(const string& name, double new_value) -> void {
  imp_->map_[name] = new_value;
}
auto PropMap::clear() -> void { imp_->map_.clear(); }
auto PropMap::getPropValue(const string& name) const -> double {
  return imp_->map_.at(name);
}
auto PropMap::getPropValueOrDefault(const string& name,
                                    double default_value) const -> double {
  if (auto search = imp_->map_.find(name); search != imp_->map_.end()) {
    return search->second;
  } else {
    return default_value;
  }
}
auto PropMap::toString() const -> string {
  std::stringstream s;
  s.precision(15);
  s << "{";
  int i = 0, size = imp_->map_.size();
  for (auto& [key, value] : imp_->map_) {
    if (i < size - 1) {
      s << key << ":" << value << ",";
    } else {
      s << key << ":" << value;
    }
    ++i;
  }
  s << "}";
  return s.str();
}
auto PropMap::fromString(std::string_view str) -> bool {
  str = trim(str);
  str.remove_prefix(1);
  str.remove_suffix(1);
  const std::string_view delimiter = ",";
  size_t pos = 0;
  std::string_view pair;
  auto split_pair_and_add_prop = [this](std::string_view pair) {
    auto pair_del_pos = pair.find(":");
    if (pair_del_pos == pair.npos)
      THROW_FILE_LINE("PropMap反序列化错误，找不到键值对分隔符 :");
    std::string_view key = pair.substr(0, pair_del_pos);
    std::string_view value = pair.substr(pair_del_pos + 1);
    key = trim(key);
    value = trim(value);
    this->addProp(std::string(key), std::stod(std::string(value)));
  };
  while ((pos = str.find(delimiter)) != str.npos) {
    pair = str.substr(0, pos);
    split_pair_and_add_prop(pair);
    str.remove_prefix(pos + delimiter.length());
  }
  split_pair_and_add_prop(str);
  return true;
}
auto PropMap::swap(PropMap& other) noexcept -> PropMap& {
  std::swap(this->imp_->map_, other.imp_->map_);
  return *this;
}
PropMap::PropMap() : imp_(new Imp) {}
PropMap::~PropMap() = default;
PropMap::PropMap(const PropMap& other) = default;
PropMap::PropMap(PropMap&& other) { this->swap(other); }
PropMap& PropMap::operator=(const PropMap& other) = default;
PropMap& PropMap::operator=(PropMap&& other) {
  this->swap(other);
  return *this;
}

ARIS_REGISTRATION {
  aris::core::class_<PropMap>("PropMap").textMethod(
      [](PropMap* map) -> std::string { return map->toString(); },
      [](PropMap* map, std::string_view str) -> void {
        map->clear();
        if (!map->fromString(str))
          THROW_FILE_LINE("PropMap initial from string error");
      });
}
}  // namespace sire::core