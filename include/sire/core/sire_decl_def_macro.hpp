#ifndef SIRE_DECL_DEF_MACRO_HPP_
#define SIRE_DECL_DEF_MACRO_HPP_

// 对于struct Imp中的unique_ptr没有copy相关的构造函数
// 定义用于move constructor的宏
#define SIRE_DECLARE_MOVE_CTOR(type_name) \
  type_name(type_name&& other);           \
  type_name& operator=(type_name&& other);

#define SIRE_DEFINE_MOVE_CTOR_CPP(type_name)         \
  type_name::type_name(type_name&& other) = default; \
  type_name& type_name::operator=(type_name&& other) = default;

#define SIRE_DECLARE_JSON_INTER_TWO auto to_json(nlohmann::json& j) const->void;
// auto from_json(const nlohmann::json& j)->void;

#define SIRE_DECLARE_JSON_INTER_VIRTUAL_TWO \
  auto virtual to_json(nlohmann::json& j) const->void;
// auto virtual from_json(const nlohmann::json& j)->void;

#define SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO \
  auto to_json(nlohmann::json& j) const->void override;
// auto from_json(const nlohmann::json& j)->void override;

#define SIRE_DECLARE_JSON_INTER_VIRTUAL_INTERFACE_TWO \
  auto virtual to_json(nlohmann::json& j) const->void = 0;
// auto virtual from_json(const nlohmann::json& j)->void = 0;

#define SIRE_DECLARE_JSON_FRIEND_TWO(type_name) \
  friend auto to_json(nlohmann::json& j, const type_name& box)->void;
// friend auto from_json(const nlohmann::json& j, type_name& box)->void;

#define SIRE_DEFINE_JSON_OUTER_TWO(type_name) \
  auto to_json(nlohmann::json& j, const type_name& o)->void { o.to_json(j); }
//auto from_json(const nlohmann::json& j, type_name& o)->void {               \
  //  o.from_json(j);                                                           \
  //}

#define SIRE_DEFINE_TO_JSON_HEAD(type_name) \
  auto type_name::to_json(nlohmann::json& j) const->void

#define SIRE_DEFINE_FROM_JSON_HEAD(type_name) \
  auto type_name::from_json(const nlohmann::json& j)->void
#endif