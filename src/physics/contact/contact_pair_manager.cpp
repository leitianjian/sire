#include "sire/physics/contact/contact_pair_manager.hpp"

#include <unordered_map>

namespace sire::physics::contact {
using namespace sire::core;
struct ContactPairManager::Imp {
  std::unordered_map<SortedPair, ContactPairValue> map_;

  // 正在接触的Set
  std::unordered_set<SortedPair> contacting_set_;
};
auto ContactPairManager::init() -> void {}
auto ContactPairManager::contains(const core::SortedPair& pair) const -> bool {
  if (imp_->map_.find(pair) != imp_->map_.end()) {
    return true;
  } else {
    return false;
  }
}
auto ContactPairManager::insert(const core::SortedPair& pair) -> void {
  imp_->map_.insert({pair, ContactPairValue()});
}
auto ContactPairManager::insert(const core::SortedPair& pair,
                                const ContactPairValue& value) -> void {
  if (value.state == ContactPairState::CONTACTING) {
    imp_->contacting_set_.insert(pair);
  }
  imp_->map_.insert({pair, value});
}
auto ContactPairManager::getContactingSet() const
    -> const std::unordered_set<SortedPair>& {
  return imp_->contacting_set_;
}

// Throw out_of_range error when map not find pair
auto ContactPairManager::getValue(const core::SortedPair& pair) const
    -> const ContactPairValue& {
  return imp_->map_.at(pair);
}
auto ContactPairManager::setValue(const core::SortedPair& pair,
                                  const ContactPairValue& value) -> void {
  if (auto search = imp_->map_.find(pair); search != imp_->map_.end()) {
    if (value.state == ContactPairState::CONTACTING) {
      imp_->contacting_set_.insert(pair);
    }
    search->second = value;
  }
}
auto ContactPairManager::setState(const core::SortedPair& pair,
                                  ContactPairState state) -> void {
  if (auto search = imp_->map_.find(pair); search != imp_->map_.end()) {
    if (state == ContactPairState::CONTACTING) {
      imp_->contacting_set_.insert(pair);
    }
    search->second.state = state;
  }
}
auto ContactPairManager::setType(const core::SortedPair& pair,
                                 ContactPairType type) -> void {
  if (auto search = imp_->map_.find(pair); search != imp_->map_.end()) {
    search->second.type = type;
  }
}

ContactPairManager::ContactPairManager() : imp_(new Imp()){};
ContactPairManager::~ContactPairManager() = default;
ARIS_DEFINE_BIG_FOUR_CPP(ContactPairManager);
}  // namespace sire::physics::contact