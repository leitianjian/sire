#include "sire/core/contact_pair_manager.hpp"

#include <unordered_map>

namespace sire::core {
struct ContactPairManager::Imp {
  std::unordered_map<SortedPair<sire::PartId>, ContactPairValue>
      contact_pair_map_;
  std::unordered_set<sire::PartId> impacted_prt_set_;
  std::unordered_set<SortedPair<sire::PartId>> impacted_contact_set_;
  // 正在接触的Set
  // std::unordered_set<SortedPair<double>> contacting_set_;
};
auto ContactPairManager::init() -> void {}
auto ContactPairManager::containsContactPair(
    const core::SortedPair<sire::PartId>& pair) const -> bool {
  if (imp_->contact_pair_map_.find(pair) != imp_->contact_pair_map_.end()) {
    return true;
  } else {
    return false;
  }
}
auto ContactPairManager::hasImpactedPrt(sire::PartId prt_id) const -> bool {
  return imp_->impacted_prt_set_.count(prt_id) != 0;
}
auto ContactPairManager::isImpactedSetEmpty() const -> bool {
  return imp_->impacted_prt_set_.size() == 0;
}
auto ContactPairManager::contactPairMap()
    -> std::unordered_map<SortedPair<sire::PartId>, ContactPairValue>& {
  return imp_->contact_pair_map_;
}
auto ContactPairManager::impactedPrtSet() -> std::unordered_set<sire::PartId>& {
  return imp_->impacted_prt_set_;
}
auto ContactPairManager::impactedContactSet()
    -> std::unordered_set<core::SortedPair<sire::PartId>>& {
  return imp_->impacted_contact_set_;
}
auto ContactPairManager::insert(const sire::PartId ground,
                                const sire::PartId id_B,
                                double init_penetration_depth) -> void {
  imp_->contact_pair_map_.insert(
      {{ground, id_B}, ContactPairValue(init_penetration_depth)});
  imp_->impacted_prt_set_.insert(id_B);
}
auto ContactPairManager::insert(const core::SortedPair<sire::PartId>& pair,
                                double init_penetration_depth) -> void {
  imp_->contact_pair_map_.insert(
      {pair, ContactPairValue(init_penetration_depth)});
  imp_->impacted_prt_set_.insert(pair.first());
  imp_->impacted_prt_set_.insert(pair.second());
}
auto ContactPairManager::insert(const core::SortedPair<sire::PartId>& pair,
                                const ContactPairValue& value) -> void {
  // if (value.state == ContactPairState::CONTACTING) {
  //   imp_->contacting_set_.insert(pair);
  // }
  imp_->contact_pair_map_.insert({pair, value});
  imp_->impacted_prt_set_.insert(pair.first());
  imp_->impacted_prt_set_.insert(pair.second());
}
// auto ContactPairManager::getContactingSet() const
//     -> const std::unordered_set<SortedPair<double>>& {
//   return imp_->contacting_set_;
// }

// Throw out_of_range error when map not find pair
auto ContactPairManager::getValue(const core::SortedPair<sire::PartId>& pair)
    const -> const ContactPairValue& {
  return imp_->contact_pair_map_.at(pair);
}
auto ContactPairManager::setValue(const core::SortedPair<sire::PartId>& pair,
                                  const ContactPairValue& value) -> void {
  if (auto search = imp_->contact_pair_map_.find(pair);
      search != imp_->contact_pair_map_.end()) {
    // if (value.state == ContactPairState::CONTACTING) {
    //   imp_->contacting_set_.insert(pair);
    // }
    search->second = value;
  }
}
// auto ContactPairManager::setState(const core::SortedPair<double>& pair,
//                                   ContactPairState state) -> void {
//   if (auto search = imp_->map_.find(pair); search != imp_->map_.end()) {
//     // if (state == ContactPairState::CONTACTING) {
//     //   imp_->contacting_set_.insert(pair);
//     // }
//     search->second.state = state;
//   }
// }
// auto ContactPairManager::setType(const core::SortedPair<double>& pair,
//                                  ContactPairType type) -> void {
//   if (auto search = imp_->map_.find(pair); search != imp_->map_.end()) {
//     search->second.type = type;
//   }
// }

ContactPairManager::ContactPairManager() : imp_(new Imp()){};
ContactPairManager::~ContactPairManager() = default;
ARIS_DEFINE_BIG_FOUR_CPP(ContactPairManager);
}  // namespace sire::core