#ifndef SIRE_CONTACT_PAIR_MANAGER_HPP_
#define SIRE_CONTACT_PAIR_MANAGER_HPP_
#include <unordered_set>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

#include "sire/core/sorted_pair.hpp"
namespace sire::physics {
namespace contact {
enum class ContactPairState {
  START,
  CONTACTING,
  END,
};
enum class ContactPairType { SPLIT, STICK };
struct ContactPairValue {
  ContactPairState state;
  ContactPairType type;
  ContactPairValue()
      : state(ContactPairState::START), type(ContactPairType::SPLIT) {}
};
class ContactPairManager {
 public:
  auto getContactingSet() const -> const std::unordered_set<core::SortedPair>&;
  auto insert(const core::SortedPair& pair) -> void;
  auto insert(const core::SortedPair& pair, const ContactPairValue& value)
      -> void;
  auto contains(const core::SortedPair& pair) const -> bool;
  auto getValue(const core::SortedPair& pair) const -> const ContactPairValue&;
  auto getValue(const core::SortedPair& pair) -> ContactPairValue& {
    return const_cast<ContactPairValue&>(
        static_cast<const ContactPairManager*>(this)->getValue(pair));
  }
  auto setValue(const core::SortedPair& pair, const ContactPairValue& value)
      -> void;
  auto setState(const core::SortedPair& pair, ContactPairState state) -> void;
  auto setType(const core::SortedPair& pair, ContactPairType type) -> void;
  auto init() -> void;
  ContactPairManager();
  ~ContactPairManager();
  ARIS_DECLARE_BIG_FOUR(ContactPairManager);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace contact
}  // namespace sire::physics
#endif