#include "sire/core/base_factory.hpp"

#include "gtest/gtest.h"

#include "sire/core/event_base.hpp"
using namespace sire::core;
// 单例没法测试，只能先写了，不管那么多
GTEST_TEST(EventBaseRegister, DefaultConstructor) {
  // EventBaseRegister<EventBase> reg;
  // reg.registration("hello");
  // std::unique_ptr<EventBase> ptr =
  //     BaseFactory<EventBase>::instance().createObject("hello");
//   std::unique_ptr<EventBase> ptr = std::make_unique<EventBase>();
//   ptr->setEventType("yes");
//   EXPECT_EQ(ptr->eventType(), "yes");
}
