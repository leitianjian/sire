#include "sire/physics/utils.hpp"

#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"

namespace sire::simulator {
using aris::dynamic::GeneralForce;
using aris::dynamic::SingleComponentForce;
auto compareAndCopy(const Geometry* src, Geometry* dest) -> void {
  // 检查内部是否一致 

}

auto compareAndCopy(const GeometryPool* src, GeometryPool* dest) -> void {
  if (src->size() != dest->size()) {
  } else {
    // 检查内部是否一致
  }
  for (sire::Size i = 0; i < src->size(); ++i) {
  }
}
auto compareAndCopy(const Marker* src, Marker* dest) -> void {
  //
}
auto compareAndCopy(const MarkerPool* src, MarkerPool* dest) -> void {
  if (src->size() != dest->size()) {
  } else {
    // 检查内部是否一致
  }
  for (sire::Size i = 0; i < src->size(); ++i) {
  }
}
auto compareAndCopy(const Part* src, Part* dest) -> void {
  //
}
auto compareAndCopy(const PartPool* src, PartPool* dest) -> void {
  if (src->size() != dest->size()) {
  
  } else {
  // 检查内部是否一致
  
  }
  for (sire::Size i = 0; i < src->size(); ++ i) {
  }
}
auto compareAndCopy(const Model* src, Model* dest) -> void {
  // 1. 确保两个Model结构一致
  //   (a) part pool结构检测
    // copy part pool data
    SIRE_ASSERT(src->partPool().size() == dest->partPool().size());
  sire::Size part_size = src->partPool().size();
  for (int i = 0; i < part_size; ++i) {
    dest->partPool().at(i).setPm(*src->partPool().at(i).pm());
    dest->partPool().at(i).setVs(src->partPool().at(i).vs());
    dest->partPool().at(i).setAs(src->partPool().at(i).as());
  }

  // copy motion pool data
  SIRE_ASSERT(src->motionPool().size() == dest->motionPool().size());
  sire::Size motion_size = src->motionPool().size();
  for (int i = 0; i < motion_size; ++i) {
    dest->motionPool().at(i).setMp(src->motionPool().at(i).mp());
    dest->motionPool().at(i).setMv(src->motionPool().at(i).mv());
    dest->motionPool().at(i).setMa(src->motionPool().at(i).ma());
    dest->motionPool().at(i).setMf(src->motionPool().at(i).mf());
  }

  // copy joint pool data
  // joint pool 里面的数据不一定会变，会变也不一定会给计算结果带来影响
  // joint 是纯约束

  // general motion pool data copy
  // TODO(leitianjian): 这里先只考虑generalMotion，考虑其他的话情况就太多了，
  // 更理想的做法应该是使用Deepcopy相关的方法而不是手动复制黏贴
  // 或者让他们自己更新就好了，不用手动复制，设置完motion和part就可以更新generalMotion的位置了

  // force pool data copy
  SIRE_ASSERT(src->forcePool().size() == dest->forcePool().size());
  sire::Size force_size = src->forcePool().size();
  for (int i = 0; i < force_size; ++i) {
    try {
      dynamic_cast<SingleComponentForce&>(dest->forcePool().at(i))
          .setFce(
              dynamic_cast<const SingleComponentForce&>(src->forcePool().at(i))
                  .fce(),
              dynamic_cast<const SingleComponentForce&>(src->forcePool().at(i))
                  .componentAxis());

    } catch (std::bad_cast& e) {
    }

    try {
      dynamic_cast<aris::dynamic::GeneralForce&>(dest->forcePool().at(i))
          .setFce(
              dynamic_cast<const GeneralForce&>(src->forcePool().at(i)).fce());
    } catch (std::bad_cast& e) {
    }
  }
}
}  // namespace sire::simulator