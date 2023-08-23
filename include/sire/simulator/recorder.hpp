#ifndef SIRE_RECORDER_HPP_
#define SIRE_RECORDER_HPP_
#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::simulator {
/**
* 1. 假定只有Model中的数据需要被记录以支持仿真回放功能
* 2. 假定Model中只有ForcePool的结构会发生变化。
* 3. 假定运动过程中，机器人本身结构不会发生变化，环境不会发生变化
* 
* 其实需要做的事情与Model的CopyConstructor并无二质，
* 所以可以写一个接受两个Model指针的方法src复制到dest，然后dest init。
*/
class Record {
 public:

};

class Recorder : aris::core::NamedObject {
 public:
  auto registerRecord() -> void;
   auto virtual init() -> void{};
  Recorder() = default;
  virtual ~Recorder() = default;
  
 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::simulator
#endif